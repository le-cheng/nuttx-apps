/****************************************************************************
 * apps/examples/dualscreen_lvgl/dualscreen_lvgl.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <nuttx/video/fb.h>
#include <nuttx/wqueue.h>

#include <lvgl/lvgl.h>
#include <lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.h>
#include <time.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef NEED_BOARDINIT
#if defined(CONFIG_BOARDCTL) && !defined(CONFIG_NSH_ARCHINIT)
#  define NEED_BOARDINIT 1
#endif

/* 屏幕配置 - 每个物理屏幕 960x720 */
#define SCREEN_WIDTH      960
#define SCREEN_HEIGHT     720
#define VIRTUAL_WIDTH     (SCREEN_WIDTH * 2)  /* 1920 */
#define VIRTUAL_HEIGHT    SCREEN_HEIGHT       /* 720 */
#define COLOR_DEPTH       32
#define BYTES_PER_PIXEL   (COLOR_DEPTH / 8)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* 模拟 DMA 2D 传输配置结构体
 * 在真实硬件上，这些参数对应 DMA2D/PXP 等外设的寄存器配置
 */
typedef struct
{
  const void *src;        /* 源地址 */
  void *dst;              /* 目的地址 */
  uint32_t width;         /* 区域宽度 (像素) */
  uint32_t height;        /* 区域高度 (行数) */
  uint32_t src_stride;    /* 源缓冲区跨度 (字节) */
  uint32_t dst_stride;    /* 目的缓冲区跨度 (字节) */
  uint32_t bpp;           /* 每像素字节数 */
} dma_2d_config_t;

/* DMA 请求结构体 */

typedef struct
{
  struct work_s work;
  dma_2d_config_t cfg;
  lv_display_t *disp;
  bool notify;
} dma_request_t;

/* 双屏驱动数据结构 */

typedef struct
{
  int fd_left;                    /* 左屏 framebuffer 文件描述符 */
  int fd_right;                   /* 右屏 framebuffer 文件描述符 */
  void *mem_left;                 /* 左屏内存映射 */
  void *mem_right;                /* 右屏内存映射 */
  uint32_t stride_left;           /* 左屏每行字节数 */
  uint32_t stride_right;          /* 右屏每行字节数 */
  size_t fblen_left;              /* 左屏 framebuffer 大小 */
  size_t fblen_right;             /* 右屏 framebuffer 大小 */
  void *draw_buf;                 /* LVGL 绘制缓冲区 */
  void *draw_buf2;                /* LVGL 第二绘制缓冲区(双缓冲) */
  dma_request_t dma_req[2];       /* DMA 请求工作结构 */
} dualscreen_ctx_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dualscreen_ctx_t g_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NuttX tick 回调函数 */

static uint32_t millis(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint32_t tick = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
  return tick;
}

/****************************************************************************
 * Name: init_framebuffer
 *
 * Description:
 *   打开并初始化单个 framebuffer
 *
 ****************************************************************************/

static int init_framebuffer(const char *path, int *fd, void **mem,
                            uint32_t *stride, size_t *fblen)
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;

  *fd = open(path, O_RDWR);
  if (*fd < 0)
    {
      printf("Error: cannot open %s: %d\n", path, errno);
      return -1;
    }

  if (ioctl(*fd, FBIOGET_VIDEOINFO, &vinfo) < 0)
    {
      printf("Error: FBIOGET_VIDEOINFO failed: %d\n", errno);
      close(*fd);
      return -1;
    }

  if (ioctl(*fd, FBIOGET_PLANEINFO, &pinfo) < 0)
    {
      printf("Error: FBIOGET_PLANEINFO failed: %d\n", errno);
      close(*fd);
      return -1;
    }

  *mem = mmap(NULL, pinfo.fblen, PROT_READ | PROT_WRITE,
              MAP_SHARED | MAP_FILE, *fd, 0);
  if (*mem == MAP_FAILED)
    {
      printf("Error: mmap failed: %d\n", errno);
      close(*fd);
      return -1;
    }

  *stride = pinfo.stride;
  *fblen = pinfo.fblen;

  return 0;
}

/****************************************************************************
 * Name: dma_worker
 *
 * Description:
 *   工作队列回调，执行实际的内存拷贝并通知完成
 *
 ****************************************************************************/

static void dma_worker(FAR void *arg)
{
  dma_request_t *req = (dma_request_t *)arg;
  const uint8_t *s = (const uint8_t *)req->cfg.src;
  uint8_t *d = (uint8_t *)req->cfg.dst;
  uint32_t line_bytes = req->cfg.width * req->cfg.bpp;
  uint32_t h = req->cfg.height;

  while (h--)
    {
      memcpy(d, s, line_bytes);
      s += req->cfg.src_stride;
      d += req->cfg.dst_stride;
    }

  if (req->notify && req->disp)
    {
      lv_display_flush_ready(req->disp);
    }
}

/****************************************************************************
 * Name: sim_dma_transfer_2d
 *
 * Description:
 *   模拟 DMA 2D 传输操作。
 *   使用工作队列模拟异步传输。
 *
 ****************************************************************************/

static void sim_dma_transfer_2d(const dma_2d_config_t *cfg, lv_display_t *disp,
                                bool notify, int req_idx)
{
  dualscreen_ctx_t *ctx = lv_display_get_driver_data(disp);
  dma_request_t *req = &ctx->dma_req[req_idx];

  /* 复制配置到请求结构 */
  req->cfg = *cfg;
  req->disp = disp;
  req->notify = notify;

  /* 提交到系统高优先级工作队列 */
  work_queue(HPWORK, &req->work, dma_worker, req, 0);
}

/****************************************************************************
 * Name: dualscreen_flush_cb
 *
 * Description:
 *   自定义 flush 回调 - 使用模拟 DMA 接口进行分发
 *
 ****************************************************************************/

static void dualscreen_flush_cb(lv_display_t *disp, const lv_area_t *area,
                                uint8_t *color_p)
{
  dualscreen_ctx_t *ctx = lv_display_get_driver_data(disp);

  int32_t x1 = area->x1;
  int32_t y1 = area->y1;
  int32_t x2 = area->x2;
  int32_t y2 = area->y2;
  int32_t h = y2 - y1 + 1;

  /* 源缓冲区（LVGL 绘制缓冲区）的跨度是固定的：虚拟屏幕宽度 * BPP */
  uint32_t src_stride = VIRTUAL_WIDTH * BYTES_PER_PIXEL;

  /* 检查是否需要更新右屏，用于决定左屏更新是否是最后一步 */
  bool update_right = (x2 >= SCREEN_WIDTH);

  /* 处理左屏 (x: 0-959) */

  if (x1 < SCREEN_WIDTH)
    {
      int32_t left_x1 = x1;
      int32_t left_x2 = (x2 < SCREEN_WIDTH) ? x2 : (SCREEN_WIDTH - 1);
      int32_t left_w = left_x2 - left_x1 + 1;

      /* 配置模拟 DMA 传输参数 */
      dma_2d_config_t dma_cfg;

      /* 源地址：LVGL buffer 中的起始点 */
      dma_cfg.src = color_p + (y1 * VIRTUAL_WIDTH + x1) * BYTES_PER_PIXEL;

      /* 目的地址：左屏 Framebuffer 中的起始点 */
      dma_cfg.dst = (uint8_t *)ctx->mem_left +
                    y1 * ctx->stride_left + left_x1 * BYTES_PER_PIXEL;

      dma_cfg.width = left_w;
      dma_cfg.height = h;
      dma_cfg.src_stride = src_stride;
      dma_cfg.dst_stride = ctx->stride_left;
      dma_cfg.bpp = BYTES_PER_PIXEL;

      /* 启动传输 - 如果不更新右屏，则这是最后一步，需要通知 */
      sim_dma_transfer_2d(&dma_cfg, disp, !update_right, 0);
    }

  /* 处理右屏 (x: 960-1919) */

  if (update_right)
    {
      int32_t right_x1 = (x1 >= SCREEN_WIDTH) ? (x1 - SCREEN_WIDTH) : 0;
      int32_t right_x2 = x2 - SCREEN_WIDTH;
      int32_t right_w = right_x2 - right_x1 + 1;
      int32_t src_x1 = (x1 >= SCREEN_WIDTH) ? x1 : SCREEN_WIDTH;

      /* 配置 DMA 传输参数 */
      dma_2d_config_t dma_cfg;

      /* 源地址：LVGL buffer 中的起始点 (注意 src_x1 的计算) */
      dma_cfg.src = color_p + (y1 * VIRTUAL_WIDTH + src_x1) * BYTES_PER_PIXEL;

      /* 目的地址：右屏 Framebuffer 中的起始点 */
      dma_cfg.dst = (uint8_t *)ctx->mem_right +
                    y1 * ctx->stride_right + right_x1 * BYTES_PER_PIXEL;

      dma_cfg.width = right_w;
      dma_cfg.height = h;
      dma_cfg.src_stride = src_stride;
      dma_cfg.dst_stride = ctx->stride_right;
      dma_cfg.bpp = BYTES_PER_PIXEL;

      /* 启动传输 - 右屏总是最后一步（如果发生），需要通知 */
      sim_dma_transfer_2d(&dma_cfg, disp, true, 1);
    }
}

/****************************************************************************
 * Name: create_dualscreen_display
 *
 * Description:
 *   创建双屏显示，使用双缓冲和 DIRECT 渲染模式
 *
 ****************************************************************************/

static lv_display_t *create_dualscreen_display(void)
{
  dualscreen_ctx_t *ctx = &g_ctx;
  size_t buf_size;

  /* 初始化左屏 /dev/fb0
   * mem_left是显示屏物理地址
  */

  if (init_framebuffer("/dev/fb0", &ctx->fd_left, &ctx->mem_left,
                       &ctx->stride_left, &ctx->fblen_left) < 0)
    {
      return NULL;
    }

  /* 初始化右屏 /dev/fb1 */

  if (init_framebuffer("/dev/fb1", &ctx->fd_right, &ctx->mem_right,
                       &ctx->stride_right, &ctx->fblen_right) < 0)
    {
      munmap(ctx->mem_left, ctx->fblen_left);
      close(ctx->fd_left);
      return NULL;
    }

  /* 分配 LVGL 绘制缓冲区 (虚拟大屏尺寸, 双缓冲) */

  buf_size = VIRTUAL_WIDTH * VIRTUAL_HEIGHT * BYTES_PER_PIXEL;
  ctx->draw_buf = malloc(buf_size);
  if (!ctx->draw_buf)
    {
      munmap(ctx->mem_left, ctx->fblen_left);
      munmap(ctx->mem_right, ctx->fblen_right);
      close(ctx->fd_left);
      close(ctx->fd_right);
      return NULL;
    }

  ctx->draw_buf2 = malloc(buf_size);
  if (!ctx->draw_buf2)
    {
      free(ctx->draw_buf);
      munmap(ctx->mem_left, ctx->fblen_left);
      munmap(ctx->mem_right, ctx->fblen_right);
      close(ctx->fd_left);
      close(ctx->fd_right);
      return NULL;
    }

  /* 创建 LVGL 显示 */

  lv_display_t *disp = lv_display_create(VIRTUAL_WIDTH, VIRTUAL_HEIGHT);
  if (!disp)
    {
      free(ctx->draw_buf);
      free(ctx->draw_buf2);
      munmap(ctx->mem_left, ctx->fblen_left);
      munmap(ctx->mem_right, ctx->fblen_right);
      close(ctx->fd_left);
      close(ctx->fd_right);
      return NULL;
    }

  /* 设置显示参数 */

  lv_display_set_driver_data(disp, ctx);
  lv_display_set_flush_cb(disp, dualscreen_flush_cb);

  /* 设置双缓冲和 DIRECT 渲染模式 */

  lv_display_set_buffers(disp, ctx->draw_buf, ctx->draw_buf2, buf_size,
                         LV_DISPLAY_RENDER_MODE_DIRECT);

  return disp;
}

/****************************************************************************
 * Name: destroy_dualscreen_display
 *
 * Description:
 *   清理双屏显示资源
 *
 ****************************************************************************/

static void destroy_dualscreen_display(lv_display_t *disp)
{
  dualscreen_ctx_t *ctx = &g_ctx;

  if (disp)
    {
      lv_display_delete(disp);
    }

  if (ctx->draw_buf)
    {
      free(ctx->draw_buf);
      ctx->draw_buf = NULL;
    }

  if (ctx->draw_buf2)
    {
      free(ctx->draw_buf2);
      ctx->draw_buf2 = NULL;
    }

  if (ctx->mem_left)
    {
      munmap(ctx->mem_left, ctx->fblen_left);
      ctx->mem_left = NULL;
    }

  if (ctx->mem_right)
    {
      munmap(ctx->mem_right, ctx->fblen_right);
      ctx->mem_right = NULL;
    }

  if (ctx->fd_left >= 0)
    {
      close(ctx->fd_left);
      ctx->fd_left = -1;
    }

  if (ctx->fd_right >= 0)
    {
      close(ctx->fd_right);
      ctx->fd_right = -1;
    }
}

/****************************************************************************
 * Name: create_test_ui
 *
 * Description:
 *   创建测试 UI - 包含各种测试组件，跨越两个屏幕
 *
 ****************************************************************************/

static void create_test_ui(void)
{
  lv_obj_t *scr = lv_screen_active();

  /* 设置背景渐变色 - 从左到右渐变 */

  lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_bg_grad_color(scr, lv_color_hex(0x16213e), 0);
  lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_HOR, 0);

  /* ========== 标题 (跨越两屏中央) ========== */

  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "Dual Screen LVGL Demo - Virtual Large Screen");
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  /* 副标题 - 显示技术参数 */

  lv_obj_t *subtitle = lv_label_create(scr);
  lv_label_set_text(subtitle,
                    "Double Buffer | DIRECT Mode | 1920x720 Virtual");
  lv_obj_set_style_text_color(subtitle, lv_color_hex(0x888888), 0);
  lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 55);

  /* ========== 左屏内容 (Screen 1: /dev/fb0) ========== */

  /* 左屏标题 */

  lv_obj_t *left_title = lv_label_create(scr);
  lv_label_set_text(left_title, "SCREEN 1 (/dev/fb0)");
  lv_obj_set_style_text_color(left_title, lv_color_hex(0x4ECDC4), 0);
  lv_obj_set_style_text_font(left_title, &lv_font_montserrat_20, 0);
  lv_obj_set_pos(left_title, 350, 100);

  /* 左屏面板 */

  lv_obj_t *left_panel = lv_obj_create(scr);
  lv_obj_set_size(left_panel, 400, 280);
  lv_obj_set_pos(left_panel, 280, 140);
  lv_obj_set_style_bg_color(left_panel, lv_color_hex(0x2d2d44), 0);
  lv_obj_set_style_radius(left_panel, 15, 0);
  lv_obj_set_style_border_width(left_panel, 2, 0);
  lv_obj_set_style_border_color(left_panel, lv_color_hex(0x4ECDC4), 0);
  lv_obj_set_scrollbar_mode(left_panel, LV_SCROLLBAR_MODE_OFF);

  /* 按钮 1 */

  lv_obj_t *btn1 = lv_button_create(left_panel);
  lv_obj_set_size(btn1, 150, 50);
  lv_obj_set_pos(btn1, 20, 20);
  lv_obj_set_style_bg_color(btn1, lv_color_hex(0xFF6B6B), 0);
  lv_obj_t *btn1_label = lv_label_create(btn1);
  lv_label_set_text(btn1_label, "Button 1");
  lv_obj_center(btn1_label);

  /* 按钮 2 */

  lv_obj_t *btn2 = lv_button_create(left_panel);
  lv_obj_set_size(btn2, 150, 50);
  lv_obj_set_pos(btn2, 200, 20);
  lv_obj_set_style_bg_color(btn2, lv_color_hex(0x4ECDC4), 0);
  lv_obj_t *btn2_label = lv_label_create(btn2);
  lv_label_set_text(btn2_label, "Button 2");
  lv_obj_center(btn2_label);

  /* 滑块 */

  lv_obj_t *slider1 = lv_slider_create(left_panel);
  lv_obj_set_width(slider1, 320);
  lv_obj_set_pos(slider1, 20, 90);
  lv_slider_set_value(slider1, 60, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider1, lv_color_hex(0x444455), 0);
  lv_obj_set_style_bg_color(slider1, lv_color_hex(0xFF6B6B), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider1, lv_color_hex(0xFFFFFF), LV_PART_KNOB);

  lv_obj_t *slider1_label = lv_label_create(left_panel);
  lv_label_set_text(slider1_label, "Slider: 60%");
  lv_obj_set_style_text_color(slider1_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(slider1_label, 20, 120);

  /* 进度条 */

  lv_obj_t *bar1 = lv_bar_create(left_panel);
  lv_obj_set_size(bar1, 320, 25);
  lv_obj_set_pos(bar1, 20, 160);
  lv_bar_set_value(bar1, 75, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar1, lv_color_hex(0x444455), 0);
  lv_obj_set_style_bg_color(bar1, lv_color_hex(0x4ECDC4), LV_PART_INDICATOR);

  lv_obj_t *bar1_label = lv_label_create(left_panel);
  lv_label_set_text(bar1_label, "Progress: 75%");
  lv_obj_set_style_text_color(bar1_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(bar1_label, 20, 195);

  /* 开关 */

  lv_obj_t *sw1 = lv_switch_create(left_panel);
  lv_obj_set_pos(sw1, 20, 230);
  lv_obj_add_state(sw1, LV_STATE_CHECKED);

  lv_obj_t *sw1_label = lv_label_create(left_panel);
  lv_label_set_text(sw1_label, "Switch: ON");
  lv_obj_set_style_text_color(sw1_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(sw1_label, 90, 235);

  /* 复选框 */

  lv_obj_t *cb1 = lv_checkbox_create(left_panel);
  lv_checkbox_set_text(cb1, "Checkbox");
  lv_obj_set_style_text_color(cb1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(cb1, 200, 230);
  lv_obj_add_state(cb1, LV_STATE_CHECKED);

  /* ========== 右屏内容 (Screen 2: /dev/fb1) ========== */

  /* 右屏标题 */

  lv_obj_t *right_title = lv_label_create(scr);
  lv_label_set_text(right_title, "SCREEN 2 (/dev/fb1)");
  lv_obj_set_style_text_color(right_title, lv_color_hex(0xFFE66D), 0);
  lv_obj_set_style_text_font(right_title, &lv_font_montserrat_20, 0);
  lv_obj_set_pos(right_title, 1310, 100);

  /* 右屏面板 */

  lv_obj_t *right_panel = lv_obj_create(scr);
  lv_obj_set_size(right_panel, 400, 280);
  lv_obj_set_pos(right_panel, 1240, 140);
  lv_obj_set_style_bg_color(right_panel, lv_color_hex(0x2d2d44), 0);
  lv_obj_set_style_radius(right_panel, 15, 0);
  lv_obj_set_style_border_width(right_panel, 2, 0);
  lv_obj_set_style_border_color(right_panel, lv_color_hex(0xFFE66D), 0);
  lv_obj_set_scrollbar_mode(right_panel, LV_SCROLLBAR_MODE_OFF);

  /* 圆弧 (Arc) */

  lv_obj_t *arc1 = lv_arc_create(right_panel);
  lv_obj_set_size(arc1, 120, 120);
  lv_obj_set_pos(arc1, 20, 20);
  lv_arc_set_value(arc1, 70);
  lv_obj_set_style_arc_color(arc1, lv_color_hex(0x444455), 0);
  lv_obj_set_style_arc_color(arc1, lv_color_hex(0xFFE66D), LV_PART_INDICATOR);

  lv_obj_t *arc1_label = lv_label_create(right_panel);
  lv_label_set_text(arc1_label, "Arc: 70%");
  lv_obj_set_style_text_color(arc1_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(arc1_label, 45, 145);

  /* 旋钮式滑块 */

  lv_obj_t *arc2 = lv_arc_create(right_panel);
  lv_obj_set_size(arc2, 120, 120);
  lv_obj_set_pos(arc2, 160, 20);
  lv_arc_set_value(arc2, 45);
  lv_obj_set_style_arc_color(arc2, lv_color_hex(0x444455), 0);
  lv_obj_set_style_arc_color(arc2, lv_color_hex(0x9B59B6), LV_PART_INDICATOR);

  lv_obj_t *arc2_label = lv_label_create(right_panel);
  lv_label_set_text(arc2_label, "Knob: 45%");
  lv_obj_set_style_text_color(arc2_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_pos(arc2_label, 185, 145);

  /* 滚轮 (Roller) */

  lv_obj_t *roller1 = lv_roller_create(right_panel);
  lv_roller_set_options(roller1,
                        "Option 1\n"
                        "Option 2\n"
                        "Option 3\n"
                        "Option 4\n"
                        "Option 5",
                        LV_ROLLER_MODE_INFINITE);
  lv_obj_set_size(roller1, 120, 100);
  lv_obj_set_pos(roller1, 260, 40);
  lv_roller_set_selected(roller1, 2, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(roller1, lv_color_hex(0x333344), 0);
  lv_obj_set_style_text_color(roller1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_bg_color(roller1, lv_color_hex(0xFFE66D),
                            LV_PART_SELECTED);

  /* 按钮矩阵 */

  static const char *btnm_map[] =
  {
    "1", "2", "3", "\n",
    "4", "5", "6", "\n",
    "7", "8", "9", ""
  };

  lv_obj_t *btnm = lv_buttonmatrix_create(right_panel);
  lv_buttonmatrix_set_map(btnm, btnm_map);
  lv_obj_set_size(btnm, 160, 100);
  lv_obj_set_pos(btnm, 20, 170);
  lv_obj_set_style_bg_color(btnm, lv_color_hex(0x333344), 0);
  lv_obj_set_style_bg_color(btnm, lv_color_hex(0x555566), LV_PART_ITEMS);
  lv_obj_set_style_text_color(btnm, lv_color_hex(0xFFFFFF), LV_PART_ITEMS);

  /* 下拉框 */

  lv_obj_t *dd = lv_dropdown_create(right_panel);
  lv_dropdown_set_options(dd, "Item 1\nItem 2\nItem 3\nItem 4");
  lv_obj_set_size(dd, 160, 40);
  lv_obj_set_pos(dd, 200, 170);
  lv_obj_set_style_bg_color(dd, lv_color_hex(0x444455), 0);
  lv_obj_set_style_text_color(dd, lv_color_hex(0xFFFFFF), 0);

  /* 标签 */

  lv_obj_t *info_label = lv_label_create(right_panel);
  lv_label_set_text(info_label, "X: 960-1919");
  lv_obj_set_style_text_color(info_label, lv_color_hex(0x888888), 0);
  lv_obj_set_pos(info_label, 200, 230);

  /* ========== 跨屏元素 - 横跨两个物理屏幕 ========== */

  /* 跨屏面板 - 位于屏幕分界线两侧 */

  lv_obj_t *cross_panel = lv_obj_create(scr);
  lv_obj_set_size(cross_panel, 500, 120);
  lv_obj_set_pos(cross_panel, 710, 450);  /* 横跨 x=960 分界线 */
  lv_obj_set_style_bg_color(cross_panel, lv_color_hex(0x9B59B6), 0);
  lv_obj_set_style_bg_grad_color(cross_panel, lv_color_hex(0x667eea), 0);
  lv_obj_set_style_bg_grad_dir(cross_panel, LV_GRAD_DIR_HOR, 0);
  lv_obj_set_style_radius(cross_panel, 20, 0);
  lv_obj_set_style_border_width(cross_panel, 3, 0);
  lv_obj_set_style_border_color(cross_panel, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_scrollbar_mode(cross_panel, LV_SCROLLBAR_MODE_OFF);

  lv_obj_t *cross_title = lv_label_create(cross_panel);
  lv_label_set_text(cross_title, "CROSS-SCREEN ELEMENT");
  lv_obj_set_style_text_color(cross_title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(cross_title, &lv_font_montserrat_18, 0);
  lv_obj_align(cross_title, LV_ALIGN_TOP_MID, 0, 15);

  lv_obj_t *cross_desc = lv_label_create(cross_panel);
  lv_label_set_text(cross_desc, "This panel spans both screens\nX: 710-1210 | Boundary at X=960");
  lv_obj_set_style_text_color(cross_desc, lv_color_hex(0xDDDDDD), 0);
  lv_obj_set_style_text_align(cross_desc, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(cross_desc, LV_ALIGN_CENTER, 0, 10);

  /* 分界线 - 可视化显示两个屏幕的边界 */

  lv_obj_t *divider = lv_obj_create(scr);
  lv_obj_set_size(divider, 4, VIRTUAL_HEIGHT);
  lv_obj_set_pos(divider, 958, 0);
  lv_obj_set_style_bg_color(divider, lv_color_hex(0xFF6B6B), 0);
  lv_obj_set_style_bg_opa(divider, LV_OPA_70, 0);
  lv_obj_set_style_radius(divider, 0, 0);
  lv_obj_set_style_border_width(divider, 0, 0);

  /* 分界线标签 */

  lv_obj_t *div_label = lv_label_create(scr);
  lv_label_set_text(div_label, "Screen\nBoundary\nX=960");
  lv_obj_set_style_text_color(div_label, lv_color_hex(0xFF6B6B), 0);
  lv_obj_set_style_text_align(div_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(div_label, 890, 590);

  /* ========== 底部信息栏 ========== */

  lv_obj_t *footer = lv_label_create(scr);
  lv_label_set_text(footer,
    "Virtual Screen: 1920x720 | Physical: 2x 960x720 | "
    "Mode: DIRECT | Buffer: Double");
  lv_obj_set_style_text_color(footer, lv_color_hex(0x666666), 0);
  lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -15);

  /* 左下角坐标信息 */

  lv_obj_t *left_info = lv_label_create(scr);
  lv_label_set_text(left_info, "fb0: (0,0)");
  lv_obj_set_style_text_color(left_info, lv_color_hex(0x4ECDC4), 0);
  lv_obj_set_pos(left_info, 20, 680);

  /* 右下角坐标信息 */

  lv_obj_t *right_info = lv_label_create(scr);
  lv_label_set_text(right_info, "fb1: (1919,719)");
  lv_obj_set_style_text_color(right_info, lv_color_hex(0xFFE66D), 0);
  lv_obj_set_pos(right_info, 1780, 680);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   双屏 LVGL 演示程序入口
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  lv_display_t *disp;
  lv_indev_t *indev;

  if (lv_is_initialized())
    {
      printf("LVGL already initialized!\n");
      return -1;
    }

#ifdef NEED_BOARDINIT
  boardctl(BOARDIOC_INIT, 0);
#endif

  /* 初始化 LVGL */

  lv_init();

  /* 设置 NuttX tick 回调（必须，否则 LVGL 定时器和输入读取不会工作） */

  lv_tick_set_cb(millis);

  /* 创建双屏显示 */

  disp = create_dualscreen_display();
  if (!disp)
    {
      printf("Failed to create dual screen display!\n");
      lv_deinit();
      return 1;
    }

  /* 初始化触摸屏输入设备 */

  indev = lv_nuttx_touchscreen_create("/dev/input0");
  if (!indev)
    {
      printf("Warning: Failed to create touchscreen input device 0\n");
    }
  else
    {
      /* 关键：将输入设备显式关联到我们的虚拟大屏显示器 */

      lv_indev_set_display(indev, disp);
    }

  indev = lv_nuttx_touchscreen_create("/dev/input1");
  if (!indev)
    {
      printf("Warning: Failed to create touchscreen input device 1\n");
    }
  else
    {
      lv_indev_set_display(indev, disp);
    }

  /* 创建测试 UI */

  create_test_ui();

  /* 主循环 */

  while (1)
    {
      uint32_t idle = lv_timer_handler();
      idle = idle ? idle : 1;
      usleep(idle * 1000);
    }

  /* 清理 */

  destroy_dualscreen_display(disp);
  lv_deinit();

  return 0;
}

