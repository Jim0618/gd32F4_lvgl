/**
 * @file lv_gpu_stm32_dma2d.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_gpu_stm32_dma2d.h"
#include "../../core/lv_refr.h"

#if LV_USE_GPU_STM32_DMA2D

#include LV_GPU_DMA2D_CMSIS_INCLUDE

/*********************
 *      DEFINES
 *********************/

#if LV_COLOR_16_SWAP
    // TODO: F7 has red blue swap bit in control register for all layers and output
    #error "Can't use DMA2D with LV_COLOR_16_SWAP 1"
#endif

#if LV_COLOR_DEPTH == 8
    #error "Can't use DMA2D with LV_COLOR_DEPTH == 8"
#endif

#if LV_COLOR_DEPTH == 16
    #define LV_DMA2D_COLOR_FORMAT LV_DMA2D_RGB565
#elif LV_COLOR_DEPTH == 32
    #define LV_DMA2D_COLOR_FORMAT LV_DMA2D_ARGB8888
#else
    /*Can't use GPU with other formats*/
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void lv_draw_stm32_dma2d_blend_fill(lv_color_t * dest_buf, lv_coord_t dest_stride, const lv_area_t * fill_area,
                                           lv_color_t color);


static void lv_draw_stm32_dma2d_blend_map(lv_color_t * dest_buf, const lv_area_t * dest_area, lv_coord_t dest_stride,
                                          const lv_color_t * src_buf, lv_coord_t src_stride, lv_opa_t opa);

static void lv_draw_stm32_dma2d_img_decoded(lv_draw_ctx_t * draw, const lv_draw_img_dsc_t * dsc,
                                            const lv_area_t * coords, const uint8_t * map_p, lv_img_cf_t color_format);


static void invalidate_cache(void);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Turn on the peripheral and set output color mode, this only needs to be done once
 */
void lv_draw_stm32_dma2d_init(void)
{
    RCU_AHB1EN |= RCU_AHB1EN_IPAEN;

    /*Wait for hardware access to complete*/
    __asm volatile("DSB\n");

    /* Delay after setting peripheral clock */
    volatile uint32_t temp = RCU_AHB1EN;
    LV_UNUSED(temp);

    /* set output colour mode */
    IPA_DPCTL = LV_DMA2D_COLOR_FORMAT;
}


void lv_draw_stm32_dma2d_ctx_init(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{

    lv_draw_sw_init_ctx(drv, draw_ctx);

    lv_draw_stm32_dma2d_ctx_t * dma2d_draw_ctx = (lv_draw_sw_ctx_t *)draw_ctx;

    dma2d_draw_ctx->blend = lv_draw_stm32_dma2d_blend;
    dma2d_draw_ctx->base_draw.wait_for_finish = lv_gpu_stm32_dma2d_wait_cb;
    dma2d_draw_ctx->base_draw.buffer_copy = lv_draw_stm32_dma2d_buffer_copy;

}

void lv_draw_stm32_dma2d_ctx_deinit(lv_disp_drv_t * drv, lv_draw_ctx_t * draw_ctx)
{
    LV_UNUSED(drv);
    LV_UNUSED(draw_ctx);
}


void lv_draw_stm32_dma2d_blend(lv_draw_ctx_t * draw_ctx, const lv_draw_sw_blend_dsc_t * dsc)
{
    lv_area_t blend_area;
    if(!_lv_area_intersect(&blend_area, dsc->blend_area, draw_ctx->clip_area)) return;

    bool done = false;

    if(dsc->mask_buf == NULL && dsc->blend_mode == LV_BLEND_MODE_NORMAL && lv_area_get_size(&blend_area) > 100) {
        lv_coord_t dest_stride = lv_area_get_width(draw_ctx->buf_area);

        lv_color_t * dest_buf = draw_ctx->buf;
        dest_buf += dest_stride * (blend_area.y1 - draw_ctx->buf_area->y1) + (blend_area.x1 - draw_ctx->buf_area->x1);

        const lv_color_t * src_buf = dsc->src_buf;
        if(src_buf) {
            lv_draw_sw_blend_basic(draw_ctx, dsc);
            lv_coord_t src_stride;
            src_stride = lv_area_get_width(dsc->blend_area);
            src_buf += src_stride * (blend_area.y1 - dsc->blend_area->y1) + (blend_area.x1 -  dsc->blend_area->x1);
            lv_area_move(&blend_area, -draw_ctx->buf_area->x1, -draw_ctx->buf_area->y1);
            lv_draw_stm32_dma2d_blend_map(dest_buf, &blend_area, dest_stride, src_buf, src_stride, dsc->opa);
            done = true;
        }
        else if(dsc->opa >= LV_OPA_MAX) {
            lv_area_move(&blend_area, -draw_ctx->buf_area->x1, -draw_ctx->buf_area->y1);
            lv_draw_stm32_dma2d_blend_fill(dest_buf, dest_stride, &blend_area, dsc->color);
            done = true;
        }
    }

    if(!done) lv_draw_sw_blend_basic(draw_ctx, dsc);
}

void lv_draw_stm32_dma2d_buffer_copy(lv_draw_ctx_t * draw_ctx,
                                     void * dest_buf, lv_coord_t dest_stride, const lv_area_t * dest_area,
                                     void * src_buf, lv_coord_t src_stride, const lv_area_t * src_area)
{
    LV_UNUSED(draw_ctx);
    lv_draw_stm32_dma2d_blend_map(dest_buf, dest_area, dest_stride, src_buf, src_stride, LV_OPA_MAX);
}


static void lv_draw_stm32_dma2d_img_decoded(lv_draw_ctx_t * draw_ctx, const lv_draw_img_dsc_t * dsc,
                                            const lv_area_t * coords, const uint8_t * map_p, lv_img_cf_t color_format)
{
    /*TODO basic ARGB8888 image can be handles here*/

    lv_draw_sw_img_decoded(draw_ctx, dsc, coords, map_p, color_format);
}

static void lv_draw_stm32_dma2d_blend_fill(lv_color_t * dest_buf, lv_coord_t dest_stride, const lv_area_t * fill_area,
                                           lv_color_t color)
{
    /*Simply fill an area*/
    int32_t area_w = lv_area_get_width(fill_area);
    int32_t area_h = lv_area_get_height(fill_area);
    invalidate_cache();

    IPA_CTL = 0x30000;
    IPA_DMADDR = (uint32_t)dest_buf;
    IPA_DPV = color.full;
    IPA_DLOFF = dest_stride - area_w;
    IPA_IMS = (area_w << 16) | (area_h << 0);
    
    /*start transfer*/
    IPA_CTL |= IPA_CTL_TEN;
}


static void lv_draw_stm32_dma2d_blend_map(lv_color_t * dest_buf, const lv_area_t * dest_area, lv_coord_t dest_stride,
                                          const lv_color_t * src_buf, lv_coord_t src_stride, lv_opa_t opa)
{

    /*Simple copy*/
    int32_t dest_w = lv_area_get_width(dest_area);
    int32_t dest_h = lv_area_get_height(dest_area);

    invalidate_cache();
    if(opa >= LV_OPA_MAX) {

        IPA_CTL = 0x00000;
        IPA_FPCTL = LV_DMA2D_COLOR_FORMAT;
        IPA_FMADDR = (uint32_t)src_buf;
        IPA_FLOFF = src_stride - dest_w;
        IPA_DMADDR = (uint32_t)dest_buf;
        IPA_DLOFF = dest_stride - dest_w;
        IPA_IMS = (dest_w << 16) | (dest_h << 0);
        
        /*start transfer*/
        IPA_CTL |= IPA_CTL_TEN;
    }
    else {

        IPA_CTL = 0x20000;
        IPA_BPCTL = LV_DMA2D_COLOR_FORMAT;
        IPA_BMADDR = (uint32_t)dest_buf;
        IPA_BLOFF = dest_stride - dest_w;
        
        IPA_FPCTL = (uint32_t)LV_DMA2D_COLOR_FORMAT
                         /* alpha mode 2, replace with foreground * alpha value */
                         | (2 << 16)
                         /* alpha value */
                         | (opa << 24);
        IPA_FMADDR = (uint32_t)src_buf;
        IPA_FLOFF = src_stride - dest_w;
        
        IPA_DMADDR = (uint32_t)dest_buf;
        IPA_DLOFF = dest_stride - dest_w;
        IPA_IMS = (dest_w << 16) | (dest_h << 0);
        
        IPA_CTL |= IPA_CTL_TEN;
    }
}

void lv_gpu_stm32_dma2d_wait_cb(lv_draw_ctx_t * draw_ctx)
{
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    if(disp->driver && disp->driver->wait_cb) {
        while(IPA_CTL & IPA_CTL_TEN) {
            disp->driver->wait_cb(disp->driver);
        }
    }
    else {
        while(IPA_CTL & IPA_CTL_TEN);
    }
    lv_draw_sw_wait_for_finish(draw_ctx);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void invalidate_cache(void)
{
    lv_disp_t * disp = _lv_refr_get_disp_refreshing();
    if(disp->driver->clean_dcache_cb) disp->driver->clean_dcache_cb(disp->driver);
    else {
#if __CORTEX_M >= 0x07
        if((SCB->CCR) & (uint32_t)SCB_CCR_DC_Msk)
            SCB_CleanInvalidateDCache();
#endif
    }
}

#endif
