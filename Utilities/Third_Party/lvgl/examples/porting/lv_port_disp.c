/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>

#include "gd32f4xx.h"
#include "gd32f450i_lcd_eval.h"

/*********************
 *      DEFINES
 *********************/
#define LV_BUF_TYPE 2 /* 1 one buf 2 two buf 3 pingpong*/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);
static void disp_clean_dcache(lv_disp_drv_t * disp_drv);
/**********************
 *  STATIC VARIABLES
 **********************/
#if(LV_BUF_TYPE == 1)
/* TLI buf */
lv_color_t buf_disp[LV_HOR_RES_MAX * LV_VER_RES_MAX] __attribute__((section("SDRAM"))); 
/* LVGL buf */
#define LV_BUF_SIZE (LV_HOR_RES_MAX* 256/sizeof(lv_color_t))
lv_color_t buf_2_1[LV_BUF_SIZE] __attribute__((section("ADDSRAM")));
#elif(LV_BUF_TYPE==2)
/* TLI buf */
lv_color_t buf_disp[LV_HOR_RES_MAX * LV_VER_RES_MAX] __attribute__((section("SDRAM"))); 
/* LVGL buf */
#define LV_BUF_SIZE (LV_HOR_RES_MAX* 128/sizeof(lv_color_t))
lv_color_t buf_2_1[LV_BUF_SIZE] __attribute__((section("ADDSRAM")));
lv_color_t buf_2_2[LV_BUF_SIZE] __attribute__((section("ADDSRAM")));
#else
/* LVGL buf  TLI buf */
#define LV_BUF_SIZE (LV_HOR_RES_MAX * LV_VER_RES_MAX)
static lv_color_t buf_2_1[LV_HOR_RES_MAX * LV_VER_RES_MAX] __attribute__((section("SDRAM")));
static lv_color_t buf_2_2[LV_HOR_RES_MAX * LV_VER_RES_MAX] __attribute__((section("SDRAM")));
#endif

lv_disp_drv_t * g_disp_drv;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/
    static lv_disp_draw_buf_t draw_buf_dsc;
#if(LV_BUF_TYPE==1)
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_2_1, NULL, LV_BUF_SIZE);   /*Initialize the display buffer*/
#else
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_2_1, buf_2_2, LV_BUF_SIZE);   /*Initialize the display buffer*/
#endif
    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/
    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc;

    /*Required for Example 3)*/
#if(LV_BUF_TYPE==3)
    disp_drv.full_refresh = 1;
    //disp_drv.direct_mode = 1;
#endif /* LV_BUF_TYPE==3 */
    disp_drv.clean_dcache_cb = &disp_clean_dcache;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
    
#if(LV_BUF_TYPE == 3)
    nvic_irq_enable(TLI_IRQn, 2, 0);
#else
    nvic_irq_enable(IPA_IRQn, 2, 0);
#endif /* LV_BUF_TYPE==3 */
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
    /* ipa init */
    rcu_periph_clock_enable(RCU_IPA);
    /* initilize the LCD and layers */
    lcd_init();
#if(LV_BUF_TYPE==3)
    lcd_layer_init(LCD_LAYER_BACKGROUND, (uint32_t)buf_2_1, LV_HOR_RES_MAX, LV_VER_RES_MAX);
#else
    lcd_layer_init(LCD_LAYER_BACKGROUND, (uint32_t)buf_disp, LV_HOR_RES_MAX, LV_VER_RES_MAX);
#endif
    lcd_layer_enable(LCD_LAYER_BACKGROUND);
    tli_enable();
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/* A callback for cleaning any caches related to the display
 */
static void disp_clean_dcache(lv_disp_drv_t * disp_drv)
{
#if __CORTEX_M >= 0x07
    SCB_CleanInvalidateDCache();
#endif
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
#if(LV_BUF_TYPE==3)
    /* TLI buf switch */
    lcd_layer_init(LCD_LAYER_BACKGROUND, (uint32_t)color_p, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    tli_reload_config(TLI_REQUEST_RELOAD_EN);
    /* notice lvgl after TLI complete one frame*/
    tli_line_mark_set(LV_VER_RES_MAX-1);
    tli_interrupt_enable(TLI_INT_LM);
    g_disp_drv = disp_drv;
#else
    ipa_destination_parameter_struct  ipa_destination_init_struct;
    ipa_foreground_parameter_struct   ipa_fg_init_struct;

    /* configure IPA pixel format convert mode */
    ipa_pixel_format_convert_mode_set(IPA_FGTODE);
    /* configure destination pixel format */
    ipa_destination_init_struct.destination_pf = IPA_DPF_RGB565;
    /* configure destination memory base address */
    ipa_destination_init_struct.destination_memaddr = (uint32_t)((uint32_t)buf_disp + 2*(LV_HOR_RES_MAX*area->y1+area->x1));
    /* configure destination pre-defined alpha value RGB */
    ipa_destination_init_struct.destination_pregreen = 0;
    ipa_destination_init_struct.destination_preblue = 0;
    ipa_destination_init_struct.destination_prered = 0;
    ipa_destination_init_struct.destination_prealpha = 0;
    /* configure destination line offset */
    ipa_destination_init_struct.destination_lineoff = LV_HOR_RES_MAX - (area->x2 - area->x1 + 1);
    /* configure height of the image to be processed */
    ipa_destination_init_struct.image_height = (uint32_t)(area->y2 - area->y1 + 1);
    /* configure width of the image to be processed */
    ipa_destination_init_struct.image_width = (uint32_t)(area->x2 - area->x1 + 1);
    /* IPA destination initialization */
    ipa_destination_init(&ipa_destination_init_struct);

    /* configure IPA foreground */
    ipa_fg_init_struct.foreground_memaddr = (uint32_t)color_p;
    ipa_fg_init_struct.foreground_pf = FOREGROUND_PPF_RGB565;
    ipa_fg_init_struct.foreground_alpha_algorithm = IPA_FG_ALPHA_MODE_0;
    /* configure foreground pre-defined alpha value RGB */
    ipa_fg_init_struct.foreground_prealpha = 0;
    ipa_fg_init_struct.foreground_preblue = 0;
    ipa_fg_init_struct.foreground_pregreen = 0;
    ipa_fg_init_struct.foreground_prered = 0;
    /* configure foreground line offset */
    ipa_fg_init_struct.foreground_lineoff = 0;
    /* foreground initialization */
    ipa_foreground_init(&ipa_fg_init_struct);
    
    g_disp_drv = disp_drv;
    ipa_interrupt_enable(IPA_INT_FTF | IPA_INT_TAE | IPA_INT_WCF);
    ipa_transfer_enable();
#endif

}

#if(LV_BUF_TYPE==3)
void TLI_IRQHandler(void)
{
    if(RESET != tli_interrupt_flag_get(TLI_INT_FLAG_LM)){
        tli_interrupt_flag_clear(TLI_INT_FLAG_LM);
        tli_interrupt_disable(TLI_INT_LM);
        
        lv_disp_flush_ready(g_disp_drv);
    }
}
#else
void IPA_IRQHandler(void)
{
    if(RESET != ipa_interrupt_flag_get(IPA_INT_FLAG_FTF)){
        ipa_interrupt_flag_clear(IPA_INT_FLAG_FTF);
        /* IMPORTANT!!!
         * Inform the graphics library that you are ready with the flushing*/
        lv_disp_flush_ready(g_disp_drv);
    }
}
#endif /* LV_BUF_TYPE==3 */

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}


#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
