#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "exmc_sdram.h"

#include "lvgl.h"
#include "lv_port_indev.h"
#include "lv_port_disp.h"
#include "lv_demos.h"


int main(void)
{
    systick_config();
    
    exmc_synchronous_dynamic_ram_init(EXMC_SDRAM_DEVICE0);

    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    
		lv_demo_widgets();
	
    while(1) {
        delay_1ms(5);
        lv_timer_handler();
    }
}




