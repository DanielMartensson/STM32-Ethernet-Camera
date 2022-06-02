/*
 * ov7670.c
 *
 *  Created on: May 7, 2022
 *      Author: Daniel Mårtensson
 */

#include "ov7670.h"

static I2C_HandleTypeDef *_hi2c;

const uint8_t QVGA_30_fps_YUV_settings[] = {
		0x11, 0x01, /* CLKRC */
		0x12, 0x00, /* COM7 */
		0x0C, 0x04, /* COM3 */
		0x3E, 0x19, /* COM14 */
		0x70, 0x3A, /* SCALING_XSC */
		0x71, 0x35, /* SCALING_YSC */
		0x72, 0x11, /* SCALING_DCWCTR */
		0x73, 0xF1, /* SCALING_PCLK_DIV */
		0xA2, 0x02  /* SCALING_PCLK_DELAY */
};

const uint8_t QVGA_30_fps_RGB_settings[] = {
		0x11, 0x01, /* CLKRC */
		0x12, 0x11, /* COM7 */
		0x0C, 0x04, /* COM3 */
		0x3E, 0x1A, /* COM14 */
		0x70, 0x3A, /* SCALING_XSC */
		0x71, 0x35, /* SCALING_YSC */
		0x72, 0x11, /* SCALING_DCWCTR */
		0x73, 0xF9, /* SCALING_PCLK_DIV */
		0xA2, 0x02  /* SCALING_PCLK_DELAY */
};

void ov7670_init(I2C_HandleTypeDef *hi2c){
	_hi2c = hi2c;


}

