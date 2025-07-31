/*
 * OV2640.c
 * OV2640 DCMI DMA Driver
 * Created on: Aug 1, 2025
 *
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
//#include "OV7670.h"

/******************************************************************************
 *                               LOCAL MACRO                                  *
 ******************************************************************************/
//1 pixel = 2 bytes
//Because RGB565 format is used.
/*
RGB565 means:
R (Red): 5 bits

G (Green): 6 bits

B (Blue): 5 bits
Total: 5 + 6 + 5 = 16 bits = 2 bytes
*/


#define OV2640_SCCB_ADDR              (0x42U) // OLD


#define OV2640_WIDTH                  (320U)  // QVGA size
#define OV2640_HEIGHT                 (240U)

#define OV2640_RGB565_BYTES           (2U)    // 16 bits = 2 bytes per pixel
#define OV2640_LINES_IN_CHUNK        (1U)    // DMA one line at a time

#define OV2640_WIDTH_SIZE_BYTES      (OV2640_WIDTH * OV2640_RGB565_BYTES) //Total bytes in one row (line) of the image. //For QVGA: 320 × 2 = 640 bytes per line.
#define OV2640_WIDTH_SIZE_WORDS      (OV2640_WIDTH_SIZE_BYTES / 4U)       //640 / 4 = 160 words.

#define OV2640_HEIGHT_SIZE_BYTES     (OV2640_HEIGHT * OV2640_RGB565_BYTES)
#define OV2640_HEIGHT_SIZE_WORDS     (OV2640_HEIGHT_SIZE_BYTES / 4U)

#define OV2640_FRAME_SIZE_BYTES      (OV2640_WIDTH * OV2640_HEIGHT * OV2640_RGB565_BYTES)
#define OV2640_FRAME_SIZE_WORDS      (OV2640_FRAME_SIZE_BYTES / 4U)

/* For double stream-line buffer */
#define OV2640_BUFFER_SIZE            (OV2640_WIDTH_SIZE_BYTES * OV2640_RGB565_BYTES * OV2640_LINES_IN_CHUNK) // 2 LINES!!! 1280 - size for global buffer


#define OV2640_DMA_DATA_LEN           (OV2640_WIDTH_SIZE_WORDS * OV2640_LINES_IN_CHUNK)
/* Macro for update address to second half of double-line buffer */ //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OV2640_SWITCH_BUFFER()        ((OV2640.buffer_addr != (uint32_t)buffer) ? (OV2640.buffer_addr + (OV7670_BUFFER_SIZE)/ 2U) : (uint32_t)buffer)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OV2640_RESET_BUFFER_ADDR()    (uint32_t)buffer // BIND with global Buufer

#define OV2640_START_XLK(htim, channel)\
        do{SET_BIT(htim->Instance->CCER, (0x1UL << channel));\
        SET_BIT(htim->Instance->CR1, TIM_CR1_CEN);}while(0)

#define OV2640_STOP_XLK(htim, channel)\
        do{CLEAR_BIT(htim->Instance->CCER, (0x1UL << channel));\
        CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);}while(0)


#define OV2640_REG_GAIN                 0x00  // AGC – Gain control
#define OV2640_REG_COM1                 0x03  // Common control 1
#define OV2640_REG_REG04                0x04  // Mirror/VFlip
#define OV2640_REG_AEC                  0x10  // Exposure value
#define OV2640_REG_CLKRC                0x11  // Internal clock control
#define OV2640_REG_COM7                0x12  // Common control 7 (reset, format select)
#define OV2640_REG_COM8                0x13  // Enable AEC, AGC, AWB
#define OV2640_REG_COM9                0x14  // AGC gain ceiling
#define OV2640_REG_COM10               0x15  // PCLK, VSYNC, HREF polarity
#define OV2640_REG_HSTART              0x17  // Horizontal frame start
#define OV2640_REG_HSTOP               0x18  // Horizontal frame stop
#define OV2640_REG_VSTART              0x19  // Vertical frame start
#define OV2640_REG_VSTOP               0x1A  // Vertical frame stop
#define OV2640_REG_MIDH                0x1C  // Manufacturer ID high byte (0x7F)
#define OV2640_REG_MIDL                0x1D  // Manufacturer ID low byte (0xA2)
#define OV2640_REG_PIDH                0x0A  // Product ID high byte (0x26)
#define OV2640_REG_PIDL                0x0B  // Product ID low byte (0x42)
#define OV2640_REG_AEW                 0x24  // AEW – AEC upper limit
#define OV2640_REG_AEB                 0x25  // AEB – AEC lower limit
#define OV2640_REG_EXHCH               0x2A  // Dummy pixel high bits
#define OV2640_REG_EXHCL               0x2B  // Dummy pixel low bits
#define OV2640_REG_DSP_CTRL1           0x50  // DSP control 1
#define OV2640_REG_DSP_CTRL2           0x51  // DSP control 2
#define OV2640_REG_DSP_CTRL3           0x52  // DSP control 3
#define OV2640_REG_DSP_CTRL4           0x53  // DSP control 4
#define OV2640_REG_DSP_AWBC1           0x5A  // AWB control
#define OV2640_REG_DSP_AWBC2           0x5B
#define OV2640_REG_DSP_AWBC3           0x5C
#define OV2640_REG_DSP_AWBC4           0x5D
#define OV2640_REG_DSP_AWBC5           0x5E
#define OV2640_REG_DSP_AWBC6           0x5F
#define OV2640_REG_RGB444              0x8C  // RGB444 control
#define OV2640_REG_COM15               0x40  // Format control (RGB565 etc.)
#define OV2640_REG_SCCB_ID             0x0A  // SCCB slave ID
#define OV2640_REG_RESET               0xE0  // Soft reset for DSP

// Bank select
#define OV2640_REG_BANK_SEL            0xFF  // Bank select register
#define OV2640_BANK_SENSOR             0x00
#define OV2640_BANK_DSP                0x01

