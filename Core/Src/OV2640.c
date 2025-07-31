/*
 * OV2640.c
 * OV2640 DCMI DMA Driver
 * Created on: Aug 1, 2025
 *
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include "OV2640.h"

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
#define OV2640_BUFFER_SIZE            (OV2640_WIDTH_SIZE_BYTES * OV2640_RGB565_BYTES * OV2640_LINES_IN_CHUNK) // 2 LINES!!! QVGA !!! 1280 - size for global buffer


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



/* Draw line callback type */
typedef void (*drawLine_cb_t)(const uint8_t *buffer, uint32_t buf_size, uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/* Draw frame callback type */
typedef void (*drawFrame_cb_t)(const uint8_t *buffer, uint32_t buf_size);

static struct
{
    /* HAL peripheral handlers */
    DCMI_HandleTypeDef  *hdcmi;
    I2C_HandleTypeDef   *hi2c;
    TIM_HandleTypeDef   *htim;
    uint32_t            tim_ch;
    /* Requested mode */
    uint32_t            mode;
    /* Address of the buffer */
    volatile uint32_t   buffer_addr;
    /* Image line counter */
    volatile uint32_t   lineCnt;
    /* Driver status */
    volatile uint8_t    state;
    /* Draw line callback prototype */
    drawLine_cb_t       drawLine_cb;
    /* Draw frame callback prototype */
    drawFrame_cb_t      drawFrame_cb;
} OV2640;

static uint8_t buffer_2_lines[OV2640_BUFFER_SIZE];

void OV2640_Init(DCMI_HandleTypeDef *hdcmi, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim, uint32_t tim_ch)
{
    OV2640.hdcmi = hdcmi;
    OV2640.hi2c = hi2c;
    OV2640.htim = htim;
    OV2640.tim_ch = tim_ch;

    /* PWDN to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_PWDN, OV7670_GPIO_PIN_PWDN, GPIO_PIN_RESET);
    /* RET pin to LOW */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_RESET);
    OV7670_DELAY(100);
    /* RET pin to HIGH */
    HAL_GPIO_WritePin(OV7670_GPIO_PORT_RET, OV7670_GPIO_PIN_RET, GPIO_PIN_SET);
    OV7670_DELAY(100);

    /* Start camera XLK signal to be able to do initialization */
    OV2640_START_XLK(OV2640.htim, OV2640.tim_ch);

    /* Do camera reset */
    SCCB_Write(OV7670_REG_COM7, 0x80);
    OV7670_DELAY(30);

    /* Get camera ID */
    uint8_t buf[4];
    SCCB_Read(OV7670_REG_VER, buf);
    //DEBUG_LOG("[OV7670] dev id = 0x%02X", buf[0]);

    /* Do camera reset */
    SCCB_Write(OV7670_REG_COM7, 0x80);
    HAL_Delay(30);

    /* Do camera configuration */
    for (uint32_t i = 0; OV7670_reg[i][0] != OV7670_REG_DUMMY; i++)
    {
        SCCB_Write(OV7670_reg[i][0], OV7670_reg[i][1]);
        HAL_Delay(1);
    }
    /* Stop camera XLK signal */
    OV2640_STOP_XLK(OV2640.htim, OV2640.tim_ch);

    /* Initialize buffer_2_lines address */
    OV7670.buffer_addr = (uint32_t) buffer_2_lines;

}



void OV7670_RegisterCallback(OV7670_CB_t cb_type, OV7670_FncPtr_t fnc_ptr)
{
    switch (cb_type)
    {
        case OV7670_DRAWLINE_CBK:
        {
            OV7670.drawLine_cb = (drawLine_cb_t)fnc_ptr;
            break;
        }

       

        default:
            break;
    }
}

void OV2640_Start(void)
{
    __disable_irq();
    /* Update requested mode */
    OV2640.mode = DCMI_MODE_CONTINUOUS;
#if (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE)
    /* Reset buffer address */
    OV2640.buffer_addr = OV7670_RESET_BUFFER_ADDR();
#endif
    /* Reset line counter */
    OV2640.lineCnt = 0U;
    OV2640.state = BUSY;
    __enable_irq();
    /* Start camera XLK signal to capture the image data */
    OV7670_START_XLK(OV2640.htim, OV2640.tim_ch);
    /* Start DCMI capturing */

    HAL_DCMI_Start_DMA(OV2640.hdcmi, DCMI_MODE_CONTINUOUS, OV2640.buffer_addr, OV7670_DMA_DATA_LEN);
}

static uint8_t isFrameCaptured(void)
{
    uint8_t retVal;
    __disable_irq();
    retVal = (OV2640.lineCnt == 0U) ? TRUE : FALSE;
    __enable_irq();
    return retVal;
}


void OV7670_Stop(void)
{
    while(!isFrameCaptured());
    __disable_irq();
#if (OV7670_USE_DMA_CMSIS == 1)
    OV7670_DCMI_DMA_STOP(OV7670.hdcmi);
#else
    HAL_DCMI_Stop(OV2640.hdcmi);
#endif
    OV2640.state = READY;
    __enable_irq();
    OV7670_STOP_XLK(OV2640.htim, OV2640.tim_ch);
}



void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    uint8_t vsync_detected = FALSE;
    uint8_t state = BUSY;
    uint32_t buf_addr = 0x0U;
    uint32_t lineCnt;

    if (!(hdcmi->Instance->SR & DCMI_SR_VSYNC))
    {
        vsync_detected = TRUE;
    }

    if (vsync_detected)
    {
        __disable_irq();
        lineCnt = OV7670.lineCnt;
        __enable_irq();

        /* If this line is the last line of the frame */
        if (lineCnt == OV7670_HEIGHT - 1U)
        {
            /* Disable DCMI Camera interface */
#if (OV7670_USE_DMA_CMSIS == 1)
            OV7670_DCMI_DMA_STOP(OV7670.hdcmi);
#else
            HAL_DCMI_Stop(OV7670.hdcmi);
#endif
            /* Stop camera XLK signal until captured image data is drawn */
            //HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);
            /* Reset line counter */
            lineCnt = 0U;

            /* Update state if mode is SNAPSHOT */
            if (OV7670.mode == DCMI_MODE_SNAPSHOT)
            {
                state = READY;
                vsync_detected = FALSE;
            }
        }
        else
        {
            /* Increment line counter */
            lineCnt++;
        }

        if (((lineCnt + 1U) % OV7670_LINES_IN_CHUNK) == 0U)
        {
            /* Call Display flush function */
            if (OV7670.drawLine_cb != NULL)
            {
                OV7670.drawLine_cb((uint8_t*) OV7670.buffer_addr, (OV7670_WIDTH_SIZE_BYTES * OV7670_LINES_IN_CHUNK) ,
                        0U, (OV7670_WIDTH - 1U), (lineCnt + 1U - OV7670_LINES_IN_CHUNK), lineCnt);
            }

            /* If driver is still working */
            if (state == BUSY)
            {
                /* Update buffer address with the next half-part */
                buf_addr = OV7670_SWITCH_BUFFER();
                /* Capture next line from the snapshot/stream */
#if (OV7670_USE_DMA_CMSIS == 1)
                OV7670_DCMI_DMA_START(OV7670.hdcmi, buf_addr);
#else
                HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_CONTINUOUS, buf_addr, OV7670_DMA_DATA_LEN);
#endif
            }
        }

        /* Update line counter */
        __disable_irq();
        OV7670.lineCnt = lineCnt;
        OV7670.state = state;
        OV7670.buffer_addr = (buf_addr) ? buf_addr : OV7670.buffer_addr;
        __enable_irq();
    }
}






/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data)
{
    HAL_StatusTypeDef ret;
    do
    {
        ret = HAL_I2C_Mem_Write(OV7670.hi2c, OV7670_SCCB_ADDR, regAddr,
                I2C_MEMADD_SIZE_8BIT, &data, 1U, 100U);
    }
    while (ret != HAL_OK);

    return ret;
}

static HAL_StatusTypeDef SCCB_Read(uint8_t regAddr, uint8_t *data)
{
    HAL_StatusTypeDef ret;

    do
    {
        /* HAL_I2C_Mem_Read doesn't work because of SCCB protocol(doesn't have ACK) */
        ret = HAL_I2C_Master_Transmit(OV7670.hi2c, OV7670_SCCB_ADDR, &regAddr, 1U, 100U);
        ret |= HAL_I2C_Master_Receive(OV7670.hi2c, OV7670_SCCB_ADDR, data, 1U, 100U);
    }
    while (ret != HAL_OK);

    return ret;
}