/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/*----------------------------------------------------------------------------
 * Tencent is pleased to support the open source community by making TencentOS
 * available.
 *
 * Copyright (C) 2019 THL A29 Limited, a Tencent company. All rights reserved.
 * If you have downloaded a copy of the TencentOS binary from Tencent, please
 * note that the TencentOS binary is licensed under the BSD 3-Clause License.
 *
 * If you have downloaded a copy of the TencentOS source code from Tencent,
 * please note that TencentOS source code is licensed under the BSD 3-Clause
 * License, except for the third-party components listed below which are
 * subject to different license terms. Your integration of TencentOS into your
 * own projects may require compliance with the BSD 3-Clause License, as well
 * as the other licenses applicable to the third-party components included
 * within TencentOS.
 *---------------------------------------------------------------------------*/
/**
 * @file    MIMXRT1011xxxxx_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1011.h"
/* TODO: insert other include files here. */
#include "cmsis_os.h"

#if defined(SDK_I2C_BASED_COMPONENT_USED)
#include "fsl_lpi2c.h"

typedef status_t (*I2C_SendFunc_t)(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff);
typedef status_t (*I2C_ReceiveFunc_t)(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);

/*! @brief fxos8700cq configure definition. This structure should be global.*/
typedef struct _fxos_handle
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
    /* The I2C slave address . */
    uint8_t slaveAddress;
} fxos_handle_t;

/*! @brief fxos8700cq configure structure.*/
typedef struct _fxos_config
{
    /* Pointer to the user-defined I2C Send Data function. */
    I2C_SendFunc_t I2C_SendFunc;
    /* Pointer to the user-defined I2C Receive Data function. */
    I2C_ReceiveFunc_t I2C_ReceiveFunc;
    /* The I2C slave address . */
    uint8_t slaveAddress;
} fxos_config_t;

#define EXAMPLE_I2C_MASTER_BASE (LPI2C1_BASE)

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY


#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 33U
// globlals
uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
lpi2c_master_handle_t g_m_handle;
volatile bool xfer_complete = false;
volatile bool nak_flag = false;
lpi2c_master_config_t masterConfig;
uint8_t g_sensor_address[] = { 0x39, 0x29, 0x49 };
//lpi2c_master_transfer_t masterXfer = {0};
fxos_handle_t g_fxosHandle;

fxos_config_t config = {0};
uint8_t array_addr_size = 0;

/** TSL2561 I2C Registers */
enum
{
  TSL2561_REGISTER_CONTROL          = 0x00, // Control/power register
  TSL2561_REGISTER_TIMING           = 0x01, // Set integration time register
  TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02, // Interrupt low threshold low-byte
  TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03, // Interrupt low threshold high-byte
  TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04, // Interrupt high threshold low-byte
  TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05, // Interrupt high threshold high-byte
  TSL2561_REGISTER_INTERRUPT        = 0x06, // Interrupt settings
  TSL2561_REGISTER_CRC              = 0x08, // Factory use only
  TSL2561_REGISTER_ID               = 0x0A, // TSL2561 identification setting
  TSL2561_REGISTER_CHAN0_LOW        = 0x0C, // Light data channel 0, low byte
  TSL2561_REGISTER_CHAN0_HIGH       = 0x0D, // Light data channel 0, high byte
  TSL2561_REGISTER_CHAN1_LOW        = 0x0E, // Light data channel 1, low byte
  TSL2561_REGISTER_CHAN1_HIGH       = 0x0F  // Light data channel 1, high byte
};

status_t FXOS_Init(fxos_handle_t *fxos_handle, fxos_config_t *cfg);

#endif // SDK_I2C_BASED_COMPONENT_USE

/* TODO: insert other definitions and declarations here. */

#define WAIT_INT()	while (1) {	 	 			\
						asm("wfi;"); 			\
					}

#define TOS_ERROR_HANDLER(ERR_CODE)				\
		printf("Error %d\r\n",ERR_CODE);        \
		WAIT_INT();

#define TOS_ERROR_CHECK(ERR_CODE)				\
		do {									\
			if ( ERR_CODE != osOK ) {			\
				TOS_ERROR_HANDLER(ERR_CODE);	\
			}									\
		} while(0)								\

#define TOS_ERROR_PRINT(ERRMSG)					\
        do {									\
        	printf("%s\r\n",ERRMSG);			\
        } while(0);								\
        WAIT_INT();


#define TASK1_STK_SIZE          768
void task1(void *arg);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);

#define TASK2_STK_SIZE          768
void task2(void *arg);
osThreadDef(task2, osPriorityNormal, 1, TASK2_STK_SIZE);

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED

#define LPI2C_MASTER_SLAVE_ADDR_7BIT 0x39
#define LPI2C_BAUDRATE 100000U

/* Send master blocking data to slave */
status_t wrI2Cdata (void)
{
status_t reval;
size_t txCount        = 0x1;

		reval = LPI2C_MasterStart(EXAMPLE_I2C_MASTER, g_sensor_address[0], kLPI2C_Write);
		if (kStatus_Success == reval)
		{

	        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        while (txCount)
	        {
	            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        }
	        /* Check communicate with slave successful or not */
	        while (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
	        {
	        }

	        reval = LPI2C_MasterSend(EXAMPLE_I2C_MASTER,g_master_txBuff , 1);

	        if (reval != kStatus_Success)
	        {
	             return reval;
	        }

	        reval = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);

		}

		return reval;
}

/* read blocking data from slave */
status_t rdI2Cdata (void)
{
status_t reval;
size_t txCount        = 0x1;

		reval = LPI2C_MasterStart(EXAMPLE_I2C_MASTER, g_sensor_address[0], kLPI2C_Read);
		if (kStatus_Success == reval)
		{

	        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        while (txCount)
	        {
	            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        }
	        /* Check communicate with slave successful or not */
	        while (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
	        {
	        }

	        reval = LPI2C_MasterReceive(EXAMPLE_I2C_MASTER, g_master_rxBuff, 1);
	        if (reval != kStatus_Success)
	        {
	        	return reval;
	        }

	        reval = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);

		}

		return reval;
}


status_t FXOS_ReadReg(fxos_handle_t *handle, uint8_t reg, uint8_t *val, uint8_t bytesNumber)
{
    assert(handle);
    assert(val);

    if (!handle->I2C_ReceiveFunc)
    {
        return kStatus_Fail;
    }

    return handle->I2C_ReceiveFunc(handle->slaveAddress, reg, 1, val, bytesNumber);
}

status_t FXOS_Init(fxos_handle_t *fxos_handle, fxos_config_t *cfg)
{
    assert(fxos_handle);
    assert(cfg);
    assert(cfg->I2C_SendFunc);
    assert(cfg->I2C_ReceiveFunc);

    uint8_t tmp[1] = {0};

    /* Initialize the I2C access function. */
    fxos_handle->I2C_SendFunc    = cfg->I2C_SendFunc;
    fxos_handle->I2C_ReceiveFunc = cfg->I2C_ReceiveFunc;
    /* Set Slave Address. */
    fxos_handle->slaveAddress = cfg->slaveAddress;

    if (FXOS_ReadReg(fxos_handle, TSL2561_REGISTER_ID, tmp, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}


#endif  // SDK_I2C_BASED_COMPONENT_USED

void task1(void *arg)
{
	int count = 0;
	while(1)
	{
		printf("********This is Task 1, count is %d \r\n",count++);
		osDelay(1000);
		USER_LED_TOGGLE();
	}
}


void task2(void *arg)
{
	int count = 0;
	while(1)
	{

		printf("++++++++This is Task 2, count is %d \r\n",count++);
		osDelay(2000);
	}
}

/*
 * @brief   Application entry point.
 */

int main(void) {

status_t result;
osStatus st;
osThreadId otid;

	BOARD_ConfigMPU();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	printf("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
	printf("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
	printf("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
	printf("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
	printf("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
	printf("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));

	st = osKernelInitialize();
	TOS_ERROR_CHECK(st);
    otid = osThreadCreate(osThread(task1), NULL); // Create task1
	if ( otid == NULL ) {
		TOS_ERROR_PRINT("Task1 create error");
	}
    otid = osThreadCreate(osThread(task2), NULL); // Create task2
	if ( otid == NULL ) {
		TOS_ERROR_PRINT("Task2 create error");
	}


//	st = osKernelStart(); // Start TencentOS Tiny
//	TOS_ERROR_CHECK(st);

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED

	/*Clock setting for LPI2C*/
	CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
	CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;

//    LPI2C_MasterDeinit(EXAMPLE_I2C_MASTER);
//    LPI2C_MasterReset(EXAMPLE_I2C_MASTER);

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);

    /* Initialize sensor devices */

    g_master_txBuff[0] =  TSL2561_REGISTER_ID;

    result = wrI2Cdata();
    if (result != kStatus_Success)
    	printf("\r\nI2C device initialize failed! %d\r\n",result);
    else
    	printf("WR data ok\r\n");
    result = rdI2Cdata();
    if (result != kStatus_Success)
         printf("\r\nI2C device initialize failed! %d\r\n",result);
    else
    	printf("\r\nI2C register %d\r\n",g_master_rxBuff[0]);
#endif

    st = osKernelStart(); // Start TencentOS Tiny
    TOS_ERROR_CHECK(st);
	return 0;

}


