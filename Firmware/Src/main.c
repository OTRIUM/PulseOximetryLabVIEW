/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"


/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "algorithm.h"
    
#define I2C_ADDRESS 0x57

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t autoMode = 0;
uint8_t rxData = 0, txData = 0;
uint8_t USB_Data[64]={0};

uint32_t aun_ir_buffer[500];                                                    // IR LED sensor data
int32_t n_ir_buffer_length;                                                     // data length
uint32_t aun_red_buffer[500];                                                   // Red LED sensor data
int32_t n_sp02;                                                                 // SPO2 value
int8_t ch_spo2_valid;                                                           // indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;                                                           // heart rate value
int8_t  ch_hr_valid;                                                            // indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint32_t un_min, un_max, un_prev_data;                                          // variables to calculate the on-board LED brightness that reflects the heartbeats
uint32_t i, counter;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t receiveByteI2C (uint8_t i2cRxReg)
{
  HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) I2C_ADDRESS<<1, &i2cRxReg, 1, 100);
  HAL_I2C_Master_Receive(&hi2c2, (uint16_t) I2C_ADDRESS<<1, (uint8_t*)&rxData, 1, 100);
  return rxData;
}

void transmitByteI2C (uint8_t i2cTxReg, uint8_t i2cTxData)
{
  uint8_t i2c_Buffer[2];
  i2c_Buffer[0] = i2cTxReg;
  i2c_Buffer[1] = i2cTxData;
  HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) I2C_ADDRESS<<1, (uint8_t*)i2c_Buffer, 2, 100);
}

void MAX30102_Init (void)
{
  transmitByteI2C(0x02, 0xC0);                                                  // INTR setting
  transmitByteI2C(0x03, 0x00);                                                  
  transmitByteI2C(0x04, 0x00);                                                  // FIFO_WR_PTR[4:0]
  transmitByteI2C(0x05, 0x00);                                                  // OVF_COUNTER[4:0]
  transmitByteI2C(0x06, 0x00);                                                  // FIFO_RD_PTR[4:0]
  transmitByteI2C(0x08, 0x0f);                                                  // sample avg = 1, fifo rollover=false, fifo almost full = 17
  transmitByteI2C(0x09, 0x03);                                                  // 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
  transmitByteI2C(0x0A, 0x27);                                                  // SPO2_ADC range = 4096mA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
  
  transmitByteI2C(0x0C, 0x24);                                                  // Choose value for ~ 7mA for LED1
  transmitByteI2C(0x0D, 0x24);                                                  // Choose value for ~ 7mA for LED2
  transmitByteI2C(0x10, 0x7F);                                                  // Choose value for ~ 25mA for Pilot LED
}

void MAX30102_Reset (void)
{
  transmitByteI2C(0x09, 0x40);                                                  // Reset chip
}

void MAX30102_Read_FIFO (uint32_t *pun_red_led, uint32_t *pun_ir_led)           // FIFO Read
{
  uint32_t un_temp;
  uint8_t uch_temp;
  
  *pun_red_led = 0;
  *pun_ir_led = 0;
  
  uint8_t ach_i2c_data[6];
    
  //read and clear status register
  uch_temp = receiveByteI2C(0x00);
  uch_temp = receiveByteI2C(0x01);
  
  ach_i2c_data[0] = 0x07;                                                       // == REG_FIFO_DATA 
  HAL_I2C_Master_Transmit(&hi2c2, (uint16_t) I2C_ADDRESS<<1, ach_i2c_data, 1, 100);
  HAL_I2C_Master_Receive(&hi2c2, (uint16_t) I2C_ADDRESS<<1, (uint8_t*) &ach_i2c_data, 6, 100);
  
  un_temp = (uint8_t) ach_i2c_data[0];
  un_temp <<= 16;
  *pun_red_led += un_temp;
  un_temp =(uint8_t) ach_i2c_data[1];
  un_temp <<= 8;
  *pun_red_led += un_temp;
  un_temp =(uint8_t) ach_i2c_data[2];
  *pun_red_led += un_temp;
  
  un_temp = (unsigned char) ach_i2c_data[3];
  un_temp <<= 16;
  *pun_ir_led += un_temp;
  un_temp = (unsigned char) ach_i2c_data[4];
  un_temp <<= 8;
  *pun_ir_led += un_temp;
  un_temp =(unsigned char) ach_i2c_data[5];
  *pun_ir_led += un_temp;
  
  *pun_red_led &= 0x03FFFF;                                                     // Mask MSB [23:18]
  *pun_ir_led  &= 0x03FFFF;                                                     // Mask MSB [23:18]
   
}

void USB_Request_Handler(char RX_Data[64], uint32_t Len)                        
{  
  uint8_t TX_Data[64]={0};
  uint8_t sentAddress = 0, sentData = 0;
  
  if(strncmp(RX_Data, "SIEG", 4) == 0)                                          // connection test
  {
    CDC_Transmit_FS("HEIL", 4);    
  }
  
  if(strncmp(RX_Data, "echo ", 5) == 0)                                         // echo
  {
    CDC_Transmit_FS(RX_Data + 5, Len - 5);    
  }
  
  if(strncmp(RX_Data, "PartID", 6) == 0)                                        // PartID
  {
    rxData = receiveByteI2C(0xFF);
    
    sprintf (TX_Data, "%d", rxData);
    CDC_Transmit_FS(TX_Data, strlen(TX_Data));
  }
  
  if(strncmp(RX_Data, "RevID", 5) == 0)                                         // RevID
  {
    rxData = receiveByteI2C(0xFE);
    
    sprintf (TX_Data, "%d", rxData);
    CDC_Transmit_FS(TX_Data, strlen(TX_Data));
  }
   
  if(strncmp(RX_Data, "AUTO", 4) == 0)                                          // AUTO
  {   
    autoMode = 1;
  }
  
  if(strncmp(RX_Data, "STOP", 4) == 0)                                          // STOP
  {    
    autoMode = 0;
  }
  
  if(strncmp(RX_Data, "I2CSEND ", 8) == 0)                                      // I2CSEND
  {
    sscanf(RX_Data + 8, "%d, %d", &sentAddress, &sentData);
    transmitByteI2C(sentAddress, sentData);
  }
  
}

/* USER CODE END 0 */

  int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  
  /* USER CODE BEGIN 2 */
 
  MAX30102_Reset();
    
  uch_dummy = receiveByteI2C(0x00);
  
  MAX30102_Init();
  
  un_min = 0x3FFFF;
  un_max = 0;
  un_prev_data = 0;
  
  n_ir_buffer_length = 500;                                                     // buffer length of 100 stores 1 second of samples running at 100sps
  
  i = 0;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {

    if (autoMode)
    {
      if (i < n_ir_buffer_length)
      {
        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5));                             // wait until the interrupt pin goes fucken low
      
        MAX30102_Read_FIFO ((aun_red_buffer+i), (aun_ir_buffer+i));             // read from MAX30102 FIFO
      
        if(un_min>aun_red_buffer[i])
          un_min=aun_red_buffer[i];                                             // update signal min
        if(un_max<aun_red_buffer[i])
          un_max=aun_red_buffer[i];                                             // update signal max
        
        sprintf(USB_Data, "R%06d", aun_red_buffer[i]);
        while (CDC_Transmit_FS(USB_Data, strlen(USB_Data)) != USBD_OK );
        sprintf(USB_Data, "I%06d", aun_ir_buffer[i]);
        while (CDC_Transmit_FS(USB_Data, strlen(USB_Data)) != USBD_OK);   
        
        i++;
      }
      else 
      {
        //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
        
        sprintf(USB_Data, "P%06d", n_heart_rate);
        while (CDC_Transmit_FS(USB_Data, strlen(USB_Data)) != USBD_OK);   
        
        HAL_Delay(10);
        
        sprintf(USB_Data, "O%06d", n_sp02);
        while (CDC_Transmit_FS(USB_Data, strlen(USB_Data)) != USBD_OK);   
        
        un_min = 0x3FFFF;
        un_max = 0;
        un_prev_data = 0;
        
        for(i=0; i<n_ir_buffer_length; i++)
        {
          aun_red_buffer[i] = 0;
          aun_ir_buffer[i] = 0;    
        }
        
        i = 0;
        
      }
      
    }
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
