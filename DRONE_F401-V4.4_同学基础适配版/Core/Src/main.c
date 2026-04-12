/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mpu6050.h"
#include "flash.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "all_define.h"
#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t nrf2401_tx_flag=1;
uint8_t nrf2401_txbuf[33]={0xAA,0xAA,0x01,0x06,0x00,0x1D,0xFF,0xCA,0x09,0x1E,0x68};
uint8_t txbuf_pos=0;
extern uint16_t nrf_cnt;

uint8_t uart1_rxbuf[100];
uint8_t uart1_buf_pos=0;
uint8_t uart1_flag=0;
uint32_t uart1_tick_start=0;

__IO uint8_t uart2_rxbuf[100];
uint8_t uart2_buf_pos=0;
uint8_t uart2_flag=0;
uint32_t uart2_tick_start=0;
float temperature;
float presure; 
uint32_t baro_height; // barometer height in mm 
uint16_t voltage;    // battery voltage * 100
/* UART command buffer */
char uart_rx_buf[128];
uint8_t uart_rx_len = 0;
// FC soft-sleep timing state
uint32_t idle_tick = 0;          // last non-idle tick for FC soft sleep
static uint8_t fc_soft_sleeping = 0U;
#define SLEEP_TIME 60000         // legacy 60s timeout value

// IMU calibration Flash flag
#define CALIB_ADDR  0x0801F000
#define CALIB_OK    0xAA



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
extern void pid_param_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Enter FC soft sleep without shutting down MCU or NRF.
void Enter_Standby(void)
{
  /* FC soft sleep stops motors and LED indication only.
     MCU and NRF stay alive so the remote link and telemetry continue. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  ALL_flag.unlock = 0;
  LED.status = AlwaysOff;
  fc_soft_sleeping = 1U;
}
// Reset FC soft-sleep timer and wake from soft sleep.
void Reset_Idle(void)
{
  idle_tick = HAL_GetTick();
  fc_soft_sleeping = 0U;
}

extern uint8_t RC_rxData[32];
//uint8_t cnt_angle=0;
uint8_t SPL06_cnt=0,SPL06_flag=0;
uint8_t volt_cnt=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
  if(htim==&htim1)
  { 
   #if 1
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);

    MpuGetData();
    GetAngle(&MPU6050,&Angle,0.006f);
    OPTICAL2Read();

    FlightPidControl(0.006f);
    MotorControl();  
   #endif    
    nrf_cnt++;
    // Legacy note removed because its original encoding was corrupted.
    // Legacy note removed because its original encoding was corrupted.
    // Legacy note removed because its original encoding was corrupted.
    if(nrf_cnt>=100) 
    {
      ALL_flag.unlock = 0;
      nrf_cnt=0;
      //LED3_Toggle();
      //remote_unlock();
      LED.status = AllFlashLight;
      LED.FlashTime = 300;  
    }
    SPL06_cnt++;
    if(SPL06_cnt>10)
    {
      SPL06_cnt=0;
      temperature = user_SPL06_001_get_temperature();
      presure = user_SPL06_001_get_presure();
      baro_height = ((uint32_t)((102000.0f- presure) * 78.740f)+5)/10;
      SPL06_flag=1;
    }
    volt_cnt++;
    if(volt_cnt>=200)
    { HAL_ADC_Start_IT(&hadc1);
      volt_cnt =0;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	uint8_t sta;
  if(GPIO_Pin==NRF24L01_IRQ_Pin)
  { nrf_cnt = 0;
    sta=NRF24L01_Read_Reg(STATUS);
    if(sta&RX_OK)
    { 
      // Legacy note removed because its original encoding was corrupted.
      RC_Analy();
      NRF24L01_Write_Buf(0xa8, nrf2401_txbuf, txbuf_pos);
      nrf2401_tx_flag=1;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{ uint16_t adv;
  adv=HAL_ADC_GetValue(&hadc1);
  voltage=adv*430/4096;
  if (voltage<315)
  {
    ALL_flag.unlock = 0;
    LED.status =WARNING;
    LED.FlashTime = 500;     
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
//	MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(NRF24L01_IRQ_EXTI_IRQn); // Keep NRF IRQ disabled until NRF24L01 init is finished.
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET); // Disable NRF CE during setup.
	// ===================== IMU calibration =====================
uint8_t calib = FLASH_ReadByte(CALIB_ADDR);
if(calib != CALIB_OK)
{
    // No saved calibration: blink LEDs before first calibration.
    HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
    HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
    HAL_Delay(300);

    // Run sensor calibration before writing the calibration flag.
    MPU6050_Calibrate();

    // Mark calibration as valid in Flash.
    FLASH_WriteByte(CALIB_ADDR, CALIB_OK);
}
// Turn LEDs off after calibration.
HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);
HAL_Delay(1000);
  LED1_H();

  HAL_Delay(100);
  SPL06_001_init(); // Initialize SPL06 barometer.
  MpuInit(); // Initialize MPU6050.


// ==============================================
// Boot calibration is skipped when the Flash flag is already valid.
// Factory test helper: erase Flash to force recalibration.
// End of calibration block.
HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, LED4_Pin, GPIO_PIN_SET);
HAL_Delay(300);

// ====================== LED test block ======================
HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
HAL_Delay(3000);   // wait 3 seconds for visual check

HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
HAL_Delay(1000);

// ====================== LED test block ======================
//while(1)
//{
    // LED1 on, LED2 off
//    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
//    HAL_Delay(1000);
    
    // LED1 off, LED2 on
//    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1000);
//}
#if 1
 
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  __HAL_TIM_CLEAR_IT(&htim1, (TIM_IT_UPDATE|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4));
  //HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_DMA(&huart2,(uint8_t *)uart2_rxbuf,9);
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);  // Enable USART2 idle interrupt for fixed-length GL9306 packets.
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // Enable USART1 RX interrupt for command input.

  while(NRF24L01_Check())
  {
    LED1_Toggle();
    HAL_Delay(250);
  }
  pid_param_Init();
  LED1_Toggle();
  if((FLASH_ReadByte(PAIR_BUILD_ADDR) != (uint8_t)(PAIR_BUILD_ID & 0xFFU)) ||
     (FLASH_ReadByte(PAIR_BUILD_ADDR + 1U) != (uint8_t)((PAIR_BUILD_ID >> 8) & 0xFFU)))
  {
    FLASH_EraseSectorByAddr(PAIR_ADDR);
    FLASH_WriteByte(PAIR_ADDR, 0);
    FLASH_WriteByte(PAIR_ADDR + 1U, 0x11);
    FLASH_WriteByte(PAIR_ADDR + 2U, 0x22);
    FLASH_WriteByte(PAIR_ADDR + 3U, 0x33);
    FLASH_WriteByte(PAIR_ADDR + 4U, 0x44);
    FLASH_WriteByte(PAIR_ADDR + 5U, 0x55);
    FLASH_WriteByte(PAIR_FLAG, PAIR_MAGIC);
    FLASH_WriteByte(PAIR_BUILD_ADDR, (uint8_t)(PAIR_BUILD_ID & 0xFFU));
    FLASH_WriteByte(PAIR_BUILD_ADDR + 1U, (uint8_t)((PAIR_BUILD_ID >> 8) & 0xFFU));
    FLASH_WriteByte(FC_SLEEP_ADDR, (uint8_t)(g_fc_idle_sleep_seconds & 0xFFU));
    FLASH_WriteByte(FC_SLEEP_ADDR + 1U, (uint8_t)((g_fc_idle_sleep_seconds >> 8) & 0xFFU));
    FLASH_WriteByte(FC_SLEEP_MAGIC_ADDR, FC_SLEEP_MAGIC);
  }
  if(FLASH_ReadByte(PAIR_FLAG) == PAIR_MAGIC)
  {
    uint8_t saved_rf_addr[5];
    uint8_t saved_rf_channel = FLASH_ReadByte(PAIR_ADDR);
    saved_rf_addr[0] = FLASH_ReadByte(PAIR_ADDR + 1U);
    saved_rf_addr[1] = FLASH_ReadByte(PAIR_ADDR + 2U);
    saved_rf_addr[2] = FLASH_ReadByte(PAIR_ADDR + 3U);
    saved_rf_addr[3] = FLASH_ReadByte(PAIR_ADDR + 4U);
    saved_rf_addr[4] = FLASH_ReadByte(PAIR_ADDR + 5U);
    if(saved_rf_channel <= 125U)
    {
      NRF24L01_SetChannelAddress(saved_rf_channel, saved_rf_addr);
    }
  }
  RX_Mode();
  HAL_ADC_Start_IT(&hadc1); // Start battery-voltage ADC conversion.

  NRF24L01_Write_Buf(0xa8, nrf2401_txbuf, 11); // Initialize ACK payload buffer.
  HAL_NVIC_EnableIRQ(NRF24L01_IRQ_EXTI_IRQn); // Enable NRF IRQ after NRF24L01 init is complete.
  FC_LoadSleepConfig();

  Reset_Idle();  // start FC soft-sleep timer after RF init
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  { 
    // Enter FC soft sleep after the configured locked-idle timeout.
    if((g_fc_idle_sleep_seconds != 0U) &&
       (fc_soft_sleeping == 0U) &&
       ((HAL_GetTick() - idle_tick) >= ((uint32_t)g_fc_idle_sleep_seconds * 1000U)))
    {
        ALL_flag.unlock = 0;
        // MotorStop(); // PWM outputs are cleared inside Enter_Standby().
        Enter_Standby();
    }

    ANO_ServicePairSave();
    ANTO_polling();
    PilotLED();
    ANTO_polling();
    PilotLED();
    //OPTICAL2Read();
    if(uart1_flag)
    { uart1_flag=0;
      //send_char_array(&huart1,uart1_rxbuf,uart1_buf_pos);
      uart1_buf_pos=0;
    }
    if(uart2_flag)
    { uart2_flag=0;
      flow_decode();
      //send_char_array(&huart1,(uint8_t *)uart2_rxbuf,9);//uart2_buf_pos);
      uart2_buf_pos=0;
    }

    if(SPL06_flag)
    { SPL06_flag=0;
      // Legacy note removed because its original encoding was corrupted.
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,65535);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,65535);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,65535);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,65535);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_CSN_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF24L01_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24L01_CE_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CE_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24L01_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24L01_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#include <string.h>
#include <stdio.h>


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    ITM_SendChar(ch);
    return ch;
}

// USART1 command parser, currently used by AT+PAIR.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        // A line ending marks one complete UART command.
        if(uart_rx_buf[uart_rx_len] == '\n' || uart_rx_buf[uart_rx_len] == '\r')
        {
            uart_rx_buf[uart_rx_len] = '\0'; // terminate command string
            
            // Handle pair/reset command.
            if(strstr(uart_rx_buf, "AT+PAIR") != NULL || strstr(uart_rx_buf, "pair") != NULL)
            {
                // 1. Write pair-valid marker.
                FLASH_WriteByte(PAIR_FLAG, PAIR_MAGIC);
                
                // 2. Restore default channel/address and preserve FC sleep settings.
                uint8_t default_addr[] = {0,0x11,0x22,0x33,0x44,0x55};
                FLASH_EraseSectorByAddr(PAIR_ADDR);
                FLASH_WriteByte(PAIR_ADDR, default_addr[0]);
                FLASH_WriteByte(PAIR_ADDR + 1U, default_addr[1]);
                FLASH_WriteByte(PAIR_ADDR + 2U, default_addr[2]);
                FLASH_WriteByte(PAIR_ADDR + 3U, default_addr[3]);
                FLASH_WriteByte(PAIR_ADDR + 4U, default_addr[4]);
                FLASH_WriteByte(PAIR_ADDR + 5U, default_addr[5]);
                FLASH_WriteByte(PAIR_FLAG, PAIR_MAGIC);
                FLASH_WriteByte(FC_SLEEP_ADDR, (uint8_t)(g_fc_idle_sleep_seconds & 0xFFU));
                FLASH_WriteByte(FC_SLEEP_ADDR + 1U, (uint8_t)((g_fc_idle_sleep_seconds >> 8) & 0xFFU));
                FLASH_WriteByte(FC_SLEEP_MAGIC_ADDR, FC_SLEEP_MAGIC);
                
                // 3. Reply to the UART console.
                char ok_msg[] = "PAIR OK! Reboot...\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)ok_msg, strlen(ok_msg), 100);
                
                // 4. Reboot so RF settings are reloaded cleanly.
                HAL_NVIC_SystemReset();
            }
            else
            {
                // Unknown command.
                char err_msg[] = "Unknown Command! Use: AT+PAIR\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)err_msg, strlen(err_msg), 100);
            }
            
            // Clear receive buffer for the next command.
            uart_rx_len = 0;
        }
        else
        {
            // Continue collecting command bytes.
            uart_rx_len++;
            if(uart_rx_len >= 127) uart_rx_len = 0; // prevent buffer overflow
        }
        
        // Re-arm UART interrupt reception.
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buf[uart_rx_len], 1);
    }
}
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//    {
//        // Capture ultrasonic echo width.
//        hc_sr04_time = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//        
//        // Distance formula: d = time_us * 0.034 / 2
//        hc_sr04_dist = hc_sr04_time * 0.017f;  
//        hc_sr04_ok = 1;
//        
//        // Switch capture edge for the next measurement.
//        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
//    }
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
