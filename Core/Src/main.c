/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t u1_Age;
  uint16_t u2_Year;
  uint32_t u4_Count;
} t_Infor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile t_Infor xnl_Infor = { 3U, 8U, 6U };
static volatile uint8_t u1l_buffer_write[1024u];
static volatile uint8_t u1l_buffer_read[1024u];
/* Flash buffer */
static uint8_t u1l_arr_read_buffer[10U];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void f_vol_flash_ena_write_volatile_req(void);
static void f_vol_flash_send_command(void);
static void f_vol_flash_read_id(uint8_t *p2u1a_Status);
static void f_vol_flash_read_status_register(uint8_t *p2u1a_Status);
static void f_vol_flash_read_REpower_down( void );
static void f_vol_flash_read_power_down( void );
static void f_vol_flash_read_enaWrite( void );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
#ifdef USING_W25_LIB
  for( uint16_t u2_Cnt = 0U; u2_Cnt < 1024U; u2_Cnt += 1U )
  {
    u1l_buffer_write[u2_Cnt] = u2_Cnt;
  }
  W25qxx_Init();
  W25qxx_EraseSector(1u);
  W25qxx_WriteSector((uint8_t *)u1l_buffer_write, 1u, 0u, 1024u);
  W25qxx_ReadSector((uint8_t *)u1l_buffer_read, 1u, 0u, 1024u);
  for( uint16_t u2_Cnt = 0U; u2_Cnt < 1024U; u2_Cnt += 1U )
  {
    u1l_buffer_read[u2_Cnt] = 0U;
  }
  for( uint16_t u2_Cnt = 0U; u2_Cnt < 1024U; u2_Cnt += 1U )
  {
    u1l_buffer_write[u2_Cnt] = 0xFFU;
  }
  W25qxx_WriteSector((uint8_t *)u1l_buffer_write, 1u, 0u, 1024u);
  W25qxx_ReadSector((uint8_t *)u1l_buffer_read, 1u, 0u, 1024u);
  W25qxx_EraseSector(1u);
  W25qxx_WriteSector((uint8_t *)u1l_buffer_write, 1u, 0u, 1024u);
  for( uint16_t u2_Cnt = 0U; u2_Cnt < 1024U; u2_Cnt += 1U )
  {
    u1l_buffer_read[u2_Cnt] = 0U;
  }
  // f_vol_flash_send_command();

#elif defined TEST_ERASE_DUPLICATE

	uint8_t u1a_ReadData[3] ;
  uint8_t u1a_Data[3] = { 0x33, 0x44, 0x55 };
  uint8_t u1a_Data2[3] = { 0x34, 0x45, 0x56 };
	
  W25qxx_Init();
  W25qxx_EraseSector(1);
  W25qxx_EraseSector(2);
	
	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 1u, 0u, 3u);
	u1a_ReadData[0] = 0xaa;
	u1a_ReadData[1] = 0xaa;
	u1a_ReadData[2] = 0xaa;
	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 2u, 0u, 3u);
	
	W25qxx_WriteSector((uint8_t *)u1a_Data, 1u, 0u, 3u);
	W25qxx_WriteSector((uint8_t *)u1a_Data2, 2u, 0u, 3u);

	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 1u, 0u, 3u);
	u1a_ReadData[0] = 0xaa;
	u1a_ReadData[1] = 0xaa;
	u1a_ReadData[2] = 0xaa;
	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 2u, 0u, 3u);

  W25qxx_EraseSector(1);
  W25qxx_EraseSector(1);
  W25qxx_EraseSector(2);
	
	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 1u, 0u, 3u);
	u1a_ReadData[0] = 0xaa;
	u1a_ReadData[1] = 0xaa;
	u1a_ReadData[2] = 0xaa;
	W25qxx_ReadSector((uint8_t *)u1a_ReadData, 2u, 0u, 3u);


#endif /* USING_W25_LIB */
W25qxx_Init();
  volatile uint8_t u1a_Status_id, u1a_Status_ic;
  volatile t_Infor *p2xna_Infor;
  p2xna_Infor = ( t_Infor* )&xnl_Infor;
  u1a_Status_id = p2xna_Infor->u1_Age;

//  f_vol_flash_read_enaWrite();
//  f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//  f_vol_flash_read_power_down();
//  f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
    f_vol_flash_read_id((uint8_t *)&u1a_Status_id);
//    if ( u1a_Status_id == 0u )
//    {
//      while(1){}
//    }

//  f_vol_flash_read_REpower_down();
//  f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//    f_vol_flash_read_id((uint8_t *)&u1a_Status_id);
//    if ( u1a_Status_id == 1u )
//    {
//      while(1){}
//    }
//  f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);

//  W25qxx_EraseSector(1u);
////  W25qxx_ReadSector((uint8_t *)u1l_buffer_read, 1u, 0u, 1024u);

//  f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//  if ( 0U == ( u1a_Status_ic & 1U ) )
//  {
//    /* Flash Idle */
//    while( 1U ){}
//  }
//  else
//  {
//    /* Flash busy */
//    u1a_Status_ic = u1a_Status_id = 10u;
//    f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//    f_vol_flash_read_id((uint8_t *)&u1a_Status_id);
//    f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//    for( uint32_t i = 0U; i < 100000U; i++ ){}
//    f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);
//    if (1U == u1a_Status_id)
//    {
//      /* Read Id failed */
//      while (1u)
//      {
//      }
//    }
//    else
//    {
//      /* read Id ok */
//      while (1)
//      {
//      }
//    }
//  }


//  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOA, LED_0_Pin);
    HAL_GPIO_TogglePin(GPIOA, LED_1_Pin);
    HAL_Delay(500u);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void f_vol_flash_ena_write_volatile_req(void)
{
  uint8_t u1a_arr_cmd[1u] = {0x06};
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, u1a_arr_cmd, 1u, 10u);
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);
}
static void f_vol_flash_send_command(void)
{
  /* Comment buffer */
  static uint8_t u1l_arr_write_command_0[8U] = {0x42, 0x00, 0x10, 0x00, 0x01, 0x06, 0x06, 0x09};
  static uint8_t u1l_arr_write_command_1[8U] = {0x48, 0x00, 0x10, 0x00, 0xFF, 0xFF, 0xFF, 0XFF};
  static uint8_t u1l_arr_write_command_2[1U] = {0x98};

  // /* Unlock global block */
  // f_vol_flash_ena_write_volatile_req();
  // HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);
  // HAL_SPI_Transmit(&hspi1, u1l_arr_write_command_2, 1u, 10u);
  // HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);
  // /* Write status register 2 */
  // W25qxx_EraseSector(1u);
  // f_vol_flash_ena_write_volatile_req();
  // HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);
  // for(uint8_t u1a_loop_count = 0u; u1a_loop_count < 8u; u1a_loop_count++)
  // {
  //   HAL_SPI_Transmit(&hspi1, u1l_arr_write_command_0 + u1a_loop_count, 1u, 10u);
  // }
  // HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  /* Read status register 2 */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);
  for (uint8_t u1a_loop_count = 0u; u1a_loop_count < 9u; u1a_loop_count++)
  {
    if (u1a_loop_count < 5)
    {
      HAL_SPI_Transmit(&hspi1, u1l_arr_write_command_1 + u1a_loop_count, 1u, 10u);
    }
    else
    {
      HAL_SPI_Receive(&hspi1, u1l_arr_read_buffer + u1a_loop_count - 1u, 1u, 10u);
    }
  }
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);
  W25qxx_ReadSector((uint8_t *)u1l_buffer_read, 1u, 0u, 8u);
}
static void f_vol_flash_read_id(uint8_t *p2u1a_Status)
{
  uint8_t u1a_cmd[] = {0x9F};
  uint8_t u1a_recv_buff[3u];
  uint8_t u1a_FlashId[3U] = {0xEFU, 0x40U, 0x15U};
  uint8_t u1a_Status_ic;

  *p2u1a_Status = 0U;
  u1a_recv_buff[0U] = 0U;
  u1a_recv_buff[1U] = 1U;
  u1a_recv_buff[2U] = 2U;


    f_vol_flash_read_status_register((uint8_t *)&u1a_Status_ic);

//  if ( 1U == (u1a_Status_ic&1U) )
//  {
    /* Enable CS pin */
    HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);

    /* Transmit data */
    HAL_SPI_Transmit(&hspi1, u1a_cmd, sizeof(u1a_cmd), 10u);
    HAL_SPI_Receive(&hspi1, u1a_recv_buff, sizeof(u1a_recv_buff), 10u);
    /* Disable CS pin */
    HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

    for (uint8_t u1a_Cnt = 0U; u1a_Cnt < 3U; u1a_Cnt += 1u)
    {
      if (u1a_FlashId[u1a_Cnt] != u1a_recv_buff[u1a_Cnt])
      {
        *p2u1a_Status = 1U;
        //break;
      }
    }
//  }

  return;
}

static void f_vol_flash_read_status_register(uint8_t *p2u1a_Status)
{
  uint8_t u1a_cmd[] = {0x05U};
  uint8_t u1a_recv_buff[1U];

  *p2u1a_Status = 0U;

  /* Enable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);

  /* Transmit data */
  HAL_SPI_Transmit(&hspi1, u1a_cmd, sizeof(u1a_cmd), 10u);
  HAL_SPI_Receive(&hspi1, u1a_recv_buff, sizeof(u1a_recv_buff), 10u);
  /* Disable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  *p2u1a_Status = u1a_recv_buff[0U];

  return;
}

static void f_vol_flash_read_power_down( void )
{
  uint8_t u1a_cmd[] = {0xB9U};

  /* Enable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);

  /* Transmit data */
  HAL_SPI_Transmit(&hspi1, u1a_cmd, sizeof(u1a_cmd), 10u);
  /* Disable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  return;
}

static void f_vol_flash_read_REpower_down( void )
{
  uint8_t u1a_cmd[] = {0xABU};

  /* Enable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);

  /* Transmit data */
  HAL_SPI_Transmit(&hspi1, u1a_cmd, sizeof(u1a_cmd), 10u);
  /* Disable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  return;
}

static void f_vol_flash_read_enaWrite( void )
{
  uint8_t u1a_cmd[] = {0x06U};

  /* Enable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);

  /* Transmit data */
  HAL_SPI_Transmit(&hspi1, u1a_cmd, sizeof(u1a_cmd), 10u);
  /* Disable CS pin */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  return;
}

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

#ifdef USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
