/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "sbus_decoder.h"
#include "usbd_def.h"
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

/* USER CODE BEGIN PV */
uint8_t usbd_buf_recv[13];
char usbd_buf_send[128];
char pc_thrust[5];
char pc_rudder[5];
uint8_t sbus_buf[25];
uint32_t counter;
int16_t enc_count;
float rudder_angle;
int rudder_angle_to_send;
uint16_t adc_val;
float batt_mv;
RemoteControl_t rc;
ASV_t asv;
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RCUpdate(){
	if(rc_data.rawdata[7] < 1000)	rc.autonomous_mode = ASV_AUTO;
	if(rc_data.rawdata[7] > 1000)	rc.autonomous_mode = ASV_MANUAL;
	if(rc_data.rawdata[6] < 1000)	rc.fan_mode = FAN_ON;
	if(rc_data.rawdata[6] > 1000)	rc.fan_mode = FAN_OFF;
	if(rc_data.rawdata[5] < 1000)	rc.actuator_mode = ACTUATOR_ENABLE;
	if(rc_data.rawdata[5] > 1000)	rc.actuator_mode = ACTUATOR_DISABLE;

	rc.stk_y = rc_data.rawdata[1];
	rc.stk_x = rc_data.rawdata[0];

	rc.stk_y = (1035 - rc.stk_y)/ 695.0;
	rc.stk_x = (1000 - rc.stk_x)/ 693.0;

	if(rc.stk_y > 1) rc.stk_y = 1;
	if(rc.stk_y < -1) rc.stk_y = -1;
	if(rc.stk_x > 1) rc.stk_x = 1;
	if(rc.stk_x < -1) rc.stk_x = -1;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_UART_Receive_DMA(&huart1, sbus_buf, sizeof(sbus_buf));
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //STEER
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //THRUST
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //THRUST
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
  HAL_Delay(2000);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		sbus_decoder_get_buf(sbus_buf, sizeof(sbus_buf));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM5){
		RCUpdate();
		HAL_ADC_Start_IT(&hadc1);
//		counter = __HAL_TIM_GET_COUNTER(&htim1);
		if(rc.autonomous_mode == ASV_AUTO){
			if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
				if(usbd_buf_recv[0] == 'A' && usbd_buf_recv[1] == 'B' && usbd_buf_recv[11] == 'B' && usbd_buf_recv[12] == 'A'){
					memcpy(pc_rudder, usbd_buf_recv + 2, 4);
					pc_rudder[4] = 0;
					memcpy(pc_thrust, usbd_buf_recv + 7, 4);
					pc_thrust[4] = 0;
					asv.thrust = atoi(pc_thrust);
					asv.thrust = asv.thrust - 1500;
					asv.steer = atoi(pc_rudder);
				}
		//		memset(usbd_buf_recv, NULL, sizeof(usbd_buf_recv));
			}else{
				memset(usbd_buf_recv, NULL, sizeof(usbd_buf_recv));
				asv.thrust = 0;
				asv.steer = 1500;
			}

		}else if(rc.autonomous_mode == ASV_MANUAL){
			asv.thrust = rc.stk_y * 500;
			asv.steer = (rc.stk_x * 500) + 1500;
		}

		if(rc.fan_mode == FAN_ON){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		}else if(rc.fan_mode == FAN_OFF){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}

		if(rc.actuator_mode == ACTUATOR_ENABLE){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}else if(rc.actuator_mode == ACTUATOR_DISABLE){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		}

		if(asv.thrust >= 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, asv.thrust);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		}else if(asv.thrust < 0){
			asv.thrust = asv.thrust * (-1);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, asv.thrust);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		}
		sprintf(usbd_buf_send, "%d", rudder_angle_to_send);
//		*usbd_buf_send = ["Hello wrld"];
		CDC_Transmit_FS((uint8_t*)usbd_buf_send, 7);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, asv.steer);
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		counter = __HAL_TIM_GET_COUNTER(&htim1);
		enc_count = (int16_t)counter / 4;
		rudder_angle = (enc_count / 600.00 * 360.00);
		rudder_angle_to_send = rudder_angle * 100;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_val = HAL_ADC_GetValue(&hadc1);
	batt_mv = adc_val * 0.0075;
  /*If continuousconversion mode is DISABLED uncomment below*/
//  HAL_ADC_Start_IT (&hadc1);
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
