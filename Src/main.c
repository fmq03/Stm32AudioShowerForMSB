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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "arm_math.h"
//#include "arm_const_structs.h"


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
uint32_t Adc_Value=0;
uint32_t arrValue[1024];
u16 ite=0,arrfull=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void testFFT(){

}
void Ding(void){
		int t=3;
		while(t--){
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
			HAL_Delay(70);
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			HAL_Delay(70);
		}
}
void KeyDect(){
	u8 lstate=HAL_GPIO_ReadPin(LBUT_GPIO_Port,LBUT_Pin);
	u8 rstate=HAL_GPIO_ReadPin(RBUT_GPIO_Port,RBUT_Pin); //left is PA2 right is PA3
	if(!lstate){
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	}
	if(!rstate){
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	}
	//if(!rstate)Ding();
	//if(!rstate)Ding();
}
void testOLED_swim(){
	for(u8 i=0;i<64;++i){
		u8 val=i;
		for(u8 j=0;j<128;++j){
			if(val>=64)val=0;
			OLED_DrawLine(j,0,j,63,0);
			OLED_DrawLine(j,0,j,val,1);
			++val;
			//OLED_Refresh();
		}
		OLED_Refresh();
	}
}
void testOLED_line(){
	for(u8 i=0;i<64;++i){
		OLED_DrawPoint(64,i,1);
		OLED_Refresh();
	}
	OLED_Clear();
	for(u8 i=1;i<64;++i){
		OLED_DrawPoint(64,63-i,1);
		OLED_Refresh();
	}
	OLED_Clear();
}
void testOLED_sin(){
	static int cur=0;
	OLED_Clear();
	for(u8 i=0;i<127;++i){
		u16 x=i+cur*50;
		float32_t xal;
		xal=x/10.0;
		OLED_DrawPoint(i,(int)(arm_sin_f32(xal)*20.0+31.0),1);
		
	}
	OLED_Refresh();
	//
	cur++;
	if(cur>=127)cur=0;
}
void testADC(){
	static u8 col=1;
	static u16 lastval=30;
	u16 adcvalue=65535;
	//HAL_Delay(1000);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,15);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
		adcvalue=HAL_ADC_GetValue(&hadc1);
		//OLED_Clear();
	//	OLED_ShowString(0,0,"ok",12,1);
		for(u8 i=col;i<col+6;++i)
		OLED_DrawLine(i,0,i,63,0);
		
		u16 val=(u16)(adcvalue/1023.0*30)+20;
		OLED_DrawPoint(col,val,1);
		if(lastval<val)OLED_DrawLine(col-1,lastval,col,val,1); else OLED_DrawLine(col,val,col-1,lastval,1);

		lastval=val;
		
	//	USARTPrintf("%d\n",adcvalue);
		
		OLED_Refresh();
		col++;
		if(col>=143)col=1;
	}

}
void Print_ADC_OLED(u32 division){
	OLED_Clear();
	for(u8 col=1;col<128;++col){
		u32 val=64-arrValue[col]/4096.0*62;
		u32 lastval=64-arrValue[col-1]/4096.0*62;
		OLED_DrawPoint(col,val,1);
		if(lastval<val)OLED_DrawLine(col-1,lastval,col,val,1); else OLED_DrawLine(col,val,col-1,lastval,1);
	}
	OLED_Refresh();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)//????
{
	if(ite==1023){
		arrfull=1;
		ite=0;
		//Print_ADC_OLED();
	}
 //	Ding();
	arrValue[ite]=HAL_ADC_GetValue(&hadc1);
	++ite;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	//testOLED_Write_Init();
	//HAL_Delay(1000);
	Ding();
	OLED_Init();
	OLED_ColorTurn(0);
	OLED_DisPlay_On();
	OLED_ShowString(5,25,"By fmq lyz mwx",16,1);
	OLED_Refresh();

	HAL_Delay(300);
	OLED_Clear();
	OLED_Refresh();
	HAL_TIM_Base_Start(&htim3);
	if(HAL_ADC_Start_IT(&hadc1)!=HAL_OK)Error_Handler();
	HAL_Delay(200);
	Ding();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(500);		
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);*/
		//OLED_ShowString(0,0,"test",12,1);
		//OLED_Refresh();
		Print_ADC_OLED(1);
		//HAL_Delay(500);
		//KeyDect();
		//testOLED_sin();
		//HAL_ADC_Star
		USARTPrintf("%d\n",arrValue[0]);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

/* USER CODE BEGIN 4 */

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
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
		HAL_Delay(200);
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
