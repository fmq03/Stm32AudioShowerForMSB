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
#include "arm_const_structs.h"

#define APP_MENU 0
#define APP_WAVE 1
#define APP_FFT64 2
#define APP_FFT1024 3
#define APP_AUDIO 4
#define APP_DEBUG 5

#define MAXFFTNUM 1024
#define FFTNUMB 1024
#define PHI 0.46 
#define THRESHOLD 11

#define APP_OK 1
#define APP_ING 0
#define APP_WAIT 2

#define SQR(x) ((x)*(x))


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
u8 arrAUDOT[128];
u8 arrAUHANG[128];
uint32_t arrValue[MAXFFTNUM];
float32_t FFTin[MAXFFTNUM];
float32_t FFTout[MAXFFTNUM];
float32_t FFTmag[MAXFFTNUM];
u32 ite=0,cnt=0;

u8 arr_status,arr_laststatus;
u8 app_status;
u8 menu_status=APP_WAVE;
u8 ADC_status=APP_OK;
u8 wave_rate=4;
u8 fft_page=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Arr_Got();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float32_t Hann_Window(u16 n,u16 N){
	if(n==N)return 0.0; else
	return (1-PHI)-PHI*arm_cos_f32(2.0*3.14159*n/(N-1));
}

void FFT_Audio(){
	if(arr_status!=APP_OK)return;
	arm_rfft_fast_instance_f32 S;
	for(u32 i=0;i<1024;++i)FFTin[i]=arrValue[i]/512.0;
	Arr_Got();
	if(arm_rfft_fast_init_f32(&S,FFTNUMB)==ARM_MATH_ARGUMENT_ERROR)Error_Handler();
	arm_rfft_fast_f32(&S, FFTin, FFTout, 0);
	arm_cmplx_mag_f32(FFTout, FFTmag, FFTNUMB/2);
	FFTmag[0]=0;
}
void Arr_Got(){
	ADC_status=APP_OK;
	arr_status=APP_ING;
	return;
}
void App_ShowMenu(){
	OLED_Clear();
	OLED_ShowString(0,0,"Menu",16,1);
	if(menu_status==APP_WAVE)OLED_ShowString(0,16,"1.Show Wave <---",12,1); else OLED_ShowString(0,16,"1.Show Wave",12,1);
	if(menu_status==APP_FFT64)OLED_ShowString(0,28,"2.64FFT <---",12,1); else OLED_ShowString(0,28,"2.64FFT",12,1);
	if(menu_status==APP_FFT1024)OLED_ShowString(0,40,"3.AudioFFT <---",12,1);else OLED_ShowString(0,40,"3.AudioFFT",12,1);
	if(menu_status==APP_AUDIO)OLED_ShowString(0,52,"4.Audio <---",12,1); else OLED_ShowString(0,52,"4.Audio",12,1);
	OLED_Refresh();
}


void App_FFT64(){
	if(arr_status!=APP_OK)return;
	arm_rfft_fast_instance_f32 S;
	for(u32 i=0;i<1024;++i)FFTin[i]=arrValue[i]/4096.0;
	Arr_Got();
	if(arm_rfft_fast_init_f32(&S,64)==ARM_MATH_ARGUMENT_ERROR)Error_Handler();
	arm_rfft_fast_f32(&S, FFTin, FFTout, 0);
	//FFTout[0]=FFTout[1]=0;                                                       
	arm_cmplx_mag_f32(FFTout, FFTmag, 32);
	FFTmag[0]=0;
	OLED_Clear();
	for(u32 i=0;i<128;++i){
		u32 val=FFTmag[i/4]*62;
		if(val<=10)val=0;
		else{
			if(FFTmag[i/4]>FFTmag[i/4-1] && FFTmag[i/4]>FFTmag[i/4+1])
				val=sqrt(FFTmag[i/4]*FFTmag[i/4]+FFTmag[i/4-1]*FFTmag[i/4-1]+FFTmag[i/4+1]*FFTmag[i/4+1])*62;
			else val=0;
		}
		if(val>63)val=62;
		OLED_DrawLine(i,63-val,i,64,1);
	}
	OLED_Refresh();
	return;
}
void App_FFT1024(u8 page){
	FFT_Audio();
	OLED_Clear();
	for(u32 i=0;i<128;++i){
		OLED_DrawLine(i,63-(u32)FFTmag[i+(page-1)*128],i,64,1);
	}
	OLED_Refresh();
	return;
}
void App_Audio(){
	FFT_Audio();
	OLED_Clear();
	for(u16 i=0;i<64;++i){
		float32_t val;
		arm_sqrt_f32(SQR(FFTmag[i*3+2])/3+SQR(FFTmag[i*3+3])/3+SQR(FFTmag[i*3+4])/3,&val);
		val=(u32)val;
		if(arrAUDOT[i]<val){
			arrAUDOT[i]=(val>61)?61:val+1;
			arrAUHANG[i]=3;
		}else{
			if(arrAUHANG[i]>0)arrAUHANG[i]--;
			else arrAUDOT[i]=(arrAUDOT[i]>3)?arrAUDOT[i]-3:1;
			OLED_DrawLine(i*2,63-arrAUDOT[i],i*2+1,63-arrAUDOT[i],1);
			OLED_DrawLine(i*2,63-arrAUDOT[i]+1,i*2+1,63-arrAUDOT[i]+1,1);
		}
		OLED_DrawLine(i*2,63-val,i*2,64,1);
		OLED_DrawLine(i*2+1,63-val,i*2+1,64,1);
	}
	OLED_Refresh();
	return;
}
void Ding(void){ //LED light3times
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
	u8 keystate=HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
	if(lstate){	//l is up				
		if(app_status==APP_MENU){menu_status=(menu_status==1)?4:menu_status-1; HAL_Delay(100);}
		if(app_status==APP_WAVE && wave_rate>1){wave_rate/=2; HAL_Delay(100);}
		if(app_status==APP_FFT1024 && fft_page>1){fft_page-=1; HAL_Delay(100);}
	}
	if(!keystate){
		if(app_status==APP_MENU)app_status=menu_status;else app_status=APP_MENU;
		Ding();
	}
	if(rstate){//r is down
		//HAL_Delay(10);
		if(app_status==APP_MENU){menu_status=(menu_status==4)?1:menu_status+1; HAL_Delay(100);}
		if(app_status==APP_WAVE && wave_rate<8){wave_rate*=2; HAL_Delay(100);}
		if(app_status==APP_FFT1024 && fft_page<4){fft_page+=1; HAL_Delay(100);}
	}
}

void App_ShowWave(u32 rate){
	if(arr_status!=APP_OK)return;
	OLED_Clear();
	for(u32 col=1;col<128;++col){
		u32 val=64-(arrValue[col*rate]/4096.0*62);
		u32 lastval=64-arrValue[(col-1)*rate]/4096.0*62;
		OLED_DrawPoint(col,val,1);
		if(lastval<val)OLED_DrawLine(col-1,lastval,col,val,1); else OLED_DrawLine(col,val,col-1,lastval,1);
	}
	OLED_Refresh();
	Arr_Got();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //ADC所产生的回调函数
{
	if(ADC_status==APP_WAIT)return;
	if(arr_laststatus!=app_status){
		arr_status=APP_ING;
		arr_laststatus=app_status;
	//	for(u16 i=0;i<MAXFFTNUM;++i)arrValue[i]=0;
		cnt=ite=0;
	}
	arrValue[ite]=HAL_ADC_GetValue(&hadc1);
	++ite;
	if(ite==MAXFFTNUM-1){
		arr_status=APP_OK;
		ADC_status=APP_WAIT;
		ite=0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	//testOLED_Write_Init();
	//HAL_Delay(1000);
	Ding();
	OLED_Init();
	APP_ShowWelcome();
	HAL_TIM_Base_Start(&htim3);
	if(HAL_ADC_Start_IT(&hadc1)!=HAL_OK)Error_Handler();
	HAL_Delay(200);
	Ding();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//app_status=APP_AUDIO;
  while (1)
  {
		KeyDect();
		switch(app_status){
			case APP_MENU:
				App_ShowMenu();
				break;
			case APP_FFT64:
				App_FFT64();
			//App_ShowWave(1);
				break;
			case APP_FFT1024:
				App_FFT1024(fft_page);
				break;
			case APP_WAVE:
				App_ShowWave(wave_rate);
				break;
			case APP_AUDIO:
				App_Audio();
			case APP_DEBUG:
				//App_FFT1024(fft_page);
				break;
		}
		
		
		//USARTPrintf("%d\n",arrValue[0]);
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
