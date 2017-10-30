/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t     TIM4CH1_CAPTURE_STA=0X80;		//输入捕获状态		    				
uint32_t	TIM1_COUNTER_VAL[100];	        //定时器1计数组
uint32_t    TIM2_COUNTER_VAL[100];          //定时器2计数组
float	    TIM1_COUNTER_Val;	            //定时器1计数值
float       TIM2_COUNTER_Val;               //定时器2计数值
float       TIM_FREQ;                       //频率值
uint8_t     TIM4CH1_CAPTURE_GATE=0;         //预设闸门标志位
uint8_t     TIM4CH1_CAPTURE_STB=0;          //数组存满标志位
static uint8_t     i;                       //数组用
uint32_t	TIM1_COUNTER_TEMP;	            //定时器1计数滤波
uint32_t    TIM2_COUNTER_TEMP;              //定时器2计数滤波
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  //使能PWM波
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  //使能PWM波 
  
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1); //使能TIM4输入捕获中断
  __HAL_TIM_ENABLE_IT(&htim4, TIM_CHANNEL_1); //使能更新中断
  HAL_TIM_Base_Start_IT(&htim5);             //使能TIM5定时器中断

  
    __HAL_TIM_SET_COUNTER(&htim1,0);      //定时器1清零
    __HAL_TIM_SET_COUNTER(&htim2,0);      //定时器2清零 
  
  TIM4CH1_CAPTURE_GATE = 1;                   //开启预设闸门
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
 HAL_Delay(50);
	//	printf("jiance");
    if(TIM4CH1_CAPTURE_STB)
    {        
        TIM1_COUNTER_Val = (float) TIM1_COUNTER_TEMP / 100;
        TIM2_COUNTER_Val = (float) TIM2_COUNTER_TEMP / 100;        
        TIM_FREQ = (TIM2_COUNTER_Val*6000.0)/TIM1_COUNTER_Val;
        
        printf("TIM1:%.2f\r\n",TIM1_COUNTER_Val);
        printf("TIM2:%.2f\r\n",TIM2_COUNTER_Val);
        printf("FREQ:%.2fKHZ\r\n",TIM_FREQ);
        
        TIM1_COUNTER_TEMP = 0;
        TIM2_COUNTER_TEMP = 0;
        TIM4CH1_CAPTURE_STB = 0;
        TIM4CH1_CAPTURE_STA |= 0X80;
        HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
    }    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */
//定时器TIM4输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{	 

			if(TIM4CH1_CAPTURE_GATE==0)
			{   
				if(TIM4CH1_CAPTURE_STA&0X40)	//第二次捕获到一个上升沿 		
				{	                      
					TIM4CH1_CAPTURE_STA = 0X80;		//标记成功捕获到一次高电平脉宽
					TIM1_COUNTER_VAL[i] = __HAL_TIM_GET_COUNTER(&htim1); //获得定时器1计数值
					TIM2_COUNTER_VAL[i] = __HAL_TIM_GET_COUNTER(&htim2); //获得定时器2计数值
				 
					TIM1_COUNTER_TEMP += TIM1_COUNTER_VAL[i];
					TIM2_COUNTER_TEMP += TIM2_COUNTER_VAL[i];

					i++;
							if(i == 100)
							{                        
									i = 0;
									TIM4CH1_CAPTURE_STB = 1;
									TIM4CH1_CAPTURE_STA = 0;
									HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
							}					
					HAL_TIM_Base_Stop(&htim1);            //关闭定时器1
					HAL_TIM_Base_Stop(&htim2);            //关闭定时器2   
					__HAL_TIM_SET_COUNTER(&htim1,0);      //定时器1清零
					__HAL_TIM_SET_COUNTER(&htim2,0);      //定时器2清零                

			}
			

}
			if(TIM4CH1_CAPTURE_GATE==1)
			{
					if(TIM4CH1_CAPTURE_STA&0X80)//第一次捕获上升沿
	{
						TIM4CH1_CAPTURE_STA = 0X40;		//标记捕获到了上升沿
						HAL_TIM_Base_Start(&htim1);     //使能定时器1，对标准信号计数
						HAL_TIM_Base_Start(&htim2);     //使能定时器2，对被测信号计数
				}
	}		    
		
}
 //定时器TIM5溢出中断处理回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(TIM5 == htim->Instance)
    {
       if(TIM4CH1_CAPTURE_GATE)
       {
            TIM4CH1_CAPTURE_GATE = 0;
       }
       else 
       {
            TIM4CH1_CAPTURE_GATE = 1;
       }
    }
}    

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch ,1, 0xffff);
    
    return ch;
}
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
