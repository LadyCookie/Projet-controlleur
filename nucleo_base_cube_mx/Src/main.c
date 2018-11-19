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
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "servomoteur.h"
#include "handlerbutton.h"
#include "useless_actions.h"

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


	//Random action register
	void nothing(){};
	

	//handler configuration
	void HAL_TIM_PeriodElaspedCallback(TIM_HandleTypeDef *htim) {
		handlerbutton();
	}

	//timer configuration
	TIM_HandleTypeDef timerArm;
	TIM_OC_InitTypeDef timerArmOC;
	
	TIM_HandleTypeDef timerLid;
	TIM_OC_InitTypeDef timerLidOC;
	
	TIM_HandleTypeDef timerbutton;
	
	int max_arm=2000; //1472   pour le bras
  int min_arm=450; //352   pour le bras
	int max_lid=1400;
	int min_lid=900;
	
/* USER CODE END 0 */

int main(void)
	{

  /* USER CODE BEGIN 1 */
	
	Register_Action(&nothing,1);
		
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
/* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	
	//Initizalization of the button pin
	GPIO_InitTypeDef button;
	button.Pin=GPIO_PIN_5;
	button.Speed=0x2; //frequency 20ms
	button.Mode=GPIO_MODE_INPUT;
	
	HAL_GPIO_Init(GPIOB,&button);
	
	//initialization PWM for arm
	GPIO_InitTypeDef PinA1;
	PinA1.Pin=GPIO_PIN_1;
	PinA1.Speed=0x2;
	PinA1.Mode=GPIO_MODE_AF_PP;
	
	HAL_GPIO_Init(GPIOA,&PinA1);
	
	timerArm.Instance=TIM2;
	//CALCUL FREQUENCE
	timerArm.Init.Prescaler =99;
	timerArm.Init.Period = 17999; //pour 50Hz ie 20ms mettre 14399
	
	timerArmOC.OCMode=TIM_OCMODE_PWM1;
	timerArmOC.Pulse = min_arm;
	
	//initialization PWM for lid
	GPIO_InitTypeDef PinA0;
	PinA0.Pin=GPIO_PIN_0;
	PinA0.Speed=0x2;
	PinA0.Mode=GPIO_MODE_AF_PP;
	
	HAL_GPIO_Init(GPIOA,&PinA0);
	
	timerLid.Instance=TIM2;
	//CALCUL FREQUENCE
	timerLid.Init.Prescaler =99;
	timerLid.Init.Period = 17999; //pour 50Hz ie 20ms mettre 14399
	
	timerLidOC.OCMode=TIM_OCMODE_PWM1;
	timerLidOC.Pulse = min_lid;
	
	
	//Initializing
	HAL_TIM_PWM_MspInit(&timerArm);
	HAL_TIM_PWM_MspInit(&timerLid);
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	HAL_TIM_PWM_Init(&timerArm);
	HAL_TIM_PWM_ConfigChannel(&timerArm,&timerArmOC,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&timerArm,TIM_CHANNEL_2);
	
	
	//Initializing & activating TIM3 for button interruption
	timerbutton.Instance = TIM3;
	timerbutton.Init.Prescaler = 3599;
	timerbutton.Init.Period = 9999;
		
	HAL_TIM_Base_MspInit(&timerbutton);
	__HAL_RCC_TIM3_CLK_ENABLE();
	HAL_TIM_Base_Init(&timerbutton);
	HAL_TIM_Base_Start(&timerbutton);
	
	HAL_NVIC_SetPriority(29,10,10);
	HAL_NVIC_EnableIRQ(29);

	HAL_TIM_Base_Start_IT(&timerbutton);

	//SPI
	
	static SPI_HandleTypeDef spi = { .Instance = SPI1 };
  spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  spi.Init.Direction = SPI_DIRECTION_1LINE;
  spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  spi.Init.DataSize = SPI_DATASIZE_16BIT;
  spi.Init.FirstBit = SPI_FIRSTBIT_LSB;
  spi.Init.NSS = SPI_NSS_SOFT;
  spi.Init.TIMode = SPI_TIMODE_DISABLED;
  spi.Init.Mode = SPI_MODE_MASTER; 
  if (HAL_SPI_Init(&spi) != HAL_OK)
  {
		//TODO fail
  }
	GPIO_InitTypeDef  GPIO_InitStruct;
  
  GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	

	HAL_TIM_PWM_Init(&timerLid);
	HAL_TIM_PWM_ConfigChannel(&timerLid,&timerLidOC,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&timerLid,TIM_CHANNEL_1);

	/*Initialisation timer 3 pour interruption bouton*/

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
