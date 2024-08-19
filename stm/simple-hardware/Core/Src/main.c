/* Author : Roche Christopher */

# include "main.h"
#include "string.h"


TIM_HandleTypeDef timer2;

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  // enable the GPIOB clock
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Initialize the Green LED control registers - PB1
   * Set the control for GPIOB port as output
   */

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*
   * Initialize timer 2 - TIM2
   * Set the counter controls and enable it
   *
   */

  uint32_t timer_period = 10000;
  uint32_t target_frequency = 8;
  uint32_t prescaler = (HAL_RCC_GetHCLKFreq()/(timer_period * target_frequency)) - 1;

  __HAL_RCC_TIM2_CLK_ENABLE();
  timer2.Instance = TIM2;
  timer2.Init.Prescaler = prescaler;
  timer2.Init.Period = timer_period;
  timer2.Init.CounterMode = TIM_COUNTERMODE_UP;

  HAL_TIM_Base_Init(&timer2);
  HAL_TIM_Base_Start_IT(&timer2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Let the device alone to meditate. Do not disturb it. Please!!!!
  }
  /* USER CODE END 3 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // Set priority for TIM2 interrupt
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);

        // Enable TIM2 interrupt in NVIC
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}


void TIM2_IRQHandler(void) {
    // Check whether TIM2 update interrupt is pending
    if (__HAL_TIM_GET_FLAG(&timer2, TIM_FLAG_UPDATE) != 0) {
        // Clear the update interrupt flag
        __HAL_TIM_CLEAR_IT(&timer2, TIM_IT_UPDATE);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Toggle the LED
    }
}
