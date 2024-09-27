/* Author : Roche Christopher */

# include "main.h"
#include "string.h"


TIM_HandleTypeDef timer2;
I2C_HandleTypeDef hi2c1;
uint8_t i2c_rx_buffer[1];

uint32_t prescaler = 10000;
uint32_t target_frequency = 8;
uint32_t timer_period  = 1;
uint8_t counter = 1;

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

  /* Initialize the RED LED */
  GPIO_InitTypeDef GPIO_InitStruct1 = {0};
  GPIO_InitStruct1.Pin = LED_RED_Pin;
  GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct1);

  MX_USART1_UART_Init();
  UART_Print("Initialization begins\n");

  /*
   * Initialize timer 2 - TIM2
   * Set the counter controls and enable it
   *
   */

  timer_period = (HAL_RCC_GetHCLKFreq()/(prescaler * target_frequency)) - 1;

  //__HAL_RCC_TIM2_CLK_ENABLE();
  timer2.Instance = TIM2;
  timer2.Init.Prescaler = prescaler;
  timer2.Init.Period = timer_period;
  timer2.Init.CounterMode = TIM_COUNTERMODE_UP;


  HAL_TIM_Base_Init(&timer2);
  HAL_TIM_Base_Start_IT(&timer2);

  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /* Initialize the I2C slave*/

  hi2c1.Instance = I2C1;
  hi2c1.Init.OwnAddress1 = (16 << 1); // The address is stored as 8 bit while the address in I2C protocol is 7 bit. So, shifting left once.
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if(HAL_I2C_Init(&hi2c1) != HAL_OK){
	  // Error occurred while trying to setup I2C - Turn on the RED LED to indicate the failure to the user
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }
  // If the following function is not called, the device would not even respond to address scans like i2cdetect
  HAL_I2C_EnableListen_IT(&hi2c1);

  UART_Print("Initialization Done!!\n");


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Let the device alone to meditate. Do not disturb it. Please!!!!

  }
  /* USER CODE END 3 */
}

/* The user code in the following function is moved to stm32h7xx_hal_msp.c */

//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
//
//    if (htim->Instance == TIM2) {
//        // Set priority for TIM2 interrupt
//        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
//
//        // Enable TIM2 interrupt in NVIC
//        HAL_NVIC_EnableIRQ(TIM2_IRQn);
//    }
//}

extern void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	uint32_t received_number = (uint32_t)(i2c_rx_buffer[0]);
	UART_Print("The received byte is ");
	UART_Print(i2c_rx_buffer[0]);
	UART_Print("\n");

	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

	target_frequency = received_number;
	timer_period = (HAL_RCC_GetHCLKFreq()/(prescaler * target_frequency*2)) - 1;
	timer2.Instance->CNT = 0; // reset the value of the counter
	timer2.Instance->ARR = timer_period; // writing to the autoreload register.

}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // Master is looking to transmit data
	{
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2c_rx_buffer, 1, I2C_FIRST_AND_LAST_FRAME);
	}
	/*else
	{
		// let us handle the master reading from the slave later.
	}
	*/
}

extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void TIM2_IRQHandler(void) {
    // Check whether TIM2 update interrupt is pending
    if (__HAL_TIM_GET_FLAG(&timer2, TIM_FLAG_UPDATE) != 0) {
        // Clear the update interrupt flag
        __HAL_TIM_CLEAR_IT(&timer2, TIM_IT_UPDATE);
       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Toggle the LED
    }
}

void Error_Handler(void) {
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // Indicate to the user that there was an error.
}
