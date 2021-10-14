#include <stdio.h>
#include "main.h"
#include <string.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"

#define MAX_IMPULS_PER_SECOND 150
#define LEFT_DIRECTION 0
#define RIGHT_DIRECTION 1
#define TIMER_PERIOD 200
#define ONE_SECOND 1000
#define QTR_SENSOR_BORDER 3200
#define PWM_MAX 255
#define PWM_MIN 0
#define XY_MAX 160
#define XY_CENTER 80
#define MAX_APP_VAL 100
#define MAX_ERRRORS_SUM 30

ADC_HandleTypeDef hadc;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);

char buff[100];
int len;

uint16_t adcInputs[8];
int8_t adcWeights[8] = {-50, -20, -10, -5, 5, 10, 20, 50};
int16_t adcError = 0;
int16_t adcError_t = 0;
uint8_t adcLine = 0;

uint8_t rxIndx;
char rxData[2];
char rxBuff[100];
char transferCplt;

uint16_t pwmLeft = 0;
uint16_t pwmRight = 0;

uint8_t encVal1 = 0;
uint8_t encVal2 = 0;
uint8_t encVal1t = 0;
uint8_t encVal2t = 0;

uint16_t encPos1 = 0;
uint16_t encPos2 = 0;
int pos1 = 0;
int pos2 = 0;

int8_t appVel = 0;
int8_t appKp = 0;
int8_t appKi = 0;
int8_t appKd = 0;

double pidKp, pidKi, pidKd;

double encCorrection = 0;

double pidError1 = 0;
double pidError1_t = 0;
double pidError1_sum = 0;
double pidSignal1 = 0;

double pidError2 = 0;
double pidError2_t = 0;
double pidError2_sum = 0;
double pidSignal2 = 0;

double dcVel_set = 0;

uint16_t dcVel1_r = 0;
uint16_t dcVel2_r = 0;

int parArray[4];

int16_t joyX = 0;
int16_t joyY = 0;
int8_t joyUpdate = 0;
uint8_t dcDirection = 0;

uint8_t counterCyclesPerSecond = 0;

enum menuOpt {robotStop, drivingForward, lineFollower, joystickControll};
enum menuOpt selectMode;

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == enc1a_Pin) encPos1++;
	if(GPIO_Pin == enc2a_Pin) encPos2++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART1) //aktualny UART
	{
		if(rxIndx == 0)
		{
			for(int i = 0; i < 100; i++) rxBuff[i] = 0; // czyszczenie buforu rx
		}
		if(rxData[0] != 64)
		{
			rxBuff[rxIndx++] = rxData[0]; // dodawanie danych do bufora
		}
		else
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			rxIndx = 0;
			transferCplt = 1; // transfer skończony, dane gotowe do odczytu
		}
		HAL_UART_Receive_IT(&huart1, rxData, 1); // aktywowanie przerwania uart dla każdego 1 bajta
	}
}

void send_string(char* s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

		dcVel1_r = encPos1 * counterCyclesPerSecond;
		dcVel2_r = encPos2 * counterCyclesPerSecond;

		pos1 += encPos1;
		pos2 += encPos2;

		encPos1 = 0;
		encPos2 = 0;
	}
}

void Robot_Stop()
{
	pwmLeft = 0;
	pwmRight = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmLeft);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmRight);
}

void PID_Controll()
{
	if(appVel > MAX_APP_VAL) appVel = MAX_APP_VAL;
	if(appKp > MAX_APP_VAL) appKp = MAX_APP_VAL;
	if(appKi > MAX_APP_VAL) appKi = MAX_APP_VAL;
	if(appKd > MAX_APP_VAL) appKd = MAX_APP_VAL;

	pidKp = (float)appKp / 100;
	pidKi = (float)appKi / 100;
	pidKd = (float)appKd / 100;

	dcVel_set = (PWM_MAX * appVel) / MAX_APP_VAL;
	pidError1 = dcVel_set - dcVel1_r;
	pidError2 = dcVel_set - dcVel2_r;

	pwmLeft = pidError1 * pidKp + pidError1_sum * pidKi + (pidError1 - pidError1_t) * pidKd;
	pwmRight = pidError2 * pidKp + pidError2_sum * pidKi + (pidError2 - pidError2_t) * pidKd;

	encCorrection = (float)pos1 / (float)pos2;
	if(encCorrection > 0) pwmRight *= encCorrection;
	else if(encCorrection < 0) pwmLeft *= encCorrection;

	pidError1_t = pidError1;
	pidError2_t = pidError2;
	pidError1_sum += pidError1;
	pidError2_sum += pidError2;

	if(pidError1_sum > MAX_ERRRORS_SUM) pidError1_sum = MAX_ERRRORS_SUM;
	if(pidError2_sum > MAX_ERRRORS_SUM) pidError2_sum = MAX_ERRRORS_SUM;
	if(pidError1_sum < -MAX_ERRRORS_SUM) pidError1_sum = -MAX_ERRRORS_SUM;
	if(pidError2_sum < -MAX_ERRRORS_SUM) pidError2_sum = -MAX_ERRRORS_SUM;

	if(pwmLeft > PWM_MAX) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MAX);
	else if(pwmLeft < PWM_MIN) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_MIN);
	else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmLeft);
	if(pwmRight > PWM_MAX) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MAX);
	else if(pwmRight < PWM_MIN) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MIN);
	else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmRight);
}

void QTR8_Read()
{
	for(int i = 0; i < 8; i++)
	{
		if (HAL_ADC_PollForConversion(&hadc, i) == HAL_OK)
		{
			adcInputs[i] = HAL_ADC_GetValue(&hadc);
			HAL_ADC_Start(&hadc);
		}

	}
}

uint16_t ADC_Error_Count()
{
	QTR8_Read();
	for(int i = 0; i < 8; i++)
	{
		if(adcInputs[i] > QTR_SENSOR_BORDER) adcInputs[i] = 1;
		else adcInputs[i] = 0;
		adcError += adcInputs[i] * adcWeights[i];
		adcLine += adcInputs[i];
	}
	adcError /= adcLine;
	return adcError;
}

int PD_Regulator()
{
	return (appKp * adcError + appKd * (adcError - adcError_t));
}

void LineFollower_PWM(int16_t dcLeft, int16_t dcRight)
{
	if(dcLeft >= PWM_MAX) dcLeft = PWM_MAX;
	if(dcRight >= PWM_MAX) dcRight = PWM_MAX;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, dcLeft);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dcRight);
}

void Joy_Controll()
{
	if(joyX < 0) joyX = 0;
	if(joyX > XY_MAX) joyX = XY_MAX;
	if(joyY < 0) joyY = 0;
	if(joyY > XY_MAX) joyY = XY_MAX;

	if(joyX > XY_CENTER && joyY < XY_CENTER)
	{
		// I quadrant
		pwmLeft = ((PWM_MAX * joyX * (XY_CENTER - joyY)) / XY_MAX) / XY_CENTER;
		pwmRight = ((PWM_MAX * (XY_MAX - joyX) * (XY_CENTER - joyY)) / XY_MAX) / XY_CENTER;
		dcDirection = RIGHT_DIRECTION;
	}
	if(joyX < XY_CENTER && joyY < XY_CENTER)
	{
		// II quadrant
		pwmLeft = ((PWM_MAX * joyX * (XY_CENTER - joyY)) / XY_MAX) / XY_CENTER;
		pwmRight = ((PWM_MAX * (XY_MAX - joyX) * (XY_CENTER - joyY)) / XY_MAX) / XY_CENTER;
		dcDirection = RIGHT_DIRECTION;
	}
	if(joyX < XY_CENTER && joyY > XY_CENTER)
	{
		//III quadrant
		pwmLeft = ((PWM_MAX * joyX * (joyY - XY_CENTER)) / XY_MAX) / XY_CENTER;
		pwmRight = ((PWM_MAX * (XY_MAX - joyX) * (joyY - XY_CENTER)) / XY_MAX) / XY_CENTER;
		dcDirection = LEFT_DIRECTION;
	}
	if(joyX > XY_CENTER && joyY > XY_CENTER)
	{
		//IV quadrant
		pwmLeft = ((PWM_MAX * joyX * (joyY - XY_CENTER)) / XY_MAX) / XY_CENTER;
		pwmRight = ((PWM_MAX * (XY_MAX - joyX) * (joyY - XY_CENTER)) / XY_MAX) / XY_CENTER;
		dcDirection = LEFT_DIRECTION;
	}

	Set_Direction(dcDirection);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmLeft);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmRight);
}

void Set_Direction(uint8_t direction)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, direction);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, direction);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_ADC_Start(&hadc);

  HAL_UART_Receive_IT(&huart1, rxData, 1); // aktywowanie przerwania uart dla każdego 1 bajta

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);

  counterCyclesPerSecond = ONE_SECOND / TIMER_PERIOD;

  while (1)
  {
	 if(transferCplt)
	 {
		 if(rxBuff[0] == 's') selectMode = robotStop;
		 else if(rxBuff[0] == 'f') selectMode = drivingForward;
		 else if(rxBuff[0] == 'l') selectMode = lineFollower;
		 else if(rxBuff[0] == 'j' && rxBuff[1] == 's') selectMode = joystickControll;
		 else if(rxBuff[0] == 'x')
		 {
			 sscanf(rxBuff, "x%d-%d-%d-%d", parArray, parArray + 1, parArray + 2, parArray + 3);
			 for(int i = 0; i < 4; i++)
			 {
				 if (parArray[i] < 0) parArray[i] = 0;
				 if (parArray[i] > 100) parArray[i] = 100;
			 }
			 appVel = parArray[0];
			 appKp = parArray[1];
			 appKi = parArray[2];
			 appKd = parArray[3];
		 }
		 transferCplt = 0;
	 }

	 switch(selectMode)
	 {
	 	 case robotStop:
	 	 {
	 		Robot_Stop();
	 		joyX = 0;
	 		joyY = 0;
	 		pidError1_t = 0;
	 		pidError2_t = 0;
	 		pidError1_sum = 0;
	 		pidError2_sum = 0;
	 		pos1 = 0;
	 		pos2 = 0;
	 		encCorrection = 0;
	 		break;
	 	 }
	 	 case drivingForward:
	 	 {
	 		Set_Direction(RIGHT_DIRECTION);
	 		PID_Controll();
			break;
	 	 }
	 	 case lineFollower:
	 	 {
	 		ADC_Error_Count();
	 		LineFollower_PWM(appVel + PD_Regulator(), appVel - PD_Regulator());
	 		adcError_t = adcError;
	 		//HAL_Delay(10);
	 		break;
	 	 }
	 	 case joystickControll:
	 	 {
	 		 if(rxBuff[3] == 'c' && rxBuff[4] == 'b' && rxBuff[6] == 'c' && rxBuff[7] == 'b')
	 		 {
	 			 joyX = (int)rxBuff[2];
	 			 joyY = (int)rxBuff[5];
	 			 Joy_Controll();
	 		 }
	 		//HAL_Delay(10);
	 		break;
	 	 }
	 	 default:
	 	 {
	 		 break;
	 	 }
	 }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_PERIOD - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIMER_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = enc2a_Pin|enc2b_Pin|enc1a_Pin|enc1b_Pin;;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{ 

}
#endif
