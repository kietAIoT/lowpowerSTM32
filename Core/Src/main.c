/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void PinInit();
void RTC_Init();
void MuaLed();
void EnterStandBy();
void EnterStandByRegister();
void SetAlarm();
void KIET_reset_rtc_register();
void KIET_configure_rtc_register();
void revise();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
//		__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}

//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//HAL_GPIO_WrietPin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
void alertLED() {
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	if (_HAL_PWR_GET_FLAG(PWR_FLAG_SB)!=RESET) {
		_HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		printf("#RED# Wakeup from the STANDBY MODE\n\n");
		for (int i = 0; i<= 5; i++) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}
//	HAL_PWR_DisableBkUpAccess()
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); // disbale PA0
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
//  PinInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
//  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  PinInit();
  MuaLed();
  RTC_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //	RTC->DIVH = 0x0000U;
	  //	RTC->DIVL = 0x8000U;
//	  	printf("CHEKC POINT 3: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
//	  	RTC->CNTH = 0x0000U;
//	  	RTC->CNTL = 0x0000U;
//
//	  	RTC->ALRH = 0x0000U;
//	  	RTC->ALRL = 0x000AU;
//	  	SET_BIT(RTC->CRH, RTC_CRH_ALRIE);
//	  	SET_BIT(RTC->CRH, RTC_CRH_ALRIE);
	printf("RUN MODE\n");
//	HAL_Delay(1000);
	MuaLed();
//	SET_BIT(PWR->CR, PWR_CR_CSBF);
	SET_BIT(PWR->CR, PWR_CR_CWUF); //Quan trong bit này, vì sau khi nhận được sự kiện wakup thì phải tắt,
	//Sự kiện WAKUP đo sđi, nếu không nó thấy rằng
//	SetAlarm();
	revise();
	EnterStandBy();
	printf("CHECK RTC_CRL_ALRG %d\n", READ_BIT(RTC->CRL, RTC_CRL_ALRF));
	printf("CHEKC RTC_DIVL: %d\n", RTC->DIVL);
	printf("CHEKC RTC_CNTL: %d\n", RTC->CNTL);
	printf("CHEKC RTC_ANRL: %d\n", RTC->ALRL);
	HAL_Delay(200);


		//	HAL_Delay(100);
		//	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB)!=RESET) {
		//		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		//		printf("#RED# Wakeup from the STANDBY MODE\n\n");
		//		for (int i = 0; i<= 5; i++) {
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		//			HAL_Delay(100);
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		//		}
		//	//	HAL_PWR_DisableBkUpAccess()
		//		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); // disbale
		//	}
		//	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		//	printf("Enter STANDBY\n");
		//	for (uint8_t i=0; i<=5; i++) {
		//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		//		HAL_Delay(1000);
		//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		//	}
		//	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

//		PWR->CR = PWR->CR * 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void PinInit() {
	for (uint8_t i = 4; i<= 7; i++) {
		GPIOB->CRL = (GPIOB->CRL&~(0b11<<(i*4+2)))|(0<<(4*i+2)); //DescriptMode
		GPIOB->CRL = (GPIOB->CRL&~(0b11<<(i*4)))|(1<<(4*i)); //MODE
	}
	for (uint8_t i = 8 ; i<=9; i++) {
		uint8_t position = i - 7;
		GPIOB->CRH = (GPIOB->CRH&~(0b11<<(position*4+2)))|(0b0<<(4*position+2));
		GPIOB->CRH = (GPIOB->CRH&~(0b11<<(position*4)))|(0b01<<4*position);
	}
	for (uint8_t i = 4; i<= 9; i++) {
		GPIOB->BSRR = (GPIOB->BSRR &~(0b1<<i))|(0b0<<i);
		GPIOB->BSRR = (GPIOB->BSRR &~(0b1<<(i+16)))|(0b0<<(i+16));
	}
	int temp = GPIOB->CRL;
	printf("%d /n", temp);
	temp = GPIOB->CRH;
	printf("%d /n", temp);
}

void MuaLed() {

	for (uint8_t i=4; i<=9; i++) GPIOB->BSRR = (GPIOB->BSRR&~(0b1<<(i+16)))|(1<<(i+16));
	for (uint8_t i= 4; i<= 9; i++) {
		GPIOB->BSRR = (GPIOB->BSRR&~(0b1<<(i)))|(1<<(i));
		HAL_Delay(100);
		GPIOB->BSRR = (GPIOB->BSRR&~(0b1<<(i)))|(0<<(i));
		GPIOB->BSRR = (GPIOB->BSRR&~(0b1<<(i+16)))|(1<<(i+16));
	}
}

void EnterStandBy() {
	printf("Entering StandBy");
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_EnterSTANDBYMode();
}

//void EnterStandByRegister() {
//	SETBIT(PWR->CR, PWR_CR_CSBF);
//}

void SetAlarm() {
	RTC_AlarmTypeDef a;
	a.Alarm = RTC_ALARM_A;
	a.AlarmTime.Hours = 0x00;
	a.AlarmTime.Minutes = 0;
	a.AlarmTime.Seconds = 10;
	hrtc.Instance = RTC;
	HAL_RTC_SetAlarm(&hrtc, &a, RTC_FORMAT_BIN);
}



void RTC_Init(){
	/** @note */
	/* PWREN: Power interface clock enable
	 * Set and cleared by software.
	 * 0: Power interface clock disabled
	 * 1: Power interface clock enabl*/
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	/* BKPEN: Backup interface clock enable
	 * Set and cleared by software.
	 * 0: Backup interface clock disabled
	 * 1: Backup interface clock enabled*/
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN);

	/* Bit 8 DBP: Disable backup domain write protection.
	 * In reset state, the RTC and backup registers are protected against parasitic write access.
	 * This bit must be set to enable write access to these registers.
	 * 0: Access to RTC and Backup registers disabled
	 * 1: Access to RTC and Backup registers enabled
	 * Note: If the HSE divided by 128 is used as the RTC clock, this bit must remain set to 1.*/
	SET_BIT(PWR->CR, PWR_CR_DBP);
	/*RTCEN: RTC clock enable
	 * Set and cleared by software.
	 * 0: RTC clock disabled
	 * 1: RTC clock enabled
	 * */
	printf("CHEKC POINT -1: %d\n", READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY));
	SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
	/* Bit 1 LSERDY: External low-speed oscillator ready
	 * Set and cleared by hardware to indicate when the external 32 kHz oscillator is stable. After
	 * the LSEON bit is cleared, LSERDY goes low after 6 external low-speed oscillator clock cycles.
	 * 0: External 32 kHz oscillator not ready
	 * 1: External 32 kHz oscillator ready*/
//	printf("BIT %d: \n", RCC_BDCR_LSEBYP);
	uint8_t count = 0;

	while (READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)==0) {
		count++;
		if (count>100) {
			printf("Have a problem for connecting the External Crystal Clock!");
			break;
		} else;
	}
//	printf("BDCR: %d\n", RCC->BDCR);

	SET_BIT(RCC->BDCR, 9);

	SET_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
	printf("CHEKC POINT -1: %d\n", RCC->BDCR);
//	printf("READ BIT: %d\n", READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN));
	/* Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	 * Set by software to select the clock source for the RTC. Once the RTC clock source has been
	 * selected, it cannot be changed anymore unless the Backup domain is reset. The BDRST bit can be used to reset them.
	 * 00: No clock
	 * 01: LSE oscillator clock used as RTC clock
	 * 10: LSI oscillator clock used as RTC clock
	 * 11: HSE oscillator clock divided by 128 used as RTC clock*/
//	RCC->BDCR |= (0b00<<8);

//	SET_BIT(RCC->BDCR, 8);

	/* Bit 0 LSEON: External low-speed oscillator enable (Low speed external)
	 * Set and cleared by software.
	 * 0: External 32 kHz oscillator OFF
	 * 1: External 32 kHz oscillator ON*/

	KIET_configure_rtc_register();
}



void KIET_configure_rtc_register() {
	/* To write in the RTC_PRL, RTC_CNT, RTC_ALR registers, the peripheral must enter
	 * Configuration mode. This is done by setting the CNF bit in the RTC_CRL register
	 * In addition, writing to any RTC register is only enabled if the previous write operation is finished.
	 * To enable the software to detect this situation, the RTOFF status bit is provided in
	 * the RTC_CR register to indicate that an update of the registers is in progress. A new value
	 * can be written to the RTC registers only when the RTOFF status bit value is ’1’*/
	/* 1. Poll RTOFF, wait until its value goes to ‘1
	 * 2. Set the CNF bit to enter configuration mode
	 * 3. Write to one or more RTC registers
	 * 4. Clear the CNF bit to exit configuration mode
	 * 5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation*/
	printf("CHEKC POINT 0: %d\n", READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)); //OKEE ==> VẤn đ�? ở CLOK chưa được kết nối vào
	while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L1_Check RTOFF\n");}
	/* Bit 4 CNF: Configuration flag
	 * This bit must be set by software to enter in configuration mode so as to allow new values to
	 * be written in the RTC_CNT, RTC_ALR or RTC_PRL registers. The write operation is only
	 * executed when the CNF bit is reset by software after has been set.
	 * 0: Exit configuration mode (start update of RTC registers).
	 * 1: Enter configuration mode.*/
	SET_BIT(RTC->CRL, RTC_CRL_CNF);
	printf("CHEKC POINT 1: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	/*Begin for writing to RTC Register - Write one or more RTC register*/
	RTC->PRLH = 0U;
	RTC->PRLL = 0x7FFFU;
	printf("CHEKC POINT 2: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
//	RTC->DIVH = 0x0000U;
//	RTC->DIVL = 0x8000U;
	printf("CHEKC POINT 3: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	RTC->CNTH = 0x0000U;
	RTC->CNTL = 0x0000U;

	RTC->ALRH = 0x0000U;
	RTC->ALRL = 0x0004U;
	SET_BIT(RTC->CRH, RTC_CRH_ALRIE);
	SET_BIT(RTC->CRH, RTC_CRH_ALRIE);
	/* During each period of TR_CLK, the counter inside the RTC prescaler is reloaded with the
	 * value stored in the RTC_PRL register. To get an accurate time measurement it is possible to
	 * read the current value of the prescaler counter, stored in the RTC_DIV register, without
	 * stopping it. This register is read-only and it is reloaded by hardware after any change in the
	 * RTC_PRL or RTC_CNT registers.*/

	/*End of writing to RTC register*/
	CLEAR_BIT(RTC->CRL, RTC_CRL_CNF);
	printf("CHEKC POINT 4: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	/* Bit 5 RTOFF: RTC operation OFF With this bit the RTC reports the status of the last write operation performed on its registers,
	 * indicating if it has been completed or not. If its value is ‘0’ then it is not possible to write to any
	 * of the RTC registers. This bit is read only.
	 * 0: Last write operation on RTC registers is still ongoing.
	 * 1: Last write operation on RTC registers terminated.*/
	while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L2_Ongoing in other command\n %d",READ_BIT(RTC->CRL, RTC_CRL_RTOFF) );}
	printf("Done configuration RTC\n");

}


void revise() {
	/* To write in the RTC_PRL, RTC_CNT, RTC_ALR registers, the peripheral must enter
		 * Configuration mode. This is done by setting the CNF bit in the RTC_CRL register
		 * In addition, writing to any RTC register is only enabled if the previous write operation is finished.
		 * To enable the software to detect this situation, the RTOFF status bit is provided in
		 * the RTC_CR register to indicate that an update of the registers is in progress. A new value
		 * can be written to the RTC registers only when the RTOFF status bit value is ’1’*/
		/* 1. Poll RTOFF, wait until its value goes to ‘1
		 * 2. Set the CNF bit to enter configuration mode
		 * 3. Write to one or more RTC registers
		 * 4. Clear the CNF bit to exit configuration mode
		 * 5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation*/
		while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L1_Check RTOFF\n");}
		/* Bit 4 CNF: Configuration flag
		 * This bit must be set by software to enter in configuration mode so as to allow new values to
		 * be written in the RTC_CNT, RTC_ALR or RTC_PRL registers. The write operation is only
		 * executed when the CNF bit is reset by software after has been set.
		 * 0: Exit configuration mode (start update of RTC registers).
		 * 1: Enter configuration mode.*/
		SET_BIT(RTC->CRL, RTC_CRL_CNF);
		/*Begin for writing to RTC Register - Write one or more RTC register*/
	//	RTC->DIVH = 0x0000U;
	//	RTC->DIVL = 0x8000U;
		RTC->CNTH = 0x0000U;
		RTC->CNTL = 0x0000U;

		/* During each period of TR_CLK, the counter inside the RTC prescaler is reloaded with the
		 * value stored in the RTC_PRL register. To get an accurate time measurement it is possible to
		 * read the current value of the prescaler counter, stored in the RTC_DIV register, without
		 * stopping it. This register is read-only and it is reloaded by hardware after any change in the
		 * RTC_PRL or RTC_CNT registers.*/

		/*End of writing to RTC register*/
		CLEAR_BIT(RTC->CRL, RTC_CRL_CNF);
		/* Bit 5 RTOFF: RTC operation OFF With this bit the RTC reports the status of the last write operation performed on its registers,
		 * indicating if it has been completed or not. If its value is ‘0’ then it is not possible to write to any
		 * of the RTC registers. This bit is read only.
		 * 0: Last write operation on RTC registers is still ongoing.
		 * 1: Last write operation on RTC registers terminated.*/
		while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L2_Ongoing in other command\n %d",READ_BIT(RTC->CRL, RTC_CRL_RTOFF) );}
}
void KIET_reset_rtc_register() {
	/* All system registers are asynchronously reset by a System Reset or Power Reset, except*/
	/* for RTC_PRL, RTC_ALR, RTC_CNT, and RTC_DIV.
	 * The backup domain has two specific resets that affect only the backup domain (see Figure 4).
	 * A backup domain reset is generated when one of the following events occurs:
	 * 1. Software reset, triggered by setting the BDRST bit in the Backup domain control register (RCC_BDCR).
	 * 2. VDD or VBAT power on, if both supplies have previously been powered off.*/
	SET_BIT(RCC->BDCR, RCC_BDCR_BDRST);
	HAL_Delay(1);
	CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);
}


//HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
//{
//  uint32_t counter_alarm = 0U, counter_time;
//  RTC_TimeTypeDef stime = {0U};
//
//  /* Check input parameters */
//  if ((hrtc == NULL) || (sAlarm == NULL))
//  {
//    return HAL_ERROR;
//  }
//
//  /* Check the parameters */
//  assert_param(IS_RTC_FORMAT(Format));
//  assert_param(IS_RTC_ALARM(sAlarm->Alarm));
//
//  /* Process Locked */
//  __HAL_LOCK(hrtc);
//
//  hrtc->State = HAL_RTC_STATE_BUSY;
//
//  /* Call HAL_RTC_GetTime function to update date if counter higher than 24 hours */
//  if (HAL_RTC_GetTime(hrtc, &stime, RTC_FORMAT_BIN) != HAL_OK)
//  {
//    return HAL_ERROR;
//  }
//
//  /* Convert time in seconds */
//  counter_time = (uint32_t)(((uint32_t)stime.Hours * 3600U) + \
//                            ((uint32_t)stime.Minutes * 60U) + \
//                            ((uint32_t)stime.Seconds));
//
//  if (Format == RTC_FORMAT_BIN)
//  {
//    assert_param(IS_RTC_HOUR24(sAlarm->AlarmTime.Hours));
//    assert_param(IS_RTC_MINUTES(sAlarm->AlarmTime.Minutes));
//    assert_param(IS_RTC_SECONDS(sAlarm->AlarmTime.Seconds));
//
//    counter_alarm = (uint32_t)(((uint32_t)sAlarm->AlarmTime.Hours * 3600U) + \
//                               ((uint32_t)sAlarm->AlarmTime.Minutes * 60U) + \
//                               ((uint32_t)sAlarm->AlarmTime.Seconds));
//  }
//  else
//  {
//    assert_param(IS_RTC_HOUR24(RTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)));
//    assert_param(IS_RTC_MINUTES(RTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)));
//    assert_param(IS_RTC_SECONDS(RTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));
//
//    counter_alarm = (((uint32_t)(RTC_Bcd2ToByte(sAlarm->AlarmTime.Hours)) * 3600U) + \
//                     ((uint32_t)(RTC_Bcd2ToByte(sAlarm->AlarmTime.Minutes)) * 60U) + \
//                     ((uint32_t)RTC_Bcd2ToByte(sAlarm->AlarmTime.Seconds)));
//  }
//
//  /* Check that requested alarm should expire in the same day (otherwise add 1 day) */
//  if (counter_alarm < counter_time)
//  {
//    /* Add 1 day to alarm counter*/
//    counter_alarm += (uint32_t)(24U * 3600U);
//  }
//
//  /* Write alarm counter in RTC registers */
//  if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
//  {
//    /* Set RTC state */
//    hrtc->State = HAL_RTC_STATE_ERROR;
//
//    /* Process Unlocked */
//    __HAL_UNLOCK(hrtc);
//
//    return HAL_ERROR;
//  }
//  else
//  {
//    /* Clear flag alarm A */
//    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
//
//    /* Configure the Alarm interrupt */
//    __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);
//
//    /* RTC Alarm Interrupt Configuration: EXTI configuration */
//    __HAL_RTC_ALARM_EXTI_ENABLE_IT();
//
//    __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();
//
//    hrtc->State = HAL_RTC_STATE_READY;
//
//    __HAL_UNLOCK(hrtc);
//
//    return HAL_OK;
//  }
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
