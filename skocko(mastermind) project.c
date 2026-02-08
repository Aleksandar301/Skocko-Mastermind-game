/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Skocko game (Mastermind) for Nucleo-L476RG
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* Private define ------------------------------------------------------------*/
#define MAX_DIGIT 6
#define TOTAL_DIGITS 4
#define MAX_GUESSES 6

#define SEG_A_GPIO GPIOA
#define SEG_A_PIN  GPIO_PIN_10
#define SEG_B_GPIO GPIOA
#define SEG_B_PIN  GPIO_PIN_9
#define SEG_C_GPIO GPIOA
#define SEG_C_PIN  GPIO_PIN_8
#define SEG_D_GPIO GPIOB
#define SEG_D_PIN  GPIO_PIN_10
#define SEG_E_GPIO GPIOB
#define SEG_E_PIN  GPIO_PIN_5
#define SEG_F_GPIO GPIOB
#define SEG_F_PIN  GPIO_PIN_4
#define SEG_G_GPIO GPIOB
#define SEG_G_PIN  GPIO_PIN_3

#define SEL1_GPIO GPIOB
#define SEL1_PIN  GPIO_PIN_6
#define SEL2_GPIO GPIOC
#define SEL2_PIN  GPIO_PIN_7

const uint8_t digits[10] = {
    0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
    0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
RNG_HandleTypeDef hrng;

uint8_t secret[TOTAL_DIGITS];
uint8_t currentGuess[TOTAL_DIGITS];
uint8_t lockedDigits = 0;
uint8_t currentTry = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);

uint8_t ButtonPressed(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET){
        HAL_Delay(50);
        if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET){
            while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET);
            HAL_Delay(50);
            return 1;
        }
    }
    return 0;
}

uint8_t readPotentiometer(void){
    uint32_t adcValue = 0;
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK){
        adcValue = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    uint16_t step = 4096 / MAX_DIGIT;
    uint8_t broj = (adcValue / step) + 1;
    if(broj>MAX_DIGIT) broj=MAX_DIGIT;
    return broj;
}

void set_segments(uint8_t value){
    HAL_GPIO_WritePin(SEG_A_GPIO,SEG_A_PIN,(value & 0x01)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_B_GPIO,SEG_B_PIN,(value & 0x02)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_C_GPIO,SEG_C_PIN,(value & 0x04)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_D_GPIO,SEG_D_PIN,(value & 0x08)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_E_GPIO,SEG_E_PIN,(value & 0x10)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_F_GPIO,SEG_F_PIN,(value & 0x20)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_G_GPIO,SEG_G_PIN,(value & 0x40)?GPIO_PIN_RESET:GPIO_PIN_SET);
}

void display_digit(uint8_t digit, uint8_t position){
    HAL_GPIO_WritePin(SEL1_GPIO,SEL1_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEL2_GPIO,SEL2_PIN,GPIO_PIN_SET);
    set_segments(digits[digit]);
    if(position==0) HAL_GPIO_WritePin(SEL1_GPIO,SEL1_PIN,GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(SEL2_GPIO,SEL2_PIN,GPIO_PIN_RESET);
}

void GenerateSecretCombination(uint8_t *numbers,uint8_t count,uint8_t max){
    uint32_t randNum;
    for(uint8_t i=0;i<count;i++){
        HAL_RNG_GenerateRandomNumber(&hrng,&randNum);
        numbers[i]=(randNum % max)+1;
        HAL_Delay(5); // Small delay between RNG calls
    }
}

uint8_t countExactMatches(uint8_t *user,uint8_t *secret){
    uint8_t exact=0;
    for(uint8_t i=0;i<TOTAL_DIGITS;i++)
        if(user[i]==secret[i]) exact++;
    return exact;
}

uint8_t countPartialMatches(uint8_t *user,uint8_t *secret){
    uint8_t partial=0;
    uint8_t usedSecret[TOTAL_DIGITS]={0};
    uint8_t usedUser[TOTAL_DIGITS]={0};
    for(uint8_t i=0;i<TOTAL_DIGITS;i++){
        if(user[i]==secret[i]){
            usedSecret[i]=1; usedUser[i]=1;
        }
    }
    for(uint8_t i=0;i<TOTAL_DIGITS;i++){
        if(usedUser[i]) continue;
        for(uint8_t j=0;j<TOTAL_DIGITS;j++){
            if(usedSecret[j]) continue;
            if(user[i]==secret[j]){ partial++; usedSecret[j]=1; break; }
        }
    }
    return partial;
}

void DisplayCurrentGuess(void){
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0,0);
    ssd1306_WriteString("Current guess:",Font_7x10,White);
    uint16_t x=0;
    for(uint8_t i=0;i<lockedDigits;i++){
        char buf[2]; sprintf(buf,"%d",currentGuess[i]);
        ssd1306_SetCursor(x,15); ssd1306_WriteString(buf,Font_7x10,White);
        x+=12;
    }
    char buf[2]; sprintf(buf,"%d",currentGuess[lockedDigits]);
    ssd1306_SetCursor(x,15); ssd1306_WriteString(buf,Font_7x10,White);

    char remaining[20]; sprintf(remaining,"Guesses left: %d",MAX_GUESSES-currentTry);
    ssd1306_SetCursor(0,40); ssd1306_WriteString(remaining,Font_7x10,White);
    ssd1306_UpdateScreen();
}

void DisplayFeedback(uint8_t exact, uint8_t partial) {
    uint8_t waitForButton = 1;

    // Create arrays to track which positions are used for exact and partial matches
    uint8_t exactMatches[TOTAL_DIGITS] = {0};
    uint8_t partialMatches[TOTAL_DIGITS] = {0};
    uint8_t secretUsed[TOTAL_DIGITS] = {0};

    // First pass: mark exact matches
    for (uint8_t i = 0; i < TOTAL_DIGITS; i++) {
        if (currentGuess[i] == secret[i]) {
            exactMatches[i] = 1;
            secretUsed[i] = 1; // Mark this position in secret as used
        }
    }

    // Second pass: mark partial matches (correct digit, wrong position)
    for (uint8_t i = 0; i < TOTAL_DIGITS; i++) {
        if (!exactMatches[i]) { // Only check positions that aren't exact matches
            for (uint8_t j = 0; j < TOTAL_DIGITS; j++) {
                if (!secretUsed[j] && currentGuess[i] == secret[j]) {
                    partialMatches[i] = 1;
                    secretUsed[j] = 1; // Mark this position in secret as used
                    break; // Only count each digit once
                }
            }
        }
    }

    while (waitForButton) {
        ssd1306_Fill(Black);

        // First row: User's guess
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Guess: ", Font_7x10, White);
        uint16_t x = 75;
        for (uint8_t i = 0; i < TOTAL_DIGITS; i++) {
            char buf[2];
            sprintf(buf, "%d", currentGuess[i]);
            ssd1306_SetCursor(x, 0);
            ssd1306_WriteString(buf, Font_7x10, White);
            x += 12;
        }

        // Second row: Position markers (X for exact, O for partial, _ for wrong)
        ssd1306_SetCursor(0, 15);
        ssd1306_WriteString("Correct: ", Font_7x10, White);
        x = 75;
        for (uint8_t i = 0; i < TOTAL_DIGITS; i++) {
            if (exactMatches[i]) {
                ssd1306_SetCursor(x, 15);
                ssd1306_WriteString("X", Font_7x10, White);
            } else if (partialMatches[i]) {
                ssd1306_SetCursor(x, 15);
                ssd1306_WriteString("O", Font_7x10, White);
            } else {
                ssd1306_SetCursor(x, 15);
                ssd1306_WriteString("-", Font_7x10, White);
            }
            x += 12;
        }

        // Third row: Summary
        char summary[30];
        sprintf(summary, "Exact: %d", exact);
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString(summary, Font_7x10, White);

        sprintf(summary, "Partial: %d", partial);
        ssd1306_SetCursor(0, 42);
        ssd1306_WriteString(summary, Font_7x10, White);

        // Fourth row: Instructions
        ssd1306_SetCursor(0, 54);
        ssd1306_WriteString("Press SW2 to continue", Font_6x8, White);

        ssd1306_UpdateScreen();

        // Simple button check - wait for SW2 press and release
        if (HAL_GPIO_ReadPin(GPIOC, SW2_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(100); // Debounce
            if (HAL_GPIO_ReadPin(GPIOC, SW2_Pin) == GPIO_PIN_RESET) {
                // Wait for button release
                while (HAL_GPIO_ReadPin(GPIOC, SW2_Pin) == GPIO_PIN_RESET) {
                    HAL_Delay(10);
                }
                waitForButton = 0; // Exit the loop
            }
        }

        HAL_Delay(50);
    }
}

void showIntro(void){
    char title[]="SKOCKO";
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20,20);
    ssd1306_WriteString(title,Font_16x26,White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);
}

/* USER CODE END PFP */

int main(void){
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();
    MX_GPIO_Init();
    MX_RNG_Init();
    MX_I2C2_Init();
    MX_ADC1_Init();
    ssd1306_Init();

    HAL_GPIO_WritePin(SEL1_GPIO,SEL1_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEL2_GPIO,SEL2_PIN,GPIO_PIN_SET);

    showIntro();

    GenerateSecretCombination(secret,TOTAL_DIGITS,MAX_DIGIT);
    memset(currentGuess,0,sizeof(currentGuess));
    lockedDigits=0;
    currentTry=0;

    uint32_t lastSW1Time = 0;
    uint32_t lastSW2Time = 0;
    uint8_t currentDigit = 1;

    while (1) {
        // Read potentiometer for current digit selection
        currentDigit = readPotentiometer();
        if (currentDigit > MAX_DIGIT) currentDigit = MAX_DIGIT;
        if (currentDigit < 1) currentDigit = 1;

        // Show live digit on 7-seg continuously
        display_digit(currentDigit, 1);

        // Update OLED display
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Current guess:", Font_7x10, White);

        uint16_t x = 0;
        // Show locked digits
        for (uint8_t i = 0; i < lockedDigits; i++) {
            char buf[2];
            sprintf(buf, "%d", currentGuess[i]);
            ssd1306_SetCursor(x, 15);
            ssd1306_WriteString(buf, Font_7x10, White);
            x += 12;
        }

        // Show current digit being selected (if not all digits locked)
        if (lockedDigits < TOTAL_DIGITS) {
            char buf[2];
            sprintf(buf, "%d", currentDigit);
            ssd1306_SetCursor(x, 15);
            ssd1306_WriteString(buf, Font_7x10, White);
        }

        char remaining[20];
        sprintf(remaining, "Guesses left: %d", MAX_GUESSES - currentTry);
        ssd1306_SetCursor(0, 40);
        ssd1306_WriteString(remaining, Font_7x10, White);

        // Show instructions
        if (lockedDigits < TOTAL_DIGITS) {
            ssd1306_SetCursor(0, 55);
            ssd1306_WriteString("SW1:Pick  SW2:Guess", Font_6x8, White);
        } else {
            ssd1306_SetCursor(0, 55);
            ssd1306_WriteString("Press SW2 to submit", Font_6x8, White);
        }

        ssd1306_UpdateScreen();

        // Button handling
        uint32_t currentTime = HAL_GetTick();

        // SW1: Lock current digit (only if we have digits left to lock)
        if (lockedDigits < TOTAL_DIGITS && HAL_GPIO_ReadPin(GPIOC, SW1_Pin) == GPIO_PIN_RESET) {
            if (currentTime - lastSW1Time > 100) { // Simple debounce
                currentGuess[lockedDigits] = currentDigit;
                lockedDigits++;
                lastSW1Time = currentTime;
                HAL_Delay(200); // Brief feedback delay
            }
        }

        // SW2: Submit guess (only when all digits are locked)
        if (lockedDigits == TOTAL_DIGITS && HAL_GPIO_ReadPin(GPIOC, SW2_Pin) == GPIO_PIN_RESET) {
            if (currentTime - lastSW2Time > 300) { // Simple debounce
                currentTry++;

                uint8_t exact = countExactMatches(currentGuess, secret);
                uint8_t partial = countPartialMatches(currentGuess, secret);

                // Show feedback and wait for SW2 press to continue
                DisplayFeedback(exact, partial);

                // Check win/lose conditions
                if (exact == TOTAL_DIGITS) {
                    ssd1306_Fill(Black);
                    ssd1306_SetCursor(10, 20);
                    ssd1306_WriteString("You WIN!", Font_16x26, White);
                    ssd1306_UpdateScreen();
                    HAL_Delay(3000);

                    // Reset game
                    GenerateSecretCombination(secret, TOTAL_DIGITS, MAX_DIGIT);
                    memset(currentGuess, 0, sizeof(currentGuess));
                    lockedDigits = 0;
                    currentTry = 0;
                }
                else if (currentTry >= MAX_GUESSES) {
                    ssd1306_Fill(Black);
                    ssd1306_SetCursor(0, 10);
                    ssd1306_WriteString("Out of tries!", Font_7x10, White);
                    ssd1306_SetCursor(0, 25);
                    char buf[20];
                    sprintf(buf, "Secret: %d%d%d%d", secret[0], secret[1], secret[2], secret[3]);
                    ssd1306_WriteString(buf, Font_7x10, White);
                    ssd1306_UpdateScreen();
                    HAL_Delay(4000);

                    // Reset game
                    GenerateSecretCombination(secret, TOTAL_DIGITS, MAX_DIGIT);
                    memset(currentGuess, 0, sizeof(currentGuess));
                    lockedDigits = 0;
                    currentTry = 0;
                }
                else {
                    // Reset for next guess
                    memset(currentGuess, 0, sizeof(currentGuess));
                    lockedDigits = 0;
                }

                lastSW2Time = currentTime;
                HAL_Delay(200); // Brief feedback delay
            }
        }

        HAL_Delay(50);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin|SEG_G_Pin|SEG_F_Pin|SEG_E_Pin
                          |SEL_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_2_GPIO_Port, SEL_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin|SEG_B_Pin|SEG_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_D_Pin SEG_G_Pin SEG_F_Pin SEG_E_Pin
                           SEL_1_Pin */
  GPIO_InitStruct.Pin = SEG_D_Pin|SEG_G_Pin|SEG_F_Pin|SEG_E_Pin
                          |SEL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_2_Pin */
  GPIO_InitStruct.Pin = SEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEL_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_C_Pin SEG_B_Pin SEG_A_Pin */
  GPIO_InitStruct.Pin = SEG_C_Pin|SEG_B_Pin|SEG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
