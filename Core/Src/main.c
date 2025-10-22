/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CMD_NONE,
    CMD_START_LED,
    CMD_STOP_LED,
} Command_t;

typedef enum {
    ANALYZER_OFF,
    ANALYZER_AVG,
    ANALYZER_MAX,
    ANALYZER_MIN
} AnalyzerMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_BUFFER_SIZE     20
#define UART_RESPONSE_SIZE   512
#define MAX_SAMPLES          100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADCSampler */
osThreadId_t ADCSamplerHandle;
const osThreadAttr_t ADCSampler_attributes = {
  .name = "ADCSampler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DataAnalyzer */
osThreadId_t DataAnalyzerHandle;
const osThreadAttr_t DataAnalyzer_attributes = {
  .name = "DataAnalyzer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AlertMonitor */
osThreadId_t AlertMonitorHandle;
const osThreadAttr_t AlertMonitor_attributes = {
  .name = "AlertMonitor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledCmd */
osMessageQueueId_t ledCmdHandle;
const osMessageQueueAttr_t ledCmd_attributes = {
  .name = "ledCmd"
};
/* Definitions for adcAnalyzer */
osMessageQueueId_t adcAnalyzerHandle;
const osMessageQueueAttr_t adcAnalyzer_attributes = {
  .name = "adcAnalyzer"
};
/* Definitions for adcAlert */
osMessageQueueId_t adcAlertHandle;
const osMessageQueueAttr_t adcAlert_attributes = {
  .name = "adcAlert"
};
/* Definitions for uartTx */
osMessageQueueId_t uartTxHandle;
const osMessageQueueAttr_t uartTx_attributes = {
  .name = "uartTx"
};
/* USER CODE BEGIN PV */
volatile uint16_t adc_sample_period = 1000;
volatile uint16_t alert_threshold = 1500;
volatile AnalyzerMode_t analyzer_mode = ANALYZER_OFF;
volatile uint16_t sample_count = 10;
volatile uint8_t alert_active = 0;
uint8_t rx_char;

volatile uint8_t g_rx_buffer[UART_BUFFER_SIZE];
volatile uint16_t g_rx_idx = 0;

const char welcome_msg[] =
	      "\r\n"
	      "***************************************\r\n"
	      " Witaj w programie przygotowanym na \r\n"
	      " projekt z FreeRTOS\r\n"
	      "***************************************\r\n"
	      "Uwaga: Mozna usuwac znaki w terminalu\r\n"
	      "Dostepne polecenia:\r\n"
	      "1. LED: START LED, STOP LED\r\n"
	      "2. ADC: SET PERIOD <ms>, GET PERIOD, GET ADC\r\n"
		  "3. Alert: SET ALERT <value>, GET ALERT\r\n"
	      "4. Analyzer: START ANALYSIS, STOP ANALYSIS, SET ANALYSIS AVG/MAX/MIN, GET ANALYSIS, SET SAMPLES <value>\r\n"
		  "5. HELP\r\n"
	      "6. EXIT\r\n"
	      "***************************************\r\n"
	      "> ";

const char help[] =
	      "\r\n"
	      "***************************************\r\n"
	      " Witaj w programie przygotowanym na \r\n"
	      " projekt z FreeRTOS\r\n"
	      "***************************************\r\n"
	      "Uwaga: Mozna usuwac znaki w terminalu\r\n"
	      "Dostepne polecenia:\r\n"
	      "1. LED: START LED, STOP LED\r\n"
	      "2. ADC: SET PERIOD <ms>, GET PERIOD, GET ADC\r\n"
		  "3. Alert: SET ALERT <value>, GET ALERT\r\n"
	      "4. Analyzer: START ANALYSIS, STOP ANALYSIS, SET ANALYSIS AVG/MAX/MIN, GET ANALYSIS, SET SAMPLES <value>\r\n"
		  "5. HELP\r\n"
	      "6. EXIT\r\n"
	      "***************************************";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartUartTask(void *argument);
void StartLedTask(void *argument);
void StartADCSampler(void *argument);
void StartDataAnalyzer(void *argument);
void StartAlertMonitor(void *argument);

/* USER CODE BEGIN PFP */

char* CommandHandler_Parse(char* command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

// Odbiór pojedynczego znaku przez przerwanie
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

   if(huart->Instance == USART2){
	osMessageQueuePut(uartTxHandle, &rx_char, 0, 0);
	HAL_UART_Receive_IT(&huart2, &rx_char, 1);

   }
}

void uart_print_with_prompt(const char* msg)
{
    // ANSI: \033[2K czyści całą linię, \r ustawia na początek linii
    const char clear_line[] = "\033[2K\r";
    HAL_UART_Transmit(&huart2, (uint8_t*)clear_line, strlen(clear_line), HAL_MAX_DELAY);

    // Wypisz wynik analizy (np. "AVG: 1234\r\n")
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Nowa linia prompta i ponownie dotychczas wpisany tekst
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 5, HAL_MAX_DELAY);
    if(g_rx_idx > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)g_rx_buffer, g_rx_idx, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ledCmd */
  ledCmdHandle = osMessageQueueNew (4, sizeof(Command_t), &ledCmd_attributes);

  /* creation of adcAnalyzer */
  adcAnalyzerHandle = osMessageQueueNew (16, sizeof(uint16_t), &adcAnalyzer_attributes);

  /* creation of adcAlert */
  adcAlertHandle = osMessageQueueNew (16, sizeof(uint16_t), &adcAlert_attributes);

  /* creation of uartTx */
  uartTxHandle = osMessageQueueNew (16, sizeof(uint8_t), &uartTx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of ADCSampler */
  ADCSamplerHandle = osThreadNew(StartADCSampler, NULL, &ADCSampler_attributes);

  /* creation of DataAnalyzer */
  DataAnalyzerHandle = osThreadNew(StartDataAnalyzer, NULL, &DataAnalyzer_attributes);

  /* creation of AlertMonitor */
  AlertMonitorHandle = osThreadNew(StartAlertMonitor, NULL, &AlertMonitor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

char* CommandHandler_Parse(char* command) {
    static char response[UART_RESPONSE_SIZE];

    char* token1 = strtok(command," ");
    char* token2 = strtok(NULL," ");
    char* token3 = strtok(NULL," ");

    if (token1 == NULL) {
        snprintf(response, sizeof(response), "ERROR: Empty command");
        return response;
    }

    /* LED Commands */
    if (strcmp(token1, "START") == 0 && token2 && strcmp(token2,"LED") == 0){
            Command_t cmd = CMD_START_LED;
            osMessageQueuePut(ledCmdHandle, &cmd, 0, 0);
            snprintf(response, sizeof(response), "LED blinking started");
        }else if(strcmp(token1,"STOP") == 0 && token2 && strcmp(token2, "LED") == 0){
            Command_t cmd = CMD_STOP_LED;
            osMessageQueuePut(ledCmdHandle, &cmd, 0, 0);
            snprintf(response,sizeof(response), "LED stopped");
        }
    /* ADC Commands */
    else if (strcmp(token1, "GET") == 0 && token2 && strcmp(token2,"ADC") == 0){
    	HAL_ADC_Start(&hadc1);
    	if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
    		uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
    		snprintf(response,sizeof(response),"ADC: %u",adc_value);
    	}else{
    		snprintf(response,sizeof(response),"ADC Error\r\n");
    	}
    }
    else if(strcmp(token1, "GET") == 0 && token2 && strcmp(token2,"PERIOD") == 0){
        		snprintf(response, sizeof(response), "Current ADC period: %ums", adc_sample_period);
        	}
    else if (strcmp(token1, "SET") == 0 && token2 && strcmp(token2,"PERIOD") == 0 && token3){
       uint16_t new_period = atoi(token3);
       if(new_period >= 10 && new_period <= 5000){
    	   adc_sample_period = new_period;
    	   snprintf(response, sizeof(response),"ADC period set to %ums",new_period);
       }else{
    	   snprintf(response,sizeof(response),"ERROR: Period must be 10-5000ms");
       }
    }
    /* Alert Commands */
    else if(strcmp(token1,"SET") == 0 && token2 && strcmp(token2,"ALERT") == 0 && token3){
       alert_threshold = atoi(token3);
       snprintf(response,sizeof(response), "Alert threshold set to %u",alert_threshold);
    }else if(strcmp(token1,"GET") == 0 && token2 && strcmp(token2,"ALERT") == 0){
    	snprintf(response,sizeof(response), "Alert threshold set to %u",alert_threshold);
    }
    /* Analyzer Commands */
    else if (strcmp(token1,"START") == 0 && token2 && strcmp(token2,"ANALYSIS") == 0){
        analyzer_mode = ANALYZER_AVG;  // Default mode
        snprintf(response, sizeof(response), "Data analysis started (AVG mode)");
    }
    else if (strcmp(token1,"STOP") == 0 && token2 && strcmp(token2,"ANALYSIS") == 0){
        analyzer_mode = ANALYZER_OFF;
        snprintf(response, sizeof(response), "Data analysis stopped");
    }
    else if (strcmp(token1,"SET") == 0 && token2 && strcmp(token2,"ANALYSIS") == 0 && token3){
    	if(strcmp(token3,"AVG") == 0){
    		analyzer_mode = ANALYZER_AVG;
    		snprintf(response, sizeof(response), "Analysis mode set to AVG");
    	}else if(strcmp(token3,"MAX") == 0){
    		analyzer_mode = ANALYZER_MAX;
    		snprintf(response, sizeof(response), "Analysis mode set to MAX");
    	}else if(strcmp(token3,"MIN") == 0){
    		analyzer_mode = ANALYZER_MIN;
    		snprintf(response, sizeof(response), "Analysis mode set to MIN");
    	}else{
    		snprintf(response,sizeof(response), "ERROR: Usage: SET ANALYSIS <AVG|MAX|MIN>");
    	}
    }
    else if(strcmp(token1,"GET") == 0 && token2 && strcmp(token2,"ANALYSIS") == 0){
    	const char* mode_str = "UNKNOWN";
    	switch(analyzer_mode){
    		case ANALYZER_AVG: mode_str = "AVG"; break;
    		case ANALYZER_MAX: mode_str = "MAX"; break;
    		case ANALYZER_MIN: mode_str = "MIN"; break;
    		case ANALYZER_OFF: mode_str = "OFF"; break;
    	}
    	snprintf(response, sizeof(response), "Curremt analysis mode: %s",mode_str);
    }
    else if(strcmp(token1,"SET") == 0 && token2 && strcmp(token2, "SAMPLES") == 0 && token3){
    	uint16_t new_count = atoi(token3);
    	if(new_count > 0 && new_count <= MAX_SAMPLES){
    		sample_count = new_count;
    		snprintf(response, sizeof(response), "Sample count set to %u", new_count);
    	}else{
    		snprintf(response, sizeof(response), "ERROR: Samples must be 1-%u",MAX_SAMPLES);
    	}
    }else if(strcmp(token1,"EXIT") == 0){
    	snprintf(response, sizeof(response), "The End");
    	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    	analyzer_mode = ANALYZER_OFF;
    //	osThreadExit();
    }else if(strcmp(token1,"HELP") == 0){
    	snprintf(response, sizeof(response), "%s", help);
    }
    /* Unknown Command */
    else {
    	snprintf(response, sizeof(response), "ERROR: Unknown command or wrong syntax");
    }

    return response;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartTask */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t rx_char;
	extern volatile uint8_t g_rx_buffer[UART_BUFFER_SIZE];
	extern volatile uint16_t g_rx_idx;

	HAL_UART_Transmit(&huart2, (uint8_t*)welcome_msg, strlen(welcome_msg), HAL_MAX_DELAY);


	  /* Infinite loop */
	for(;;)
		{
			if (osMessageQueueGet(uartTxHandle, &rx_char, NULL, osWaitForever) == osOK) {
				if (rx_char == '\b' || rx_char == 127) { //Usuwanie znakow z terminala
					if (g_rx_idx > 0) {
						g_rx_idx--;
						HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, HAL_MAX_DELAY);
					}
					continue;
				}

				if (rx_char != '\r' && rx_char != '\n') {
					HAL_UART_Transmit(&huart2, &rx_char, 1, HAL_MAX_DELAY); //Echo znaku
				}

				if (rx_char == '\n' || rx_char == '\r') {
					 g_rx_buffer[g_rx_idx] = '\0';  // Null-terminate
						char* response = CommandHandler_Parse((char*)g_rx_buffer);
						HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
						HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
						HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 5, HAL_MAX_DELAY);
						g_rx_idx = 0;
				}
				else if (g_rx_idx < UART_BUFFER_SIZE - 1) {
					g_rx_buffer[g_rx_idx++] = rx_char;
				}
				else {
					HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nERROR: Command too long\r\n> ", 28, HAL_MAX_DELAY);
					g_rx_idx = 0;
				}
			}
			osDelay(1);
		}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedTask */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
	uint8_t led_state = 0;
	Command_t cmd;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(ledCmdHandle, &cmd, NULL, 0) == osOK){
	  		if(cmd == CMD_START_LED){
	  			led_state = 1;
	  		}else if(cmd == CMD_STOP_LED){
	  			led_state = 0;
	  		}
	  		}
	  		if (led_state) {
	  			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	  			osDelay(500);
	  			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	  			osDelay(500);
	  		} else {
	  			osDelay(10);
	  		}
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartADCSampler */
/**
* @brief Function implementing the ADCSampler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCSampler */
void StartADCSampler(void *argument)
{
  /* USER CODE BEGIN StartADCSampler */
	uint16_t adc_value = 0;
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
		adc_value = HAL_ADC_GetValue(&hadc1);
		osMessageQueuePut(adcAnalyzerHandle, &adc_value, 0, 0);
		osMessageQueuePut(adcAlertHandle, &adc_value, 0, 0);
	}
    osDelay(adc_sample_period);
  }
  /* USER CODE END StartADCSampler */
}

/* USER CODE BEGIN Header_StartDataAnalyzer */
/**
* @brief Function implementing the DataAnalyzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataAnalyzer */
void StartDataAnalyzer(void *argument)
{
  /* USER CODE BEGIN StartDataAnalyzer */
	uint16_t buffer[MAX_SAMPLES] = {0};
    uint16_t idx = 0;
	uint16_t current_sample_count = sample_count;
  /* Infinite loop */
  for(;;)
  {

		current_sample_count = sample_count; // aktualizacja na bieżąco
		if(osMessageQueueGet(adcAnalyzerHandle, &buffer[idx], NULL, osWaitForever) == osOK){
			if(analyzer_mode != ANALYZER_OFF){
				idx = (idx + 1)%current_sample_count;
				if(idx==0){
					uint32_t sum = 0;
					uint16_t max_val = 0;
					uint16_t min_val = 0xFFFF; //Dzieki temu kazda inna jest mniejsza
					for(int i = 0; i < current_sample_count; i++){
						sum += buffer[i];
						if(buffer[i] > max_val) max_val = buffer[i];
						if(buffer[i] < min_val) min_val = buffer[i];
					}
					char msg[64];
					switch(analyzer_mode){
						case ANALYZER_AVG:
							snprintf(msg, sizeof(msg), "AVG: %lu",sum/current_sample_count);
							break;
						case ANALYZER_MAX:
							snprintf(msg, sizeof(msg), "MAX: %u",max_val);
							break;
						case ANALYZER_MIN:
							snprintf(msg,sizeof(msg),"MIN: %u",min_val);
							break;
						default:
							break;
					}
					uart_print_with_prompt(msg);
				}
			}
		}
		osDelay(1);
  }
  /* USER CODE END StartDataAnalyzer */
}

/* USER CODE BEGIN Header_StartAlertMonitor */
/**
* @brief Function implementing the AlertMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlertMonitor */
void StartAlertMonitor(void *argument)
{
  /* USER CODE BEGIN StartAlertMonitor */
	uint16_t adc_value;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(adcAlertHandle, &adc_value, NULL, osWaitForever) == osOK){
	  			if(adc_value > alert_threshold){
	  				if(!alert_active){
	  					char alert_msg[64];
	  					snprintf(alert_msg, sizeof(alert_msg),"!!! ALERT: Value %u above limit !!!",adc_value);
	  					uart_print_with_prompt(alert_msg);
	  					alert_active = 1;
	  				}
	  			}else{
	  				alert_active = 0;
	  			}
	  		}
	  		osDelay(1);
  }
  /* USER CODE END StartAlertMonitor */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
