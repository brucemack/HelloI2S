/* USER CODE BEGIN Header */
/**
 * A demo of the STM32 with an AK4556VT Codec.
 * LOUT is synthesized and fed back to LIN.  LIN is filtered and
 * output to ROUT in software.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdio.h>

#define ARM_MATH_CM4
#include "arm_math.h"

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
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define FILTER_TAP_NUM 63
#define FILTER_BLOCK_SIZE 16

static const int blockSize = FILTER_BLOCK_SIZE;

// This where the DMA happens.  We multiply by 8 to address:
// - The fact that we have left and right data moving through at the same time
// - Each sample takes up two 16-bit words
// - There is space for two blocks.  The DMA works on one block while
//   the other block is being processed, and then we flip to the other side.
//
static uint16_t out_data[FILTER_BLOCK_SIZE * 8];
static uint16_t in_data[FILTER_BLOCK_SIZE * 8];

// A circular data transfer queue loaded from the input DMA interrupt and read from
// the output DMA interrupt.  This queue contains signed and properly scaled values.
static const int queueSize = FILTER_BLOCK_SIZE * 2 + 1;
static int32_t queue[FILTER_BLOCK_SIZE * 2 + 1];
// Pointers for the queue
static int queue_write_ptr = 0;
static int queue_read_ptr = 0;

// Synthesizer
static float amp = 0x7fffff;
static const float fs = 46875;
static const int lut_size = 256;
static float lut[256];
// The number of steps through the synthesis LUT on each cycle
static unsigned int lutStep = 0;
// Current pointer in the LUT
static unsigned int lutPtr = 0;

// Hilbert 90 degrees
static float32_t filter_taps[FILTER_TAP_NUM] = {
		 0.001070961715267396,
		 0.000000000000000000,
		 0.001987297681546781,
		 0.000000000000000000,
		 0.003277140348238743,
		 0.000000000000000000,
		 0.005026608812473088,
		 0.000000000000000000,
		 0.007336472989491107,
		 0.000000000000000000,
		 0.010328747622538574,
		 0.000000000000000000,
		 0.014158643000578517,
		 0.000000000000000000,
		 0.019036297989385199,
		 0.000000000000000000,
		 0.025267333294581040,
		 0.000000000000000000,
		 0.033332265196353454,
		 0.000000000000000000,
		 0.044053841571440711,
		 0.000000000000000000,
		 0.058988457956177756,
		 0.000000000000000000,
		 0.081488921472100143,
		 0.000000000000000000,
		 0.120317023900123110,
		 0.000000000000000000,
		 0.207702895007325888,
		 0.000000000000000000,
		 0.634100255771075982,
		 0.000000000000000000,
		-0.634100255771075982,
		 0.000000000000000000,
		-0.207702895007325888,
		 0.000000000000000000,
		-0.120317023900123110,
		 0.000000000000000000,
		-0.081488921472100143,
		 0.000000000000000000,
		-0.058988457956177756,
		 0.000000000000000000,
		-0.044053841571440711,
		 0.000000000000000000,
		-0.033332265196353454,
		 0.000000000000000000,
		-0.025267333294581040,
		 0.000000000000000000,
		-0.019036297989385199,
		 0.000000000000000000,
		-0.014158643000578517,
		 0.000000000000000000,
		-0.010328747622538574,
		 0.000000000000000000,
		-0.007336472989491107,
		 0.000000000000000000,
		-0.005026608812473088,
		 0.000000000000000000,
		-0.003277140348238743,
		 0.000000000000000000,
		-0.001987297681546781,
		 0.000000000000000000,
		-0.001070961715267396
};

/*
static float32_t filter_taps[FILTER_TAP_NUM] = {
  		 0.0007693591723595714,
		 0.0008577056171934674,
		 0.000981644864435715,
		 0.0011402836666741497,
		 0.0013240716958050664,
		 0.0015141785640862,
		 0.0016824951784660432,
		 0.0017922988545831472,
		 0.001799581146358226,
		 0.0016549955234453964,
		 0.0013063413991485225,
		 0.0007014641068754816,
		 -0.00020858039764729672,
		 -0.0014662700307441203,
		 -0.003104457186467815,
		 -0.005143494624281382,
		 -0.007588826928590531,
		 -0.010429208075408961,
		 -0.01363567593417401,
		 -0.01716137640597337,
		 -0.020942285414827483,
		 -0.02489882859951619,
		 -0.028938349041523544,
		 -0.03295832555942324,
		 -0.03685020078936992,
		 -0.0405036420149939,
		 -0.04381103068466763,
		 -0.046671960421076554,
		 -0.04899751913018745,
		 -0.05071413891252709,
		 -0.05176681751743772,
		 0.9486121374855334,
		 -0.05176681751743772,
		 -0.05071413891252709,
		 -0.04899751913018745,
		 -0.046671960421076554,
		 -0.04381103068466763,
		 -0.0405036420149939,
		 -0.03685020078936992,
		 -0.03295832555942324,
		 -0.028938349041523537,
		 -0.02489882859951619,
		 -0.020942285414827476,
		 -0.017161376405973375,
		 -0.013635675934174009,
		 -0.010429208075408961,
		 -0.007588826928590529,
		 -0.005143494624281382,
		 -0.003104457186467814,
		 -0.0014662700307441208,
		 -0.00020858039764729667,
		 0.0007014641068754816,
		 0.0013063413991485225,
		 0.0016549955234453975,
		 0.001799581146358226,
		 0.0017922988545831487,
		 0.0016824951784660432,
		 0.0015141785640862006,
		 0.0013240716958050664,
		 0.0011402836666741503,
		 0.000981644864435715,
		 0.0008577056171934679,
		 0.0007693591723595714
};
*/
static float32_t filter_state[FILTER_TAP_NUM + FILTER_BLOCK_SIZE - 1];
static arm_fir_instance_f32 filter_lp_0;

static const uint16_t fft_size = 256;
static arm_rfft_fast_instance_f32 rfft;
static float32_t fft_input[256];
static int fft_input_ptr = 0;
//static uint32_t last_time = 0;

// Delay line that is equivalent in length to the filter
static float32_t delayBuffer[FILTER_TAP_NUM];
static int delayPtr = 0;

static int fix(int ptr) {
	return ptr % FILTER_TAP_NUM;
}

static void writeDelay(const float32_t* d, int size) {
	for (int i = 0; i < size; i++) {
		delayBuffer[delayPtr] = d[i];
		delayPtr = fix(delayPtr + 1);
	}
}

static void readDelay(float32_t* d, int size) {
	for (int i = 0; i < size; i++) {
		d[i] = delayBuffer[fix(delayPtr + i)];
	}
}

/*
// For debugging/timing
static uint32_t cycles(void)
{
	return DWT->CYCCNT;
}
*/
static void setSynthFreq(float freqHz) {
	float phasePerSample = (freqHz / fs) * 2.0 * 3.14159;
	lutStep = (phasePerSample / (2.0 * 3.1415926)) * lut_size;
}

// This writes half of the outbound data into the DMA buffer.  This is
// called by the DMA interrupt at the half-way point through the
// entire DMA buffer.
static void moveOut(int base) {

	int32_t ci;
	uint16_t hi = 0;
	uint16_t lo = 0;

	int out_ptr = base;
	float32_t filterIn[FILTER_BLOCK_SIZE];
	float32_t filterOut[FILTER_BLOCK_SIZE];
	float32_t delayOut[FILTER_BLOCK_SIZE];

	// Apply the filter to the block of data that we have read most recently
	for (int i = 0; i < blockSize; i++) {
		// Move the sampled data into the arrays that will be processed by
		// the DSP functions:
		filterIn[i] = queue[queue_read_ptr++];
		// Look for the wrap
		if (queue_read_ptr == queueSize) {
			queue_read_ptr = 0;
		}
	}
	// MEASUREMENT: This takes about 4,000 cycles.
	arm_fir_f32(&filter_lp_0, filterIn, filterOut, blockSize);

	// Move data out of the circular buffer (lagged)
	readDelay(delayOut, blockSize);
	// Move data into the circular buffer
	writeDelay(filterIn, blockSize);

	// Generate the output signals
	for (int i = 0; i < blockSize; i++) {

		// LEFT CHANNEL
		// Generate synth data by stepping through the cosine table
		// This handles the wrapping of the LUT pointer:
		lutPtr = (lutPtr + lutStep) & 0xff;
		ci = amp * lut[lutPtr];
		hi = ci >> 8;
		lo = (ci & 0x000000ff) << 8;
		// Synth data on left channel
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;

		// RIGHT CHANNEL
		// Output filtered data (signed)
		//ci = filterOut[i];
		//ci = delayOut[i];
		ci = amp * lut[lutPtr];
		hi = ci >> 8;
		lo = (ci & 0x000000ff) << 8;
		out_data[out_ptr++] = hi;
		out_data[out_ptr++] = lo;
	}
}

// This read half of the inbound data from the DMA buffer.  This is
// called by the DMA interrupt at the half-way point through the
// entire DMA buffer.
static void moveIn(int base) {

	int in_ptr = base;

	for (int i = 0; i < blockSize; i++) {
		// Load the high end of the sample into the high end of the 32-bit number
		int32_t sample = (in_data[in_ptr++] << 16);
		// Preserve sign while adjusting magnitude down from 32 bits to 24 bits
		sample = sample >> 8;
		// Shift down the low end of the sample and mask off
		sample |= (in_data[in_ptr++] >> 8) & 0xff;
		// Queue the sample for processing
		queue[queue_write_ptr++] = sample;
		// Look for the wrap
		if (queue_write_ptr == queueSize) {
			queue_write_ptr = 0;
		}
		// FFT sample buffer
		fft_input[fft_input_ptr++] = sample;
		if (fft_input_ptr == fft_size) {
			fft_input_ptr = 0;
		}
		// Ignore the right channel input (for now)
		in_ptr++;
		in_ptr++;
	}
}

// Called at the half-way point.  Fills in the first half of the DMA area.
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveOut(0);
}

// Called at the end point.  Fills in the second half of the DMA area.
void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveOut(blockSize * 4);
}

// Called at the half-way point.  Fills in the first half of the DMA area.
void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveIn(0);
}

// Called at the end point.  Fills in the second half of the DMA area.
void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef * hi2s) {
	moveIn(blockSize * 4);
}

int _write(int file, char* ptr, int len) {
	for (int i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int count = 0;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  printf("Hello World\n");

  // Enable cycle counting
  //CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  //DWT->CYCCNT = 0;
  //DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Initialize the filter
  arm_fir_init_f32(&filter_lp_0, FILTER_TAP_NUM, filter_taps, filter_state, FILTER_BLOCK_SIZE);
  // Initialize FFT
  arm_rfft_fast_init_f32(&rfft, fft_size);

  // Initialize DMA
  HAL_I2S_Transmit_DMA(&hi2s2, out_data, blockSize * 4);
  HAL_I2S_Receive_DMA(&hi2s3, in_data, blockSize * 4);

	// Populate synthesizer LUT
	for (int i = 0; i < lut_size; i++) {
		float t = (float)i / (float)lut_size;
		float a = t * 2.0 * 3.1315926;
		lut[i] = cos(a);
	}

	setSynthFreq(1500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	  //HAL_Delay(100);
	  //HAL_I2S_Transmit(&hi2s2, data, 2, 100);

	  // Alternate between frequencies
	  HAL_Delay(150);
	  setSynthFreq(400);
	  HAL_Delay(150);
	  setSynthFreq(4000);
	  count++;
	  /*
		float32_t fft_max = 0;
		int fft_max_ptr = 0;
		float32_t fft_out[256];
		// MEASUREMENT: This takes about 26,000 cycles
		arm_rfft_fast_f32(&rfft, fft_input, fft_out, 0);
		// Look for the largest bin
		for (int j = 0; j < 256; j++) {
			if (fabsf(fft_out[j]) > fft_max) {
				fft_max = fabsf(fft_out[j]);
				fft_max_ptr = j;
			}
		}
		printf("FFT %d mag %f idx %d el %lu\n", count, fft_max, fft_max_ptr, last_time);
		*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
