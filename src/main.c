/*
 * Seismic Intensity Meter for STM32F303
 */

#include <stdio.h>
#include <string.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#include "stm32f3xx_hal.h"

#include "MPU6050.h"

#define LED1_PIN                                GPIO_PIN_3
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

#define NUM_STAGES           6  // 2次フィルターの段数（バイカッド数）

// Biquadのインスタンスと状態変数
arm_biquad_casd_df1_inst_f32 SX, SY, SZ;
float32_t stateX[4 * NUM_STAGES]; // 状態変数バッファ (2次×2 = 4要素/ステージ)
float32_t stateY[4 * NUM_STAGES]; // 状態変数バッファ (2次×2 = 4要素/ステージ)
float32_t stateZ[4 * NUM_STAGES]; // 状態変数バッファ (2次×2 = 4要素/ステージ)

// フィルタ係数 {b0, b1, b2, a1, a2} （※a0は通常1.0で正規化、符号に注意が必要な場合あり）
float32_t coeffArray[5 * NUM_STAGES] = {
//    b0, b1, b2, a1, a2
    // Stage 1
     0.369796f,  0.000000f, -0.369796f,  0.260408f, -0.000000f,
    // Stage 2
     0.284523f,  0.000000f, -0.284523f,  0.430953f, -0.000000f,
    // Stage 3
     0.106209f,  0.212418f,  0.106209f,  1.109157f, -0.533993f,
    // Stage 4
     0.187803f,  0.375606f,  0.187803f,  0.648175f, -0.399387f,
    // Stage 5
     0.347514f,  0.695028f,  0.347514f,  0.000492f, -0.390548f,
    // Stage 6
     0.534293f,  1.068586f,  0.534293f, -0.638541f, -0.498631f
};

static float32_t sdata[100];
static float32_t adata[100];
int spos = 0;

void Error_Handler()
{
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // (8MHz / 2) * 16 = 64MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // SYSCLK = 64MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       // HCLK = 64MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;        // PCLK1 = 32MHz (MAX 36MHzを超えないよう分周)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;        // PCLK2 = 64MHz
  
  // 64MHz駆動のフラッシュレイテンシは「2」を指定
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// GPIO・クロックの下位レイヤー初期化 (HAL_UART_Initから自動で呼ばれます)
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance==USART1) {
        // クロック有効化
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10    ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // F303のUSART1はAF7
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(hi2c->Instance == I2C1)
    {
        /* 1. 周辺機能のクロックを有効化 */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* 2. PB6 (I2C1_SCL) と PB7 (I2C1_SDA) のGPIO設定 */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;            // オルタネイト機能・オープンドレイン
        GPIO_InitStruct.Pull = GPIO_PULLUP;               // 内蔵プルアップ（外付けがある場合も必須または推奨）
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;       // 高速通信に対応
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;         // STM32F303のI2C1用オルタネイト機能 (AF4)
        
        // GPIOの初期化を実行
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void STM32_Biquad_Init(void) {
    // CMSIS-DSPの初期化関数
    arm_biquad_cascade_df1_init_f32(&SX, NUM_STAGES, coeffArray, stateX);
    arm_biquad_cascade_df1_init_f32(&SY, NUM_STAGES, coeffArray, stateY);
    arm_biquad_cascade_df1_init_f32(&SZ, NUM_STAGES, coeffArray, stateZ);
}

float32_t STM32_Biquad_Process(arm_biquad_casd_df1_inst_f32 S, float32_t input_val) {
    float32_t output_val = 0.0f;
    arm_biquad_cascade_df1_f32(&S, &input_val, &output_val, 1);
    return output_val;
}

float32_t calculate_vector_sum(float32_t a_ns, float32_t a_ew, float32_t a_ud) {
    float32_t sum_squares = (a_ns * a_ns) + (a_ew * a_ew) + (a_ud * a_ud);
    float32_t a_vector;
    
    // ARM CMSIS-DSPの高速平方根関数を使用
    arm_status status = arm_sqrt_f32(sum_squares, &a_vector);
    
    if (status == ARM_MATH_SUCCESS) {
        return a_vector; // 合成加速度(gal)
    } else {
        return 0.0f;     // エラー（入力が負数の場合など）
    }
}

int compare_float(const void *a, const void *b) {
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

int main(void)
{
  uint8_t txBuff[128];
  float32_t max;

  HAL_Init();

  SystemClock_Config();

  LED1_GPIO_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = LED1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct); 

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100204;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  MPU6050_Initialization();
  STM32_Biquad_Init();

  max = -10.0f;
  while (1)
  {
     uint8_t i;
     HAL_StatusTypeDef result;
     float32_t X, Y, Z;
     float32_t output;
     float32_t a, zout, shindo;

#if 0
    for(i = 1; i < 128; i++) {
        // STM32のHALではアドレスを左シフト（i << 1）して指定する
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 1, 1);
        if (result == HAL_OK) {
            // デバイスが見つかった場合の処理（UARTやデバッガで出力）
            // 例: printf または HAL_UART_Transmit 等でアドレスを送信

            sprintf((char *)txBuff, "%d,", i);
            HAL_UART_Transmit(&huart1, txBuff, strlen((char *)txBuff), 0xFFFF);
        }
    }
#endif
    if(MPU6050_DataReady() == 1)
    {
        MPU6050_ProcessData(&MPU6050);

        X = STM32_Biquad_Process(SX, MPU6050.acc_x);
        Y = STM32_Biquad_Process(SY, MPU6050.acc_y);
        Z = STM32_Biquad_Process(SZ, MPU6050.acc_z);

	output = calculate_vector_sum(X, Y, Z);
        arm_abs_f32(&output, &zout, 1);
	sdata[spos] = zout;
        ++spos;
        if(spos == sizeof(sdata) / 4) {
            spos = 0;
           HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
        }
	memcpy(adata, sdata, sizeof(sdata));
	qsort(adata, sizeof(sdata) / sizeof(sdata[0]), sizeof(float), compare_float);

	shindo = log10(adata[30]) * 2.0 + 0.94f;
	if (shindo > max)
 		max = shindo;
	if (spos == 0) {
          if (max > 1.0f) {
            sprintf(txBuff,"%f\n", max);
            HAL_UART_Transmit(&huart1, txBuff, strlen((char *)txBuff), 0xFFFF);
          }
  	  max = -10.0f;
        }
    }
  }
}
