/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t linha;  //Cordenada da base da nave
  uint8_t coluna; //Cordenada do centro da nave
} nave;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer[8][8] = {0};//Buffer que contém o conjunto de pixels a ser desenhado em cada frame
nave nave_heroi = {7,1};
uint8_t laser_cont = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void desenha_pixel(uint8_t x, uint8_t y);
void desenha_frame();
void adiciona_nave_no_buffer();
void adiciona_laser_no_buffer(uint8_t laser);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Função: adiciona_laser_no_buffer
 * Descrição: Inclui, no buffer, 1 pixel representando o laser da nave. A distância entre o laser e a nave e dada pela entrada "laser"
 * Parâmetros:
 *  - laser: comprimento do laser, em um dado momento no tempo.
 */
void adiciona_laser_no_buffer(uint8_t laser)
{
  if(nave_heroi.linha-1-laser <=5)
    buffer[nave_heroi.linha-1-laser][nave_heroi.coluna] = 1;
}

/*
 * Função: adiciona_nave_no_buffer
 * Descrição: Adiciona o desenho de uma nave, no buffer. O ponto nave.linha, nave.coluna é o centro inferior da nave.
 */
void adiciona_nave_no_buffer()
{
  if(nave_heroi.coluna > 6) //Passou do fim da tela (esquerda)
    nave_heroi.coluna = 6;
  if(nave_heroi.linha > 7) //Passou do fim da tela (baixo)
    nave_heroi.linha = 7;
  if(nave_heroi.coluna < 1) //Passou do fim da tela (esquerda)
    nave_heroi.coluna = 1;
  if(nave_heroi.linha < 1) //Passou do fim da tela (baixo)
    nave_heroi.linha = 1;
  buffer[nave_heroi.linha]  [nave_heroi.coluna]   = 1;
  buffer[nave_heroi.linha]  [nave_heroi.coluna-1] = 1;
  buffer[nave_heroi.linha]  [nave_heroi.coluna+1] = 1;
  buffer[nave_heroi.linha-1][nave_heroi.coluna]   = 1;
}

/*
 * Função: limpar_buffer
 * Descrição: Limpa todos os elementos do buffer
 */
void limpar_buffer()
{
  for(int i=0; i<8; i++)
    for(int j=0; j<8; j++)
      buffer[i][j] = 0;
}

/*
 * Função: desenha_frame
 * Descrição: Detecta quais pixels do buffer são 1 e acende cada um deles, em sequência, por 1ms
 */
void desenha_frame()
{
  for(int i=0; i<8; i++)
    for(int j=0; j<8; j++)
      if(buffer[i][j] != 0)
      {
        desenha_pixel(j,i);
        HAL_Delay(1);
      }
}

/*
 * Função: desenha_pixel
 * Descrição: Acende um pixel específico e manter todos os outros apagados. O ponto (0,0) é o canto superior esquerdo da matriz
 * Parâmetros:
 *  - x: coordenada do eixo x (0-7)
 *  - y: coordenada do eixo y (0-7)
 */
void desenha_pixel(uint8_t x, uint8_t y)
{
  //Zerar todos os pixels das linhas e colunas da matriz (C1-C8 e R1-R8). Polarizar inversamente todos os LEDs
  GPIOB->ODR &= 0b11111111111111110011111111111111; //resetando os pinos C8,C7
  GPIOC->ODR &= 0b11111111111111111100000011111111; //resetando os pinos C6,C5,C4,C3,C2,C1
  GPIOC->ODR |= 0b00000000000000000000000011111111; //setando os pinos R8,R7,R6,R5,R4,R3,R2,R1

  //Polarizar diretamente só o LED x,y
  if(x<=5)
    GPIOC->ODR |= 1<<(8+x);
  else
    GPIOB->ODR |= 1<<(8+x);

  GPIOC->ODR &= ~(1<<y);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Teste se algum botão foi pressionado
    if(HAL_GPIO_ReadPin(botao_direita_GPIO_Port, botao_direita_Pin)==0)
      nave_heroi.coluna++; //Desloca a nave para direita
    if(HAL_GPIO_ReadPin(botao_esquerda_GPIO_Port, botao_esquerda_Pin)==0)
      nave_heroi.coluna--; //Desloca a nave para esquerda
    if(HAL_GPIO_ReadPin(botao_acao_GPIO_Port, botao_acao_Pin)==0)
      laser_cont=1; //Dispara o laser

    limpar_buffer();
    adiciona_nave_no_buffer();
    if((laser_cont > 0)&&(laser_cont<=5))
      laser_cont++; //Avança o laser
    else
      laser_cont = 0; //Desliga o laser
    adiciona_laser_no_buffer(laser_cont);

    //Desenha o frame 10 vezes para dar um efeito de DELAY (Péssima prática. Melhoraremos na aula de TIMER)
    for(uint32_t j=0; j<10; j++)
    {
      desenha_frame();
    }


    //TESTES PRELIMINARES

    /*for(uint8_t i=0; i<8; i++)
      for(uint8_t j=0; j<8; j++)
      {
        desenha_pixel(i, j);
        HAL_Delay(10);
      }*/

    /*desenha_pixel(0, 0);
    HAL_Delay(500);
    desenha_pixel(7, 0);
    HAL_Delay(500);
    desenha_pixel(0, 7);
    HAL_Delay(500);
    desenha_pixel(7, 7);
    HAL_Delay(500);*/

    /*GPIOB->ODR &= 0b11111111111111110011111111111111;
    GPIOC->ODR  = 0b00000000000000000000000100000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000000001000000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000000010000000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000000100000000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000001000000000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000010000000000000;
    HAL_Delay(500);
    GPIOC->ODR  = 0b00000000000000000000000000000000;
    GPIOB->ODR |= 0b00000000000000000100000000000000;
    HAL_Delay(500);
    GPIOB->ODR &= 0b11111111111111111011111111111111;
    GPIOB->ODR |= 0b00000000000000001000000000000000;
    HAL_Delay(500);*/

    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
    /*HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 PC4 PC5 PC6
                           PC7 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : botao_esquerda_Pin botao_direita_Pin botao_acao_Pin */
  GPIO_InitStruct.Pin = botao_esquerda_Pin|botao_direita_Pin|botao_acao_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
