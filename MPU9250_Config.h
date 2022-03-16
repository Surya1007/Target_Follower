/*
 *  MPU9250_Config.h
 *	Original code taken from github profile: desertkun
 *	Modified due to improper device initialization from original author
 *	Configuration parameters for MPU9250.h file
 *  Created on: 23th Oct, 2021
 *  Author: Surya Teja
 */

#ifndef UTIL_MPU9250_CONFIG_H_
#define UTIL_MPU9250_CONFIG_H_

SPI_HandleTypeDef hspi1;

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

#define MPU9250_SPI			hspi1		// SPI bus
#define	MPU9250_CS_GPIO		GPIOA		// Chip Select GPIO port
#define	MPU9250_CS_PIN		GPIO_PIN_15	// Chip Select GPIO pin


#endif /* UTIL_MPU9250_CONFIG_H_ */
