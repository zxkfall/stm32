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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bmp/bmp280.h"
#include "oled/ssd1306.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t Buffer[256];
char buf[1024];
int length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);

static void MX_I2C2_Init(void);

static void MX_TIM6_Init(void);

static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void OLED_Write(uint8_t x, uint8_t y, char *text) {
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(text, Font_11x18, White);
}

void OLED_Show(float value, char *prefix, char *unit) {
    ssd1306_Fill(Black);
    OLED_Write(1, 5, prefix);
    uint8_t *tempText[256];
    sprintf((char *) tempText, "%.2f %s", value, unit);
    OLED_Write(1, 25, (char *) tempText);
    ssd1306_UpdateScreen();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        GPIO_PinState status = HAL_GPIO_ReadPin(GPIOA,
                                                GPIO_PIN_1); //can not use this when output is op always read reset
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !status);
    }
}


//加入以下代码,支持printf函数,而不�??要�?�择use MicroLIB
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#if 1
//#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE {
    int handle;
};

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x) {
    x = x;
}

//重定义fputc函数
int fputc(int ch, FILE *f) {
    while ((UART5->ISR & 0X40) == 0);//循环发�??,直到发�?�完�??
    UART5->TDR = (uint8_t) ch;
    return ch;
}

#endif










/*********************************************************************/

//串口1初始�??

//STM32H7工程模板-HAL库函数版�??
//DevEBox  大越创新
//淘宝店铺：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com

/*********************************************************************/

UART_HandleTypeDef huart5; //UART句柄



uint8_t rxBuffer[100];    // 接收缓冲�??
uint8_t rxData;            // 用于存储单个接收的字�??
uint16_t rxIndex = 0;      // 接收缓冲区索�??
_Bool newDataReceived = 0; // 接收到新数据的标�??

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void removeNewlines(char *str);

// 前向声明结构体
typedef struct GPS_INFO GPS_INFO;

// 定义结构体
struct GPS_INFO {
    uint8_t *data;
    char data2[100];
};

struct GPS_INFO GET_GPS_INFO(const char *preTag);


//struct GPS_INFO gga;
//struct GPS_INFO gsv;
//struct GPS_INFO txt;
void processData(void) {
    // 在这里解析NMEA数据，例如提取GPS定位信息
    // 这里假设NMEA语句�??'$'�??头，以回车换行符'\r\n'结尾
    // 你需要根据ATGM336H的文档来解析具体的NMEA语句格式
//    $GNRMC,162641.000,V,,,,,,,270823,,,M*5A
//    $GNVTG,,,,,,,,,M*2D
//    $GNZDA,162641.000,27,08,2023,00,00*40
//    $GPTXT,01,01,01,ANTENNA OK*35
//    $GNGGA,162642.000,,,,,0,00,25.5,,,,,,*7F
//    $GNGLL,,,,,162642.000,V,M*62
//    $GPGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*02
//    $BDGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5*13
//    $GPGSV,3,1,09,02,21,168,,04,41,219,,07,29,318,,09,41,260,*79
//    $GPGSV,3,2,09,16,36,053,,21,25,163,,26,15,077,,27,60,039,*7C
//    $GPGSV,3,3,09,31,15,129,11*4C
    struct GPS_INFO gnrmc = GET_GPS_INFO("$GNRMC");
    struct GPS_INFO gnzda = GET_GPS_INFO("$GNZDA");
    struct GPS_INFO gnvtg = GET_GPS_INFO("$GNVTG");
    struct GPS_INFO gngga = GET_GPS_INFO("$GNGGA");
    struct GPS_INFO gngll = GET_GPS_INFO("$GNGLL");
    struct GPS_INFO gngsa = GET_GPS_INFO("$GNGSA");
    struct GPS_INFO gngbs = GET_GPS_INFO("$GNGBS");
    struct GPS_INFO gngns = GET_GPS_INFO("$GNGNS");

    struct GPS_INFO gpgsv = GET_GPS_INFO("$GPGSV");
    struct GPS_INFO gptxt = GET_GPS_INFO("$GPTXT");
    struct GPS_INFO gpgga = GET_GPS_INFO("$GPGGA");

    struct GPS_INFO bdgsv = GET_GPS_INFO("$BDGSV");
    struct GPS_INFO bdgga = GET_GPS_INFO("$BDGGA");

    struct GPS_INFO qzgsv = GET_GPS_INFO("$QZGSV");
    struct GPS_INFO gagsv = GET_GPS_INFO("$GAGSV");

    struct GPS_INFO glgsv = GET_GPS_INFO("$GLGSV");



//    size_t ggaL = gga.data == NULL || gga.data[0] == '\0' ? 0 : strlen((char *)gga.data);
//    size_t gsvL = gsv.data == NULL || gsv.data[0] == '\0' ? 0 : strlen((char *)gsv.data);
//    size_t txtL = txt.data == NULL || txt.data[0] == '\0' ? 0 : strlen((char *)txt.data);

//    size_t ggaL = (gga.data == NULL) ? 0 : sizeof(gga.data);
//    size_t gsvL = (gsv.data == NULL) ? 0 : sizeof(gsv.data);
//    size_t txtL = (txt.data == NULL) ? 0 : sizeof(txt.data);
//    char * res[ggaL + gsvL + txtL + 1];
//    strcpy((char *) res, gga.data == NULL ? "" : (char *)gga.data);
//    strcat((char *) res, gsv.data == NULL ? "" : (char *)gsv.data);
//    strcat((char *) res, txt.data == NULL ? "" : (char *)txt.data);

    size_t gnggaL = (gngga.data == NULL) ? 0 : strlen((char *) gngga.data);
    size_t gpgsvL = (gpgsv.data == NULL) ? 0 : strlen((char *) gpgsv.data);
    size_t gptxtL = (gptxt.data == NULL) ? 0 : strlen((char *) gptxt.data);
    size_t gpggaL = (gpgga.data == NULL) ? 0 : strlen((char *) gpgga.data);
    size_t bdgsvL = (bdgsv.data == NULL) ? 0 : strlen((char *) bdgsv.data);
    size_t bdggaL = (bdgga.data == NULL) ? 0 : strlen((char *) bdgga.data);
    size_t qzgsvL = (qzgsv.data == NULL) ? 0 : strlen((char *) qzgsv.data);
    size_t gagsvL = (gagsv.data == NULL) ? 0 : strlen((char *) gagsv.data);
    size_t glgsvL = (glgsv.data == NULL) ? 0 : strlen((char *) glgsv.data);
    size_t gnzdaL = (gnzda.data == NULL) ? 0 : strlen((char *) gnzda.data);
    size_t gnvtgL = (gnvtg.data == NULL) ? 0 : strlen((char *) gnvtg.data);
    size_t gngllL = (gngll.data == NULL) ? 0 : strlen((char *) gngll.data);
    size_t gngsaL = (gngsa.data == NULL) ? 0 : strlen((char *) gngsa.data);
    size_t gngbsL = (gngbs.data == NULL) ? 0 : strlen((char *) gngbs.data);
    size_t gngnsL = (gngns.data == NULL) ? 0 : strlen((char *) gngns.data);
    size_t totalLength =
            gnggaL + gpgsvL + gptxtL + gpggaL + bdgsvL + bdggaL + qzgsvL + gagsvL + glgsvL + gnzdaL + gnvtgL + gngllL +
            gngsaL + gngbsL + gngnsL;
    char *res = (char *) malloc(totalLength + 1); // Allocate memory for concatenated string
    res[0] = '\0'; // Initialize the concatenated string as an empty string

    if (gngga.data != NULL) strcat(res, (char *) gngga.data);
    if (gpgsv.data != NULL) strcat(res, (char *) gpgsv.data);
    if (gptxt.data != NULL) strcat(res, (char *) gptxt.data);
    if (gpgga.data != NULL) strcat(res, (char *) gpgga.data);
    if (bdgsv.data != NULL) strcat(res, (char *) bdgsv.data);
    if (bdgga.data != NULL) strcat(res, (char *) bdgga.data);
    if (qzgsv.data != NULL) strcat(res, (char *) qzgsv.data);
    if (gagsv.data != NULL) strcat(res, (char *) gagsv.data);
    if (glgsv.data != NULL) strcat(res, (char *) glgsv.data);
    if (gnzda.data != NULL) strcat(res, (char *) gnzda.data);
    if (gnvtg.data != NULL) strcat(res, (char *) gnvtg.data);
    if (gngll.data != NULL) strcat(res, (char *) gngll.data);
    if (gngsa.data != NULL) strcat(res, (char *) gngsa.data);
    if (gngbs.data != NULL) strcat(res, (char *) gngbs.data);
    if (gngns.data != NULL) strcat(res, (char *) gngns.data);

    if (res != NULL && strlen(res) != 0) {

        // Now 'res' contains the concatenated string
        // ...
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString_auto_newline((char *) res, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(100);
        // Don't forget to free the allocated memory when done
    }
    free(res);

}


struct GPS_INFO GET_GPS_INFO(const char *preTag) {

    struct GPS_INFO info;
    if (strncmp((const char *) rxBuffer, preTag, 6) == 0) {
        // 如果是GPRMC语句，解析定位信�??
        // 例如�??$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        // 在这里提取需要的定位数据并进行处�??
        // 例如：latitude, longitude, altitude�??
        info.data = rxBuffer;
        removeNewlines(info.data);
        return info;
    }
    info.data = NULL;
    return info;
}

void removeNewlines(char *str) {
    size_t len = strlen(str);
    if (len > 0 && (str[len - 1] == '\n' || str[len - 1] == '\r')) {
        str[len - 1] = '\0'; // Set the last character to null terminator
        len--;
    }
    if (len > 0 && (str[len - 1] == '\n' || str[len - 1] == '\r')) {
        str[len - 1] = '\0'; // Set the last character to null terminator
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5)//
    {
        rxBuffer[rxIndex] = rxData; // 存储接收到的字节
        if (rxIndex > 99) {
            newDataReceived = 1; // 接收到新数据
            rxIndex = 0; // 重置缓冲区索引
        } else {
            rxIndex++;
        }
        if (rxData == '\n') {
            // 收到换行符，表示数据接收完毕
            rxBuffer[rxIndex] = '\0'; // 添加字符串结束
            rxIndex = 0; // 重置缓冲区索引
        }
        HAL_UART_Receive_IT(&huart5, &rxData, 1); // 继续接收下一个
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM6_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    ssd1306_Init();

    HAL_TIM_Base_Start_IT(&htim6); // important!!! Enable TIM6 interrupt

    HAL_UART_Receive_IT(&huart5, &rxData, 1); // 初始接收数据，使用中断方式
    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == 1) {
//        if (newDataReceived){
        processData(); // 解析和处理接收到的数据
//        }
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint8_t Data[256];
    BMP280_HandleTypedef bmp280;
    float pressure, temperature, humidity;
    bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = &hi2c1;

    while (!bmp280_init(&bmp280, &bmp280.params)) {
        HAL_Delay(2000);
    }
    bool bme280p = bmp280.id == BME280_CHIP_ID;
    sprintf((char *) Data, "BMP280: found %s", bme280p ? "BME280" : "BMP280");

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString_auto_newline(Data, Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        ssd1306_Fill(Black);
        HAL_Delay(100);
        while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
            sprintf((char *) Data, "Temperature/pressure reading failed\n");
            HAL_Delay(2000);
        }

        OLED_Show(pressure, "Pressure:", "Pa");
        HAL_Delay(1000);
        OLED_Show(temperature, "Temperature:", "C");
        HAL_Delay(1000);
        if (bme280p) {
            OLED_Show(humidity, "Humidity:", "");
            HAL_Delay(1000);
        } else {
            sprintf((char *) Data, "\n");
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB;
    hi2c1.Init.OwnAddress1 = 236;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00707CBB;
    hi2c2.Init.OwnAddress1 = 240;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void) {

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */
    //interrupt time calculation: (6400-1)*(5000-1)/64M=0.5s
    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 6400 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 5000 - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void) {

    /* USER CODE BEGIN UART5_Init 0 */

    /* USER CODE END UART5_Init 0 */

    /* USER CODE BEGIN UART5_Init 1 */

    /* USER CODE END UART5_Init 1 */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 9600;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART5_Init 2 */

    /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin : PE3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PA1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PC5 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
