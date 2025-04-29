/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_dynamixel.h"
#include "can_sys_table.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// for gripper motion state
#define GRIPPER_CONTROL_OFF 0
#define GRIPPER_INITIALIZING 1
#define GRIPPER_INITIALIZED 2
#define GRIPPER_START_RELEASING 3
#define GRIPPER_START_HOLDING 4
#define GRIPPER_KEEP_RELEASING 5
#define GRIPPER_KEEP_HOLDING 6
#define GRIPPER_MISS_RELEASING 7
#define GRIPPER_MISS_HOLDING 8

#define RELEASING_ANGLE 330
#define HOLDING_ANGLE 290
#define INITIAL_ANGLE 330


// #define GRIPPER_CONTROL_OFF 0
// #define GRIPPER_STATE_RELEASING 1
// #define GRIPPER_STATE_HOLDING 2

// for motor on/off state
#define GRIPPER_TORQUE_OFF 0
#define GRIPPER_TORQUE_ON 1

// for connection and initial check
#define CHECK 1
#define UNCHECK 0
#define UNKNOWN -1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART TX_PA2 RX_PA3

//  paramter for experiment
int flag = 0;
uint32_t startTick, endTick, delay_ms;
int recordFlag = 0;
/////////////////////////////////////////////

volatile ServoXM4340 myServo;

// CAN TX_PD1 RX_PD0
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox = 0;
CAN_TxHeaderTypeDef TxHeader;
uint16_t canIndex;

int present_current = 0;
int last_current = 0;
int present_position = 0;
float present_angle = 0;

uint16_t goalCur = 30;
float goalAngle;
uint32_t goalForce = 0;

int gripperState = GRIPPER_CONTROL_OFF;
int torqueState = GRIPPER_TORQUE_OFF;
int initialCheck = UNCHECK;
int connectCheck = UNCHECK;
// int okFlag;

uint32_t startCheckPos;
uint32_t startCheckCur;
int initFailedCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void forceToCmd(int force, float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_HOST_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  myServo.huart = &huart2;
  myServo.ctrlPort = GPIOC;
  myServo.ctrlPin = GPIO_PIN_4;
  myServo.ID = 1;

  // servo state check for debugging
  setServo_TorqueENA(&myServo, TORQUE_DISABLE);
  setServo_OperatingMode(&myServo, CurrentBased_POS_CtrlMode);

  myServo.TorqueENA = getServo_TorqueENA(&myServo);
  myServo.BaudRate = getServo_BaudRate(&myServo);
  myServo.OperatingMode = getServo_OperatingMode(&myServo);

  // setServo_TorqueENA(&myServo, TORQUE_ENABLE);
  // torqueState = GRIPPER_TORQUE_ON;
  // setServo_GoalPosition(&myServo, 300);
  // HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);

  // Set Can Filter
  CAN1_FilterInit(0,                     // Bank
                  CAN_FILTERMODE_IDLIST, // Mode
                  CAN_FILTERSCALE_16BIT, // Scale
                  1 << 5,                // FilterIdHigh
                  2 << 5,                // FilterIdLow
                  7 << 5,                // MaskIdHigh
                  0 << 5,                // MaskIdLow
                  CAN_FILTER_FIFO0,      // FIFOx
                  ENABLE);               // Activation

  // Enable Can Interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);

  // 4 state managing
  gripperState = GRIPPER_CONTROL_OFF;
  torqueState = GRIPPER_TORQUE_OFF;
  initialCheck = UNKNOWN;
  connectCheck = UNKNOWN;
  initFailedCount = 0;

  // startTick = 0;
  // endTick = 0;
  // delay_ms = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // delay_ms = endTick - startTick;

    // TxDataAssign(TxData, CANSYS_STATE_STM_INIT_OK, 0);
    // CAN_TxheaderSetup(&TxHeader, 1, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
    // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

    // State check and switch
    // if (myServo.statsGRIPPER_STATE_UNKNOWN;

    //////////////////////////   some none-state managing   ////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////

    // for motor on or off
    if (torqueState == GRIPPER_TORQUE_ON && myServo.TorqueENA == TORQUE_DISABLE)
    {
      setServo_TorqueENA(&myServo, TORQUE_ENABLE);
      myServo.TorqueENA = getServo_TorqueENA(&myServo);
    }

    if (connectCheck == UNCHECK)
    {
      TxDataAssign(TxData, CANSYS_STATE_STM_CONNECT_OK, 0);
      CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        // HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
        flag = 0;
      }
      else
        flag = 1;
      connectCheck = CHECK;
    }

    /************************  state machine ***************************/
    /////////////////////////////////////////////////////////////////////////////

    // Monitor motor
    last_current = present_current;
    present_current = getServo_PresentCurrent(&myServo);
    present_position = getServo_PresentPosition(&myServo);
    present_angle = (float)present_position * 360.0 / 4096.0;

    if (torqueState == GRIPPER_TORQUE_ON)
    {
      switch (gripperState)
      {
      case GRIPPER_INITIALIZING:
      {
        // goalAngle = INITIAL_ANGLE;
        // goalCur = 30;

        // keep trying initialize until initialCheck success
        if (fabsf(present_angle - INITIAL_ANGLE) < 5)
        {
          if (abs(present_current) > 10)
          {
            if (startCheckCur == UNCHECK)
              startCheckCur = HAL_GetTick();
            else if (HAL_GetTick() - startCheckCur > 1000)
              initialCheck = startCheckCur = UNCHECK;
          }
          else
            initialCheck = CHECK;
          startCheckPos = UNCHECK;
        }
        else
        {
          if (startCheckPos == UNCHECK)
            startCheckPos = HAL_GetTick();
          else if (HAL_GetTick() - startCheckPos > 1000)
            initialCheck = startCheckPos = UNCHECK;
        }

        if (initialCheck == CHECK)
        {
          TxDataAssign(TxData, CANSYS_STATE_STM_INIT_OK, 0);
          CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
          gripperState = GRIPPER_INITIALIZED;
          startCheckPos = startCheckCur = UNCHECK;
        }
        else if (initialCheck == UNCHECK)
        {
          initialCheck = UNKNOWN;
          TxDataAssign(TxData, CANSYS_STATE_STM_INIT_NOTOK, 0);
          CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        }

        break;
      }

      /********************************************************************* */
      case GRIPPER_INITIALIZED:
      {
        // goalAngle = INITIAL_ANGLE;
        // goalCur = 30;

        int ok_Flag = true;

        if (fabsf(present_angle - INITIAL_ANGLE) < 5)
        {
          if (abs(present_current) > 10)
          {
            if (startCheckCur == false)
              startCheckCur = HAL_GetTick();
            else if (HAL_GetTick() - startCheckCur > 500)
              ok_Flag = startCheckCur = false;
          }
          else
            startCheckCur = 0;
          startCheckPos = 0;
        }
        else
        {
          if (startCheckPos == false)
            startCheckPos = HAL_GetTick();
          else if (HAL_GetTick() - startCheckPos > 500)
            ok_Flag = startCheckPos = false;
        }

        if (!ok_Flag)
        {
          gripperState = GRIPPER_INITIALIZING;
          TxDataAssign(TxData, CANSYS_STATE_STM_INIT_NOTOK, 0);
          CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
          initFailedCount = startCheckPos = startCheckCur = false;
          initialCheck = UNKNOWN;
        }

        break;
      }

      /********************************************************************* */
      case GRIPPER_START_RELEASING:
      {
        // send pi start releasing
        TxDataAssign(TxData, CANSYS_STATE_STM_START_RELEASING, 0);
        CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

        // motion commend
        // goalAngle = 320;
        // goalCur = 200;

        // next state

        gripperState = GRIPPER_KEEP_RELEASING;
        break;
      }
      /********************************************************************* */
      case GRIPPER_START_HOLDING:
      {
        // send pi start holding
        TxDataAssign(TxData, CANSYS_STATE_STM_START_HOLDING, 0);
        CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

        // motion commend
        // goalAngle = 280;
        // goalCur = 200;

        // next state
        gripperState = GRIPPER_KEEP_HOLDING;
        break;
      }

      /********************************************************************* */
      case GRIPPER_KEEP_RELEASING:
      {
        // goalCur = 30;
        // goalAngle = RELEASING_ANGLE;
        int missRelFlag = true;

        if (fabsf(present_angle - RELEASING_ANGLE) < 5)
        {
          if (abs(present_current) > 10)
          {
            if (startCheckCur == false)
              startCheckCur = HAL_GetTick();
            else if (HAL_GetTick() - startCheckCur > 500)
              missRelFlag = startCheckCur = false;
          }
          else
            startCheckCur = 0;
          startCheckPos = 0;
        }
        else
        {
          if (startCheckPos == false)
            startCheckPos = HAL_GetTick();
          else if (HAL_GetTick() - startCheckPos > 500)
            missRelFlag = startCheckPos = false;
        }

        if (!missRelFlag)
        {
          gripperState = GRIPPER_MISS_RELEASING;
          TxDataAssign(TxData, CANSYS_STATE_STM_MISS_RELEASING, 0);
          CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        }

        break;
      }

      /********************************************************************* */
      case GRIPPER_KEEP_HOLDING:
      {
        // goalCur = 80;
        // goalAngle = HOLDING_ANGLE - 20;

        int missHolFlag = true;

        /*******************************/
        /*******************************/
        // this condition is only for delay test
        /*******************************/
        /*******************************/
        if (present_angle < HOLDING_ANGLE + 10 && recordFlag)
        {
          delay_ms = HAL_GetTick() - startTick;
          recordFlag = false;
          // delay_ms = 0;
        }
        /*******************************/
        /*******************************/

        if (present_angle < HOLDING_ANGLE - 9)
        {
          missHolFlag = startCheckPos = false;
          // HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
        }

        if (present_current > 45)
        {
          // HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
          missHolFlag = startCheckPos = false;
        }
        if (last_current > 60 && fabs(present_angle) - HOLDING_ANGLE < 2 && present_current - last_current > 30)
        {
          missHolFlag = startCheckPos = false;
          // HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
        }

        if (!missHolFlag)
        {
          goalAngle = RELEASING_ANGLE;
          goalCur = 200;
          gripperState = GRIPPER_MISS_HOLDING;
          TxDataAssign(TxData, CANSYS_STATE_STM_MISS_HOLDING, 0);
          CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        }

        break;
      }

      /********************************************************************* */
      case GRIPPER_MISS_HOLDING:
      {
        goalCur = 30;
        goalAngle = RELEASING_ANGLE;

        break;
      }

      /********************************************************************* */
      case GRIPPER_MISS_RELEASING:
      {
        goalCur = 20;
        goalAngle = RELEASING_ANGLE;

        break;
      }
      default:
        break;
      }
      // set motor
      // servoProfile(&myServo, goalAngle, goalCur);

      // if (fabsf(present_angle - goalAngle) < 2)
      //   setServo_GoalCurrent(&myServo, goalCur);
      // else
      //   setServo_GoalCurrent(&myServo, 200);

      setServo_GoalPosition(&myServo, goalAngle);
      setServo_GoalCurrent(&myServo, goalCur);
    }
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // *********************************************************************************************************************
    // if (torqueState == GRIPPER_TORQUE_ON)
    // {
    //   if (goalForce == 10 && gripperState == GRIPPER_STATE_RELEASING)
    //   {
    //     goalAngle = 320;
    //     goalCur = 30;
    //     if (startAction_flag)
    //     {
    //       TxDataAssign(TxData, CANSYS_STATE_STM_START_RELEASING, 0);
    //       CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
    //       HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    //       startAction_flag = false;
    //     }
    //   }
    //   else if (goalForce == 200 && gripperState == GRIPPER_STATE_HOLDING)
    //   {
    //     goalAngle = 285;
    //     goalCur = 100;
    //     if (startAction_flag)
    //     {
    //       TxDataAssign(TxData, CANSYS_STATE_STM_START_HOLDING, 0);
    //       CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
    //       HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    //       startAction_flag = false;
    //     }
    //   }

    //   setServo_GoalPosition(&myServo, goalAngle);
    //   setServo_GoalCurrent(&myServo, goalCur);
    // }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  // UNUSED(hcan);
  // HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);

  CAN_RxHeaderTypeDef RxHeader;
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {

    // HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
    canIndex = (uint16_t)(((RxData[1] << 4) | RxData[2]) << 8) | RxData[3];

    switch (canIndex)
    {

    case CANSYS_CMD_PI_CONNECTING: // connecting check
      connectCheck = UNCHECK;
      break;

    case CANSYS_CMD_PI_INITSTATE:

      // TxDataAssign(TxData, CANSYS_STATE_STM_INIT_OK, 0);
      // CAN_TxheaderSetup(&TxHeader, TxID_ONLY_PI, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE);
      // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

      // after this command, no EEPROM parameter can be modified
      // servo is now able to rotate

      torqueState = GRIPPER_TORQUE_ON;
      gripperState = GRIPPER_INITIALIZING;
      goalForce = RxParamToInt(RxData);
      forceToCmd(goalForce, INITIAL_ANGLE);
      initFailedCount = startCheckPos = startCheckCur = UNCHECK;
      initialCheck = UNKNOWN;
      break;

    case CANSYS_CMD_PI_HOLD:
      // HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
      startTick = HAL_GetTick();
      recordFlag = true;
      gripperState = GRIPPER_START_HOLDING;
      goalForce = RxParamToInt(RxData);
      forceToCmd(goalForce, HOLDING_ANGLE - 10);
      startCheckPos = startCheckCur = UNCHECK;
      break;

    case CANSYS_CMD_PI_REALEASE:
      // HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
      gripperState = GRIPPER_START_RELEASING;
      goalForce = RxParamToInt(RxData);
      forceToCmd(goalForce, RELEASING_ANGLE);
      startCheckPos = startCheckCur = UNCHECK;
      break;

    // case CANSYS_STATE_ONLY_FOR_EXPERIMENT_HOLDING:
    //   startTick = HAL_GetTick();
    //   gripperState = GRIPPER_STATE_HOLDING;
    //   torqueState = GRIPPER_TORQUE_ON;
    //   initialCheck = CHECK;
    //   connectCheck = CHECK;
    //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
    //   goalForce = RxParamToInt(RxData);

    //   break;

    // case CANSYS_STATE_ONLY_FOR_EXPERIMENT_RELEASING:
    //   startTick = 0;
    //   endTick = 0;
    //   gripperState = GRIPPER_STATE_RELEASING;
    //   torqueState = GRIPPER_TORQUE_ON;
    //   initialCheck = CHECK;
    //   connectCheck = CHECK;
    //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
    //   goalForce = RxParamToInt(RxData);

    //   break;
    default:
      break;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART2)
  {
    UNUSED(huart);
    setServoResponse_RxFinished(&myServo, true);
  }
}

void forceToCmd(int force, float angle)
{
  goalAngle = angle;
  if (force == 10)
    goalCur = 30;
  else if (force == 200)
    goalCur = 100;
}

// void servoProfile(ServoXM4340 *myServo, float goalAngle, uint16_t goalCur)
// {

// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   if (GPIO_Pin == GPIO_PIN_11)
//   {
//     endTick = HAL_GetTick();
//   }
// }
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
