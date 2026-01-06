/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "KX134.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

TaskHandle_t DataTaskHandle;
TaskHandle_t AlgoTaskHandle;
TaskHandle_t CommTaskHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
SemaphoreHandle_t DmaCpltSem;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void DataTask_Entry(void *argument);
void AlgoTask_Entry(void *argument);
void CommTask_Entry(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  xTaskCreate(DataTask_Entry, "DataTask", 512, NULL, osPriorityHigh, &DataTaskHandle); 
  xTaskCreate(AlgoTask_Entry, "AlgoTask", 2048, NULL, osPriorityAboveNormal, &AlgoTaskHandle);
  xTaskCreate(CommTask_Entry, "CommTask", 512, NULL, osPriorityNormal, &CommTaskHandle);
  DmaCpltSem = xSemaphoreCreateBinary();
  vTaskDelete(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void DataTask_Entry(void *argument) 
{
  if (KX134_Init()) {
    KX134_SetODR(g_cfg_freq_hz);
  }
	memset(g_SensorRawBuffer, 0, sizeof(g_SensorRawBuffer));
//	uint8_t check_cntl2 = KX134_ReadReg(0x1C);
  uint16_t buffer_offset = 0;// 记录当前填到 Buffer 的哪个位置了 (0 ~ 4096)
    for(;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// Notification 等待EXTI 中断
      if (g_ResetAcqReq == 1) {
            // 收到重置命令：
            buffer_offset = 0;              // 指针归零，丢弃这一包数据
            g_ResetAcqReq = 0;              // 清除标志           
            memset(g_SensorRawBuffer[g_PingPongMgr.write_index], 0, sizeof(g_SensorRawBuffer[0]));
            continue; 
        }
      uint8_t *p_target = (uint8_t*)&g_SensorRawBuffer[g_PingPongMgr.write_index][0];
      p_target += (buffer_offset * 6); // 偏移多少字节 

      KX134_Read_FIFO_DMA(p_target);
      if (xSemaphoreTake(DmaCpltSem, 10) == pdTRUE) 
      {
        buffer_offset += FIFO_WATERMARK;
        if (buffer_offset >= FFT_POINTS) 
        {
          g_PingPongMgr.read_index = g_PingPongMgr.write_index;
          g_PingPongMgr.write_index = !g_PingPongMgr.write_index;
          buffer_offset = 0;
          xTaskNotifyGive(AlgoTaskHandle);// 通知 AlgoTask
        }
      }
      else {           
            KX134_CS_High();// 超时处理：如�?? SPI DMA 卡死了，记得在这里拉�?? CS 复位 SPI
        }
    }

		//debug
//		uint8_t check_cntl1 = KX134_ReadReg(KX134_CNTL1); 
//		// 预期：0xD8 (1101 1000) -> 16-bit, +/-64g, Operating

//		// 定义临时变量用于 Watch 窗口观察
//		volatile uint8_t raw_xl, raw_xh, raw_yl, raw_yh, raw_zl, raw_zh;
//		volatile int16_t acc_z_raw;
//		volatile float acc_z_g;

//		for(;;) {
//				// 3. 直接读取 Z 轴输出寄存器 (绕过 FIFO/DMA)
//				// ZOUT_L = 0x0A, ZOUT_H = 0x0B
//				raw_zl = KX134_ReadReg(0x0A); 
//				raw_zh = KX134_ReadReg(0x0B);
//				
//				// 合并数据
//				acc_z_raw = (int16_t)((raw_zh << 8) | raw_zl);
//				
//				// 转换物理量 (+/- 64g 量程)
//				acc_z_g = (float)acc_z_raw * KX134_SENSITIVITY;

//				// 延时一下，方便调试器刷新
//				osDelay(100); 
//				
//				// 观察 check_cntl1, raw_zh, acc_z_g 的值
//				__NOP(); 
//		}
		
}

void AlgoTask_Entry(void *argument) 
{
  Calc_Init();
    for(;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      uint8_t process_idx = g_PingPongMgr.read_index;
      int16_t *pSource = &g_SensorRawBuffer[process_idx][0];
      Process_Data(pSource);
    }
}

void CommTask_Entry(void *argument) 
{
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Protocol_HandleRxFrame(g_UartRxBuffer, g_UartRxLen, LOCAL_DEVICE_ADDR);
    }
}
/* USER CODE END Application */

