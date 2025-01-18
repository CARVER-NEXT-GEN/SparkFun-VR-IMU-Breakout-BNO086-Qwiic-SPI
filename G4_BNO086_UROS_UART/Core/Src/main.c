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
#include "cmsis_os.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "BNO086_SPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) if(fn != RCL_RET_OK) {};
#define HALCHECK(fn) while(fn != HAL_OK) HAL_Delay(100);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rcl_timer_t timer_;
rclc_executor_t executor;


rcl_publisher_t f64array_pub;
double f64array_data[16];
std_msgs__msg__Float64MultiArray f64array_msg = {
    .data = { .data = f64array_data, .capacity = 16, .size = 16 }
};

uint32_t sync = 0;

BNO086_t IMU_086_FRTOS;
CalibrateStatus CALIBRATE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void SensorsPublished();
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //  	BNO086_Calibration(&BNO086, &CALIBRATE);
  	BNO086_Initialization(&IMU_086_FRTOS);
  	BNO086_enableRotationVector(2500); //enable rotation vector at 400Hz (2500 microsecs)
  	//	  BNO086_enableGameRotationVector(11111); //enable Gaming Rotation vector at 90Hz (2500 microsecs)

  	BNO086_enableAccelerometer(2000); //enable Accelerometer at 400Hz (2500 microsecs)
  	BNO086_enableLinearAccelerometer(2500); //enable Linear Accelerometer at 400Hz (2500 microsecs)
  	BNO086_enableGyro(2500); //enable Gyrometer  at 400Hz (2500 microsecs)
  	BNO086_enableMagnetometer(10000); //enable Magnetometer at 100Hz (10000 microsecs)

  	 HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {

		if (sync++ >= 255) {  // Sync session at lower frequency
			rmw_uros_sync_session(1000);
			sync = 0;
		}

		SensorsPublished();
		HAL_IWDG_Refresh(&hiwdg);
	}

}


void SensorsPublished(){


	// IMU_086 acceleration
	f64array_msg.data.data[0] = IMU_086_FRTOS.acceleration.x;
	f64array_msg.data.data[1] = IMU_086_FRTOS.acceleration.y;
	f64array_msg.data.data[2] = IMU_086_FRTOS.acceleration.z;

	// IMU_086 linear acceleration
	f64array_msg.data.data[3] = IMU_086_FRTOS.linear_acceleration.x;
	f64array_msg.data.data[4] = IMU_086_FRTOS.linear_acceleration.y;
	f64array_msg.data.data[5] = IMU_086_FRTOS.linear_acceleration.z;

	// IMU_086 angular velocity
	f64array_msg.data.data[6] = IMU_086_FRTOS.angular_velocity.x;
	f64array_msg.data.data[7] = IMU_086_FRTOS.angular_velocity.y;
	f64array_msg.data.data[8] = IMU_086_FRTOS.angular_velocity.z;

	// IMU_086 magnetometer
	f64array_msg.data.data[9] = IMU_086_FRTOS.magnetometer.x;
	f64array_msg.data.data[10] = IMU_086_FRTOS.magnetometer.y;
	f64array_msg.data.data[11] = IMU_086_FRTOS.magnetometer.z;
	
	//IMU_086 quaternion
	f64array_msg.data.data[12] = IMU_086_FRTOS.quaternion.i;
	f64array_msg.data.data[13] = IMU_086_FRTOS.quaternion.j;
	f64array_msg.data.data[14] = IMU_086_FRTOS.quaternion.k;
	f64array_msg.data.data[15] = IMU_086_FRTOS.quaternion.w;



    RCCHECK(rcl_publish(&f64array_pub, &f64array_msg, NULL));

}
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &hlpuart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  allocator = rcl_get_default_allocator();

  //create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 198));

//  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);


  // create node
  rclc_node_init_default(&node, "bno_086_node", "", &support);

  rclc_publisher_init_best_effort(&f64array_pub,
  		  &node,
  	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
  	"bno086_cubemx");

	// Synchronize time with the agent (t check the connectivity of micro_ros_agent)


  // create executor
  executor = rclc_executor_get_zero_initialized_executor();

  // create timer
  rclc_timer_init_default(&timer_, &support, RCL_MS_TO_NS(9.9), timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer_); // add timer
  rclc_executor_spin(&executor);


  /* USER CODE END 5 */
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim == &htim2){
	if(BNO086_dataAvailable() == 1){
		BNO086_getData(&IMU_086_FRTOS, UNIT_RAD);
	}
  }
  /* USER CODE END Callback 1 */
}

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
