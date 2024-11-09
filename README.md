# SparkFun-VR-IMU-Breakout-BNO086-Qwiic

# Introduction

![system diagram](image/system_diagram.png)

# Installation

1. Git clone the STM32CubeIDE project into your workspace.
```bash
git clone -b H745_NO_UROS https://github.com/CARVER-NEXT-GEN/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI.git
```

2. Open ST32CubeIDE in your workspace and open project "BNO086_H745_NO_UROS" from file system

3. Right click on ***BNO086_H745_NO_UROS_CM7(in CM7)*** and click on **Clean Project**
![gocon](image/1.png)

4. Next, Right click on ***BNO086_H745_NO_UROS_CM7(in CM7)*** again and click on **Build Project**
![gocon](image/2.png)

5. Click once at ***BNO086_H745_NO_UROS_CM7(in CM7)*** in Project Explorer and Open **Debug Configuration** 
![gocon](image/3.png)

    
    and **double click at STM32 C/C++ Application** and BNO086_H745_NO_UROS Debug that you click once before will appear.
    
    **Note: If the previous debugger appered, Right click on it and press delete** 
![doubleclick](image/4.png)

Click in **BNO086_H745_NO_UROS_CM7 Debug** and go to Startup, click Add.. -> Project -> BNO086_H745_NO_UROS_CM4 (Make sure you've checked the checkbox along the picture) and then press OK the Apply
![cm7 debug config](image/5.png)


# Usage

## CM4

1. Include library in main.c of CM4

![include](image/include.png)


2. Define 2 object **BNO086** and **CALIBRATE**
![define object](image/begin_pv.png)

3. Use function in BEGIN2
![enable sensor](image/pvvarcm4.png)
    - **BNO080_Calibration(&BNO086, &CALIBRATE) :** You can press B1 Button before power up to enter calibration mode. The steps are provided below if a user wants to force a calibration.
    ![step calib](image/calibration_step.png)
    when you calibrate finish press B1 button again for exit from calimration mode.
    - **BNO080_Initialization(&BNO086) :** This function for initialize sensors and prepare sensor to ready for send data.
    - **BNO080_enableRotationVector(2500) :** This is for enable Rotation vector to see data quaternion and turn to roll, pitch, yaw. It have output from 9-axis sensor fusion.
    - **BNO080_enableGameRotationVector(11111) :** This is for enable Game Rotation Vector to see data quaternion. It different from Rotation vector that it not use  magnetometer in sensor fusion. 
    - **BNO080_enableAccelerometer(2000) :** This is for enable Accelerometer to see acceleration that include gravity each axis. 
    - **BNO080_enableLinearAccelerometer(2500) :** This is for enable Linear Accelerometer to see acceleration that not include gravity each axis. 
    - **BNO080_enableGyro(2500) :** This is for enable Gyrometer to see velocity each axis.
    - **BNO080_enableMagnetometer(10000) :** This is for enable Magnetometer to see magnatic field in each axis.
    - **HAL_TIM_Base_Start_IT(&htim2) :** This is for start timer interupt for control frequancy to get data from sensors.

    **Note :** each enable sensor have maximum data rate for read data. You can see in picture below
    ![max sensor rate](image/max_sensor_rate.png)
    You can calcurate frequancy in Hz to period time in microsecond and add in function enable each sensors as parametors.

4. Getting data from sensor: You can work on it by Infinite loop (USER CODE BEGIN 3) or using Timer Interrupt to control the frequency. 
![optional](image/Nouros_infiniteloop.png)

    Timer Interrupt (For now, We're using timer interrupt)
![loop_control](image/Nouros_loop_control.png)
This is function for control frequancy to get data from sensors, It have 1000 Hz as default.

## CM7
Switch to **main.c** in ***BNO086_H745_NO_UROS_CM7 (in CM7)***. 

1. Include library in main.c of CM4
![include](image/include.png)

2. Declare the **BNO086_t IMU_086** which is the object used to receive the data from CM4.
![declare](image/declare_IMU_086.png)

3. Getting data sensor from CM4 : 
    - **BNO086_READ_HSEM(&IMU_086)** : Read the data from CM4 and store in the IMU_086 object.It work in in Infinite loop (USER CODE BEGIN 3).
![cm4_infiniteloop](image/Nouros_Infiniteloop_cm4.png)

When you already debugged, you can add these below in **Live Expression**
   - **BNO086** for see data from sensor
   - **IMU_086** for check CM7 that can read data from CM4 it will has same data as BNO086
   - **CALIBRATE**  for see status sensors when you enter calibration mode
        - **accuracyQuat**, **accuracyAccel**, **accuracyGyro**, **accuracyMag** : for see status accuracy each sensors
            - IDLE : when sensor not in calibration mode
            - UNRELIABLE -> LOW -> MEDIUM -> HIGH : it is status of sensor
        - **CalibrationData** : for check data from calibrate that can stored in flash memory
            - SUCCESSFUL : can stored data
            - FAIELD : can't stored data. please try again.

![datapub](image/liveExp1.png)

![datapub](image/LiveExp2.png)