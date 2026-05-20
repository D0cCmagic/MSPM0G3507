/****************************************************************************************
	MSPM0G3507电赛小车开源方案资源分配表--MSPM0学习中心交流群828746221		
	功能	单片机端口	外设端口
	无名创新地面站通讯	
		PA10-->UART0-TXD	USB转TTL-RXD
		PA11-->UART0-RXD	USB转TTL-TXD
	机器视觉OPENMV4 MINI	
		PA8-UART1-TXD	UART3-RXD																									接树莓派
		PA9-->UART1-RXD	UART3-TXD
	手机蓝牙APP地面站	
		PA21-UART2-TXD	蓝牙串口模块RXD
		PA22-->UART2-RXD	蓝牙串口模块TXD
	US100超声波模块	
		PB2-UART3-TXD	US100-TX/TRIG
		PB3-->UART3-RXD	US100-RX/ECHO
	12路灰度传感器FPC	
	  PA31-->P1
		PA28-->P2
		PA1-->P3
		PA0-->P4
		PA25-->P5
		PA24-->P6
		PB24-->P7
		PB23-->P8
		PB19-->P9
		PB18-->P10
		PA16-->P11
		PB13-->P12
	电机控制MPWM	
		PA4-A0-PWM-CH3	  右边电机调速INA1
		PA7-->A0-PWM-CH2	右边电机调速INA2
		PA3-->A0-PWM-CH1	左边电机调速INB1
		PB14-->A0-PWM-CH0	左边电机调速INB2		
	舵机控制SPWM	
		PA15-A1-PWM-CH0	  预留1																										
		PB1-->A1-PWM-CH1	预留2																										
		PA23-->G7-PWM-CH0	预留3																										
		PA2-->G7-PWM-CH1	前轮舵机转向控制PWM																			
	编码器测速ENC	
		PB4-RIGHT-PULSE	  右边电机脉冲倍频输出P1
		PB5-->LEFT-PULSE	左边电机脉冲倍频输出P2
		PB6-->RIGHT-DIR	  右边电机脉冲鉴相输出D1
		PB7-->LEFT-DIR	  左边电机脉冲鉴相输出D2
	外置IMU接口IMU	
		PA29-I2C-SCL	MPU6050-SCL
		PA30-->I2C-SDA	MPU6050-SDA
		PB0-->HEATER	温控IO可选
	电池电压采集										
		PA26-ADC-VBAT	需要外部分压后才允许接入																步进PWM2
****************************************************************************************/

#include "ti_msp_dl_config.h"
#include "headfile.h"
#define distance_one_quan    417
uint8_t once_times = 0;
int main(void)
{
		//delay_ms(300);
	usart_irq_config();         //串口中断配置
  SYSCFG_DL_init();	      		//系统资源配置初始化	
	ncontroller_set_priority(); //中断优先级设置	
	OLED_Init();						    //显示屏初始化
	nADC_Init();					      //ADC初始化
	nGPIO_Init();						    //蜂鸣器初始化
	w25qxx_gpio_init();         //板载w25q64初始化
	ctrl_params_init();			    //控制参数初始化
	trackless_params_init();    //硬件配置初始化
	simulation_pwm_init();      //模拟PWM初始化
	rc_range_init();				    //遥控器行程初始化
	rangefinder_init();			    //测距传感器-uart3
	while(ICM206xx_Init());	    //加速度计/陀螺仪初始化
	rgb_init();							    //RGB灯初始化	
	Encoder_Init();					    //编码器资源初始化
	Button_Init();					    //板载按键初始化
	PPM_Init();							    //接收机PPM信号初始化

	timer_irq_config();
		uart1_dma_init();
		uart1_parser_init();
	
  while(1)
  {
		screen_display();//屏幕显示
		//printf("%d,%d",values[0],values[1]);

		
			
			//step_motor_ctrl_daba();							//步进电机控制PID
		if(task_mode==1)
		{
            if(smartcar_imu.state_estimation.distance>=385)
						{
							trackless_output.unlock_flag=LOCK;//上锁
						}
		}
		else if(task_mode==2)
		{
            if(smartcar_imu.state_estimation.distance>=750)
						{
							trackless_output.unlock_flag=LOCK;//上锁
						}

		}
				else if(task_mode==3)
		{
            if(smartcar_imu.state_estimation.distance>=1145)
						{
							trackless_output.unlock_flag=LOCK;//上锁
						}

		}
		else if(task_mode == 4)
		{
					if(smartcar_imu.state_estimation.distance>=1518)
						{
							trackless_output.unlock_flag=LOCK;//上锁
						}
		}
				else if(task_mode==5)
		{
		            if(smartcar_imu.state_estimation.distance>=1890)
						{
							trackless_output.unlock_flag=LOCK;//上锁
						}

		}
		
						else if(task_mode==6)
		{
			step_motor_ctrl_dianshe();

		}
						else if(task_mode==7)
		{
			step_motor_ctrl_dianshe();

		}
		
								else if(task_mode==8)
		{
			step_motor_ctrl_dianshe();

		}
		
										else if(task_mode==9)
		{
			step_motor_ctrl_dianshe();

		}
//			switch (task_mode) 
//				{
//        case 1:
//            if(smartcar_imu.state_estimation.distance>=distance_one_quan)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 2:
//             if(smartcar_imu.state_estimation.distance>=distance_one_quan*2)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 3:
//             if(smartcar_imu.state_estimation.distance>=distance_one_quan*3)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 4:
//             if(smartcar_imu.state_estimation.distance>=distance_one_quan*4)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 5:
//             if(smartcar_imu.state_estimation.distance>=distance_one_quan*5)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 6:
//            step_motor_ctrl_dianshe();
//            break;
//        case 7:
//            step_motor_ctrl_dianshe();
//            break;
//        case 8:
//            step_motor_ctrl_daba();
//				     if(smartcar_imu.state_estimation.distance>=distance_one_quan)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 9:
//            step_motor_ctrl_daba();
//				            if(smartcar_imu.state_estimation.distance>=distance_one_quan*2)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 10:
//            step_motor_ctrl_daba();
//				      if(smartcar_imu.state_estimation.distance>=distance_one_quan)
//						{
//							trackless_output.unlock_flag=LOCK;//上锁
//						}
//            break;
//        case 11:
//            // 处理模式11的代码
//            break;
//        case 12:
//            // 处理模式12的代码
//            break;
//        case 13:
//            // 处理模式13的代码
//            break;
//        case 14:
//            // 处理模式14的代码
//            break;
//        default:
//            // 处理未知模式的代码（可选）
//            break;
//				}
		//
//		Razer_ctrl(1);
//		delay_ms(500);
//		Razer_ctrl(0);
//		delay_ms(500);
		
	//delay_ms(1);
//	DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
//	set_freq_X(100);
//	set_duty_X(0.5);
//	DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
//	set_freq_Y(100);
//	set_duty_Y(0.5);
//	delay_ms(5000);
//	DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
//	set_freq_Y(100);
//	set_duty_Y(0.5);
//	DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
//	set_freq_X(100);
//	set_duty_X(0.5);
//	delay_ms(5000);

		
//				if(values[0]>=0)
//				{
//				DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
//				set_freq_X(values[0]);
//				set_duty_X(0.5);
//				}
//				else if(values[0]<0)
//				{
//				DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
//				set_freq_X(-values[0]);
//				set_duty_X(0.5);
//				}

//				if(values[1]>=0)
//				{
//				DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
//				set_freq_Y(values[1]);
//				set_duty_Y(0.5);
//				}
//				else if(values[1]<0)
//				{
//				DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
//				set_freq_Y(-values[1]);
//				set_duty_Y(0.5);
//				}
  }
}

void Razer_ctrl(bool statue)//1 is enable ,0 is not enable
{
	if(statue)DL_GPIO_clearPins(GPIO_B_01_PORT,GPIO_B_01_D0cC_PIN);
	else DL_GPIO_setPins(GPIO_B_01_PORT,GPIO_B_01_D0cC_PIN);
}


/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void maple_duty_200hz(void)
{
	//rc_data_input();							 //遥控器PPM数据处理
	get_wheel_speed();					   //获取轮胎转速
	sdk_duty_run();					  		 //SDK总任务控制
	motor_output(speed_ctrl_mode); //控制器输出
	//rangefinder_statemachine();		 //超声波传感器数据获取
	imu_data_sampling();					 //加速度计、陀螺仪数据获采集
	//get_battery_voltage();				 //ADC数据获取
	trackless_ahrs_update();			 //ahrs姿态更新
	imu_temperature_ctrl();				 //传感器恒温控制
	read_button_state_all();       //按键状态读取
	//battery_voltage_detection();	 //电池电压检测
  laser_light_work(&beep);       //电源板蜂鸣器驱动
	bling_working(0);							 //RGB灯状态机
		uart1_dma_process();
		uart1_parser_feed();
	//step_motor_ctrl_daba();
}

/***************************************
函数名:	void duty_1000hz(void)
说明: 1000hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_1000hz(void)
{
	
		
	if(sdk_work_mode==15)
	{
		gpio_input_check_channel_12_with_handle();//检测12路灰度灰度管状态,带赛道信息处理
	}
	else
	{
		//gpio_input_check_channel_12();//检测12路灰度灰度管状态
		gpio_input_check_channel_12_2024();
	}
	gpio_input_check_from_vision();//openmv机器视觉信息获取
	simulation_pwm_output();//模拟pwm输出
}


/***************************************
函数名:	void duty_100hz(void)
说明: 100hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_100hz(void)
{
	NCLink_SEND_StateMachine();//无名创新地面站发送	
	
}


/***************************************
函数名:	void duty_10hz(void)
说明: 10hz实时任务函数
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void duty_10hz(void)
{
	//手机端app地面站发送	
	bluetooth_app_send(smartcar_imu.rpy_deg[ROL],
										 smartcar_imu.rpy_deg[PIT],
										 smartcar_imu.rpy_deg[YAW],
										 speed_setup,
										 smartcar_imu.left_motor_speed_cmps,
										 smartcar_imu.right_motor_speed_cmps,
										 smartcar_imu.state_estimation.pos.x,
										 smartcar_imu.state_estimation.pos.y);
	
	

}