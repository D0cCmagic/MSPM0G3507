#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "uart1_parser.h"


controller_output trackless_output={
	.unlock_flag=LOCK,
};

//����Ƕȡ����ٶȿ�����
float	steer_angle_expect=0,steer_angle_error=0,steer_angle_output=0;
float	steer_gyro_expect=0,steer_gyro_output=0;
float	steer_gyro_scale=steer_gyro_scale_default;//0.5f

controller steerangle_ctrl;		//ת��Ƕ��⻷�������ṹ��
controller steergyro_ctrl;		//ת����ٶ��ڻ��������ṹ��//���ٶȿ��Ʋ���   1.2  0  3							 
controller distance_ctrl,azimuth_ctrl;//���ֲ���λ�á��������
controller step_motor_X,step_motor_Y;//X,Y�Ჽ��PID
/***************************************************
������: void trackless_params_init(void)
˵��:	������ʼ��
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void trackless_params_init(void)
{
	//С��Ӳ����ز���:���������򡢵�������������Ӱ뾶�������ֵ��
	float tmp_left_enc_dir,tmp_right_enc_dir,tmp_left_move_dir,tmp_right_move_dir,tmp_wheel_radius_cm,tmp_pulse_num_per_circle;
	float tmp_servo_mid_value1,tmp_servo_mid_value2;
	ReadFlashParameterOne(LEFT_ENC_DIR_CFG,&tmp_left_enc_dir);
	ReadFlashParameterOne(RIGHT_ENC_DIR_CFG,&tmp_right_enc_dir);	
	ReadFlashParameterOne(LEFT_MOVE_DIR_CFG,&tmp_left_move_dir);
	ReadFlashParameterOne(RIGHT_MOVE_DIR_CFG,&tmp_right_move_dir);
	ReadFlashParameterOne(TIRE_RADIUS_CM_CFG,&tmp_wheel_radius_cm);
	ReadFlashParameterOne(PULSE_NPC_CFG,&tmp_pulse_num_per_circle);
	ReadFlashParameterOne(SERVO_MEDIAN_VALUE_1,&tmp_servo_mid_value1);
	ReadFlashParameterOne(SERVO_MEDIAN_VALUE_2,&tmp_servo_mid_value2);
	if(isnan(tmp_left_enc_dir)==0) 					trackless_motor.left_encoder_dir_config=tmp_left_enc_dir;				else trackless_motor.left_encoder_dir_config=left_motor_encoder_dir_default;
	if(isnan(tmp_right_enc_dir)==0) 				trackless_motor.right_encoder_dir_config=tmp_right_enc_dir;			else trackless_motor.right_encoder_dir_config=right_motor_encoder_dir_default;
	if(isnan(tmp_left_move_dir)==0) 				trackless_motor.left_motion_dir_config=tmp_left_move_dir;				else trackless_motor.left_motion_dir_config=left_motion_dir_default;
	if(isnan(tmp_right_move_dir)==0)				trackless_motor.right_motion_dir_config=tmp_right_move_dir;			else trackless_motor.right_motion_dir_config=right_motion_dir_default;
	if(isnan(tmp_wheel_radius_cm)==0) 			trackless_motor.wheel_radius_cm=tmp_wheel_radius_cm;						else trackless_motor.wheel_radius_cm=tire_radius_cm_default;
	if(isnan(tmp_pulse_num_per_circle)==0)	trackless_motor.pulse_num_per_circle=tmp_pulse_num_per_circle;	else trackless_motor.pulse_num_per_circle=pulse_cnt_per_circle_default;
	if(isnan(tmp_servo_mid_value1)==0) 			trackless_motor.servo_median_value1=tmp_servo_mid_value1;				else trackless_motor.servo_median_value1=servo_median_value1_default;
	if(isnan(tmp_servo_mid_value2)==0)			trackless_motor.servo_median_value2=tmp_servo_mid_value2;				else trackless_motor.servo_median_value2=servo_median_value2_default;
	
	//�ٶȿ�����ز���:�����ٶȡ�����ģʽ
	float tmp_speed_setup=0,tmp_work_mode=0;
	ReadFlashParameterOne(SPEED_SETUP,&tmp_speed_setup);
	ReadFlashParameterOne(WORK_MODE,&tmp_work_mode);
	if(isnan(tmp_speed_setup)==0) speed_setup=tmp_speed_setup;else speed_setup=speed_expect_default;
	if(isnan(tmp_work_mode)==0) 	sdk_work_mode=tmp_work_mode;		else sdk_work_mode=work_mode_default;
	
	//ת�������ز���:�Ƕ�/���ٶ�pid
	float tmp_turn_kp[2],tmp_turn_ki[2],tmp_turn_kd[2],tmp_turn_scale;
	float tmp_steer_gyro_kp,tmp_steer_gyro_ki,tmp_steer_gyro_kd,tmp_steer_gyro_scale;
	ReadFlashParameterThree(CTRL_TURN_KP1,&tmp_turn_kp[0],&tmp_turn_ki[0],&tmp_turn_kd[0]);
	ReadFlashParameterOne(CTRL_TURN_SCALE,&tmp_turn_scale);
	ReadFlashParameterThree(CTRL_TURN_KP2,&tmp_turn_kp[1],&tmp_turn_ki[1],&tmp_turn_kd[1]);
	ReadFlashParameterThree(CTRL_GYRO_KP,&tmp_steer_gyro_kp,&tmp_steer_gyro_ki,&tmp_steer_gyro_kd);
	ReadFlashParameterOne(CTRL_GYRO_SCALE,&tmp_steer_gyro_scale);	
	//
	if(isnan(tmp_turn_kp[0])==0) 				 seektrack_ctrl[0].kp   =tmp_turn_kp[0];								else seektrack_ctrl[0].kp   =turn_kp_default1;
	if(isnan(tmp_turn_ki[0])==0) 				 seektrack_ctrl[0].ki   =tmp_turn_ki[0];								else seektrack_ctrl[0].ki   =turn_ki_default1;	
	if(isnan(tmp_turn_kd[0])==0) 				 seektrack_ctrl[0].kd   =tmp_turn_kd[0];								else seektrack_ctrl[0].kd   =turn_kd_default1;	
	if(isnan(tmp_turn_scale)==0) 			 turn_scale=tmp_turn_scale;							else turn_scale=turn_scale_default;
	
	if(isnan(tmp_turn_kp[1])==0) 				 seektrack_ctrl[1].kp   =tmp_turn_kp[1];								else seektrack_ctrl[1].kp   =turn_kp_default2;
	if(isnan(tmp_turn_ki[1])==0) 				 seektrack_ctrl[1].ki   =tmp_turn_ki[1];								else seektrack_ctrl[1].ki   =turn_ki_default2;	
	if(isnan(tmp_turn_kd[1])==0) 				 seektrack_ctrl[1].kd   =tmp_turn_kd[1];								else seektrack_ctrl[1].kd   =turn_kd_default2;	
	//
	if(isnan(tmp_steer_gyro_kp)==0) 	 steergyro_ctrl.kp	 =tmp_steer_gyro_kp;		else steergyro_ctrl.kp   =steer_gyro_kp_default;
	if(isnan(tmp_steer_gyro_ki)==0) 	 steergyro_ctrl.ki	 =tmp_steer_gyro_ki;		else steergyro_ctrl.ki   =steer_gyro_ki_default;
	if(isnan(tmp_steer_gyro_kd)==0) 	 steergyro_ctrl.kd	 =tmp_steer_gyro_kd;		else steergyro_ctrl.kd   =steer_gyro_kd_default;
	if(isnan(tmp_steer_gyro_scale)==0) steer_gyro_scale=tmp_steer_gyro_scale; else steer_gyro_scale=steer_gyro_scale_default;	
	
	//�ٶȿ���pid����
	float tmp_speed_kp,tmp_speed_ki,tmp_speed_kd;
	ReadFlashParameterThree(CTRL_SPEED_KP,&tmp_speed_kp,&tmp_speed_ki,&tmp_speed_kd);
	//
	if(isnan(tmp_speed_kp)==0) 				 speed_kp=tmp_speed_kp;		else speed_kp   =speed_kp_default;
	if(isnan(tmp_speed_ki)==0) 				 speed_ki=tmp_speed_ki;		else speed_ki   =speed_ki_default;	
	if(isnan(tmp_speed_kd)==0) 				 speed_kd=tmp_speed_kd;		else speed_kd   =speed_kd_default;
  //ƽ�����pid����
	float tmp_balance_kp,tmp_balance_ki,tmp_balance_kd,tmp_balance_angle;
	float tmp_balance_gyro_kp,tmp_balance_gyro_ki,tmp_balance_gyro_kd;
	
	ReadFlashParameterThree(CTRL_BALANCE_ANGLE1_KP,&tmp_balance_kp,&tmp_balance_ki,&tmp_balance_kd);
	ReadFlashParameterOne(BALANCE_ANGLE_EXPECT,&tmp_balance_angle);
	ReadFlashParameterThree(CTRL_BALANCE_GYRO_KP,&tmp_balance_gyro_kp,&tmp_balance_gyro_ki,&tmp_balance_gyro_kd);	
	//
	if(isnan(tmp_balance_kp)==0) 			selfbalance_angle1_ctrl.kp=tmp_balance_kp;		     else selfbalance_angle1_ctrl.kp     =balance_angle_kp1_default;
	if(isnan(tmp_balance_ki)==0) 			selfbalance_angle1_ctrl.ki=tmp_balance_ki;		     else selfbalance_angle1_ctrl.ki     =balance_angle_ki1_default;	
	if(isnan(tmp_balance_kd)==0) 			selfbalance_angle1_ctrl.kd=tmp_balance_kd;		     else selfbalance_angle1_ctrl.kd     =balance_angle_kd1_default;
	if(isnan(tmp_balance_angle)==0)   balance_angle_expect=tmp_balance_angle;	 				 else balance_angle_expect =balance_angle_default;
	if(isnan(tmp_balance_gyro_kp)==0) 			selfbalance_gyro_ctrl.kp=tmp_balance_gyro_kp;		     else selfbalance_gyro_ctrl.kp     =balance_gyro_kp_default;
	if(isnan(tmp_balance_gyro_ki)==0) 			selfbalance_gyro_ctrl.ki=tmp_balance_gyro_ki;		     else selfbalance_gyro_ctrl.ki     =balance_gyro_ki_default;	
	if(isnan(tmp_balance_gyro_kd)==0) 			selfbalance_gyro_ctrl.kd=tmp_balance_gyro_kd;		     else selfbalance_gyro_ctrl.kd     =balance_gyro_kd_default;
	
	//ƽ���ٶȿ���pid����
	float tmp_bspeed_kp,tmp_bspeed_ki,tmp_bspeed_kd;
	ReadFlashParameterThree(CTRL_BALANCE_SPEED_KP,&tmp_bspeed_kp,&tmp_bspeed_ki,&tmp_bspeed_kd);
	if(isnan(tmp_bspeed_kp)==0) 				 balance_speed_kp=tmp_bspeed_kp;		else balance_speed_kp   =balance_speed_kp_default;
	if(isnan(tmp_bspeed_ki)==0) 				 balance_speed_ki=tmp_bspeed_ki;		else balance_speed_ki   =balance_speed_ki_default;	
	if(isnan(tmp_bspeed_kd)==0) 				 balance_speed_kd=tmp_bspeed_kd;		else balance_speed_kd   =balance_speed_kd_default;
	
	//ƽ���ٶ����ٶȡ�ת��ʹ�ܲ���
	float bspeed_ctrl_enable,bsteer_ctrl_enable,bctrl_number;
	ReadFlashParameterOne(BALANCE_VEL_CTRL_ENABLE,&bspeed_ctrl_enable);
	ReadFlashParameterOne(BALANCE_DIR_CTRL_ENABLE,&bsteer_ctrl_enable);
	ReadFlashParameterOne(BALANCE_CTRL_NUMBER,&bctrl_number);
	if(isnan(bspeed_ctrl_enable)==0) 	  balance_speed_ctrl_enable=bspeed_ctrl_enable;		else balance_speed_ctrl_enable   =balance_speed_ctrl_enable_default;
	if(isnan(bsteer_ctrl_enable)==0) 	  balance_steer_ctrl_enable=bsteer_ctrl_enable;		else balance_steer_ctrl_enable   =balance_steer_ctrl_enable_default;
	if(isnan(bctrl_number)==0) 	        balance_ctrl_loop_num=bctrl_number;		          else balance_ctrl_loop_num       =balance_ctrl_number_enable_default;
	
	
	//Ԥ������
	float tmp_reserved_params[20];
	for(uint16_t i=0;i<20;i++)
	{
		ReadFlashParameterOne(RESERVED_PARAMS_1+i,&tmp_reserved_params[i]);
	}
	if(isnan(tmp_reserved_params[0])==0) 	park_params._track_speed_cmps=tmp_reserved_params[0];								else park_params._track_speed_cmps=track_speed_cmps_default;
	if(isnan(tmp_reserved_params[1])==0) 	park_params._start_point_adjust1=tmp_reserved_params[1];						else park_params._start_point_adjust1=start_point_adjust1_default;
	if(isnan(tmp_reserved_params[2])==0) 	park_params._forward_distance_cm=tmp_reserved_params[2];						else park_params._forward_distance_cm=forward_distance_cm_default;
	if(isnan(tmp_reserved_params[3])==0) 	park_params._backward_distance1_cm=tmp_reserved_params[3];					else park_params._backward_distance1_cm=backward_distance1_cm_default;
	if(isnan(tmp_reserved_params[4])==0) 	park_params._backward_distance2_cm=tmp_reserved_params[4];					else park_params._backward_distance2_cm=backward_distance2_cm_default;
	if(isnan(tmp_reserved_params[5])==0) 	park_params._out_forward_distance1_cm=tmp_reserved_params[5];				else park_params._out_forward_distance1_cm=out_forward_distance1_cm_default;
	if(isnan(tmp_reserved_params[6])==0) 	park_params._out_forward_distance2_cm=tmp_reserved_params[6];				else park_params._out_forward_distance2_cm=out_forward_distance2_cm_default;
	if(isnan(tmp_reserved_params[7])==0) 	park_params._start_point_adjust2=tmp_reserved_params[7];						else park_params._start_point_adjust2=start_point_adjust2_default;
	if(isnan(tmp_reserved_params[8])==0) 	park_params._parallel_backward_distance1_cm=tmp_reserved_params[8];	else park_params._parallel_backward_distance1_cm=parallel_backward_distance1_cm_default;
	if(isnan(tmp_reserved_params[9])==0) 	park_params._parallel_backward_distance2_cm=tmp_reserved_params[9];	else park_params._parallel_backward_distance2_cm=parallel_backward_distance2_cm_default;
	if(isnan(tmp_reserved_params[10])==0) park_params._parallel_backward_distance3_cm=tmp_reserved_params[10];else park_params._parallel_backward_distance3_cm=parallel_backward_distance3_cm_default;
	
	//
	if(isnan(tmp_reserved_params[11])==0) 	_distance_ctrl_speed_max=tmp_reserved_params[11];								else _distance_ctrl_speed_max=distance_ctrl_speed_max;
	if(isnan(tmp_reserved_params[12])==0) 	_self_guided_tracking_speed=tmp_reserved_params[12];						else _self_guided_tracking_speed=self_guided_tracking_speed;
	if(isnan(tmp_reserved_params[13])==0) 	_move_diagonal_angle1=tmp_reserved_params[13];						      else _move_diagonal_angle1=move_diagonal_angle1;
	if(isnan(tmp_reserved_params[14])==0) 	_move_diagonal_distance1=tmp_reserved_params[14];					      else _move_diagonal_distance1=move_diagonal_distance1;	
	if(isnan(tmp_reserved_params[15])==0) 	_move_diagonal_angle2=tmp_reserved_params[15];						      else _move_diagonal_angle2=move_diagonal_angle2;
	if(isnan(tmp_reserved_params[16])==0) 	_move_diagonal_distance2=tmp_reserved_params[16];					      else _move_diagonal_distance2=move_diagonal_distance2;	

 //��ص�ѹ��������
	float tmp_vbat_enable,tmp_vbat_upper,tmp_vbat_lower;
	ReadFlashParameterThree(NO_VOLTAGE_ENABLE,&tmp_vbat_enable,&tmp_vbat_upper,&tmp_vbat_lower);		
	if(isnan(tmp_vbat_enable)==0) 	vbat.enable=tmp_vbat_enable;		else vbat.enable=no_voltage_enable_default;
	if(isnan(tmp_vbat_upper)==0) 	  vbat.upper=tmp_vbat_upper;			else vbat.upper=no_voltage_upper_default;
	if(isnan(tmp_vbat_lower)==0)   	vbat.lower=tmp_vbat_lower;			else vbat.lower=no_voltage_lower_default;	
}

/***************************************************
������: void ctrl_params_init(void)
˵��:	pid������������ʼ��
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void ctrl_params_init(void)
{
	pid_control_init(&seektrack_ctrl[0],//����ʼ���������ṹ��
										turn_kp_default1,//��������
										turn_ki_default1,//���ֲ���
										turn_kd_default1,//΢�ֲ���
										20, //ƫ���޷�ֵ
										0,  //�����޷�ֵ
										500,//����������޷�ֵ
										1,	//ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										6); //΢�ּ��ʱ��
	

	pid_control_init(&seektrack_ctrl[1],//����ʼ���������ṹ��
										turn_kp_default2,//��������
										turn_ki_default2,//���ֲ���
										turn_kd_default2,//΢�ֲ���
										20, //ƫ���޷�ֵ
										0,  //�����޷�ֵ
										500,//����������޷�ֵ
										1,	//ƫ���޷���־λ
										0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1); //΢�ּ��ʱ��

	pid_control_init(&selfbalance_angle1_ctrl,//����ʼ���������ṹ��
									 balance_angle_kp1_default,//��������
									 balance_angle_ki1_default,//���ֲ���
									 balance_angle_kd1_default,//΢�ֲ���
									 30, //ƫ���޷�ֵ
									 0,  //�����޷�ֵ
									 999,//����������޷�ֵ
									 1,	 //ƫ���޷���־λ
									 0,0,//���ַ����־λ��������ֿ���ʱ���޷�ֵ
									 1); //΢�ּ��ʱ��
									 
	pid_control_init(&selfbalance_gyro_ctrl,//����ʼ���������ṹ��
									 balance_gyro_kp_default,//��������
									 balance_gyro_ki_default,//���ֲ���
									 balance_gyro_kd_default,//΢�ֲ���
										500, //ƫ���޷�ֵ
										300, //�����޷�ֵ
										999, //����������޷�ֵ
										1,  	//ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��	

	pid_control_init(&steerangle_ctrl,//����ʼ���������ṹ��
										5.0f,//��������
										0,	 //���ֲ���
										0,	 //΢�ֲ���
										50,  //ƫ���޷�ֵ
										0,   //�����޷�ֵ
										500, //����������޷�ֵ
										1,   //ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��

	pid_control_init(&steergyro_ctrl,//����ʼ���������ṹ��
										steer_gyro_kp_default,//��������
										steer_gyro_ki_default,//���ֲ���
										steer_gyro_kd_default,//΢�ֲ���
										200, //ƫ���޷�ֵ
										300, //�����޷�ֵ
										500, //����������޷�ֵ
										1,  	//ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��
	
	pid_control_init(&distance_ctrl,//����ʼ���������ṹ��
										0.5,//��������
										0,//���ֲ���
										0,//΢�ֲ���
										500, //ƫ���޷�ֵ
										0, //�����޷�ֵ
										500, //����������޷�ֵ
										1,  	//ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��
										
										
	pid_control_init(&step_motor_X,//����ʼ���������ṹ��
										5,//��������
										0,//���ֲ���
										0,//΢�ֲ���
										50, //ƫ���޷�ֵ
										0, //�����޷�ֵ
										200, //����������޷�ֵ
										1,  	//ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��
										
										
		pid_control_init(&step_motor_Y,//����ʼ���������ṹ��
										3,//��������
										0,//���ֲ���
										0,//΢�ֲ���
										50, //ƫ���޷�ֵ
										0, //�����޷�ֵ
										200, //����������޷�ֵ
										1,  	//ƫ���޷���־λ
										0,0, //���ַ����־λ��������ֿ���ʱ���޷�ֵ
										1);  //΢�ּ��ʱ��
	azimuth_ctrl=steerangle_ctrl;	
}

/***************************************************
������: void steer_angle_ctrl(void)
˵��:	����Ƕȿ���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void steer_angle_ctrl(void)
{
	//�Ƕȿ���
	steerangle_ctrl.expect=steer_angle_expect;
	steerangle_ctrl.feedback=smartcar_imu.rpy_deg[_YAW];
	steerangle_ctrl.error=steerangle_ctrl.expect-steerangle_ctrl.feedback;
	/***********************ƫ����ƫ���+-180����*****************************/
	if(steerangle_ctrl.error<-180) steerangle_ctrl.error=steerangle_ctrl.error+360;
	if(steerangle_ctrl.error>	180) steerangle_ctrl.error=steerangle_ctrl.error-360;				
	steerangle_ctrl.error=constrain_float(steerangle_ctrl.error,-50,50);
	steer_angle_output=steerangle_ctrl.kp*steerangle_ctrl.error;
	steer_angle_output=constrain_float(steer_angle_output,-500,500);
	//��¼ƫ��ֵ,�����ж�ת��ִ��״̬
	steer_angle_error=steerangle_ctrl.error;
}
void step_motor_ctrl_dianshe(void)//���������������
{
	static float pwm_x,pwm_y;
	static int abs_pwm_x,abs_pwm_y;
	
		const VisionData *vd = uart1_parser_get_data();
		int16_t target_x = vd->valid ? vd->x : 0;
		int16_t target_y = vd->valid ? vd->y : 0;
	step_motor_X.expect=0;
	step_motor_X.feedback=target_x;
	step_motor_X.error=step_motor_X.expect-step_motor_X.feedback;
	pwm_x = pid_control_run(&step_motor_X);
	abs_pwm_x = abs(target_x);
	step_motor_Y.expect=0;
	step_motor_Y.feedback=target_y;
	step_motor_Y.error=step_motor_Y.expect-step_motor_Y.feedback;
	pwm_y = pid_control_run(&step_motor_Y);
	abs_pwm_y = abs(target_y);
	
	
	if(abs_pwm_x<=3.1&&abs_pwm_y<=3.1)//һ����Χ�ھͿ�����
	{
		Razer_ctrl(1);
	}
	
	else 
	{
		Razer_ctrl(0);
	}
	if((abs_pwm_x<=1.1||abs_pwm_y<=1.1))//pid�������С��1Ƶ���޷�����Ϊ0Ƶ�ʣ��������0ռ�ձ�
	{
		if(abs_pwm_x<=1.1&&abs_pwm_y<=1.1){set_duty_X(0.0);set_duty_Y(0.0);}
		else if(abs_pwm_x<=1.1)set_duty_X(0.0);
		else set_duty_Y(0.0);
		return ;//�������������븳ֵƵ��
	}
	
	

	if(pwm_x>=0)
	{
	DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
	set_freq_X(pwm_x);
	set_duty_X(0.5);

	}
	else if(pwm_x<0)
	{
	DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
	set_freq_X(-pwm_x);
	set_duty_X(0.5);
	}


	if(pwm_y>=0)
	{
		DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
		set_freq_Y(pwm_y);
		set_duty_Y(0.5);
	}
	else if(pwm_y<0)
	{
			DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
		set_freq_Y(-pwm_y);
		set_duty_Y(0.5);

	}

}



void step_motor_ctrl_daba(void)//���Ӳ��������
{
			Razer_ctrl(1);
	static float pwm_x,pwm_y;
	static int abs_pwm_x,abs_pwm_y;

	
		const VisionData *vd = uart1_parser_get_data();
		int16_t target_x = vd->valid ? vd->x : 0;
		int16_t target_y = vd->valid ? vd->y : 0;
	step_motor_X.expect=0;
	step_motor_X.feedback=target_x;
	step_motor_X.error=step_motor_X.expect-step_motor_X.feedback;
	pwm_x = pid_control_run(&step_motor_X);
	abs_pwm_x = abs(target_x);
	step_motor_Y.expect=0;
	step_motor_Y.feedback=target_y;
	step_motor_Y.error=step_motor_Y.expect-step_motor_Y.feedback;
	pwm_y = pid_control_run(&step_motor_Y);
	abs_pwm_y = abs(target_y);

	
	if(pwm_x>=0)
	{
	DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
	set_freq_X(pwm_x);
	set_duty_X(0.5);

	}
	else if(pwm_x<0)
	{
	DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_2_PIN);
	set_freq_X(-pwm_x);
	set_duty_X(0.5);
	}


	if(pwm_y>=0)
	{
		DL_GPIO_setPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
		set_freq_Y(pwm_y);
		set_duty_Y(0.5);
	}
	else if(pwm_y<0)
	{
			DL_GPIO_clearPins(STEP_MOTOR_DIR_PORT,STEP_MOTOR_DIR_STEP_MOTOR_DIR_1_PIN);
		set_freq_Y(-pwm_y);
		set_duty_Y(0.5);

	}
}

/***************************************************
������: void steer_gyro_ctrl(void)
˵��:	������ٶȿ���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void steer_gyro_ctrl(void)
{
	steergyro_ctrl.expect=steer_gyro_expect;//����
	steergyro_ctrl.feedback=smartcar_imu.yaw_gyro_enu;//ƫ���Ƕȷ���
	pid_control_run(&steergyro_ctrl);		  //����������
	steer_gyro_output=steergyro_ctrl.output;	
}

/***************************************************
������: void steer_control(float *output)
˵��:	�������
���:	float *output-���������ָ��
����:	��
��ע:	��
����:	��������
****************************************************/
void steer_control(float *output)
{
	controller_output *_flight_output=&trackless_output;
	
	switch(trackless_output.yaw_ctrl_mode)
	{
		case ROTATE:
		{
			if(_flight_output->yaw_outer_control_output==0)//ƫ�����Ƹ�������λ
			{
				if(steer_angle_expect==0)
				{
					steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
				}
				//�Ƕȿ���
				steer_angle_ctrl();
				//���ٶ�����
				steer_gyro_expect=steer_angle_output;				
			}
			else
			{
				steer_angle_expect=0;
				//���ٶ�����
				steer_gyro_expect=_flight_output->yaw_outer_control_output;	
			}			
		}
		break;
		case AZIMUTH://���Ժ���Ƕ�
	  {
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				steer_angle_expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(steer_angle_error)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}			
			//�Ƕȿ���
			steer_angle_ctrl();
			//���ٶ�����
			steer_gyro_expect=steer_angle_output;					
		}
		break;
		case CLOCKWISE://˳ʱ�롪��������Ը���ʱ�̵ĺ���Ƕ�
		{
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=smartcar_imu.rpy_deg[_YAW]-_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				steer_angle_expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(steer_angle_error)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			//�Ƕȿ���
			steer_angle_ctrl();
			//���ٶ�����
			steer_gyro_expect=steer_angle_output;		
		}
		break;		
		case ANTI_CLOCKWISE://��ʱ�롪����Ը���ʱ�̵ĺ���Ƕ�
		{
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=smartcar_imu.rpy_deg[_YAW]+_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				steer_angle_expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(steer_angle_error)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			//�Ƕȿ���
			steer_angle_ctrl();
			//���ٶ�����
			steer_gyro_expect=steer_angle_output;		
		}
		break;
		case CLOCKWISE_TURN://��ĳһ���ٶ�˳ʱ����ת�೤ʱ��
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				//�����ƫ�����ٶȽ�������
				steer_gyro_expect=-_flight_output->yaw_outer_control_output;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//��¼��ʼת����ʱ��				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//ִ����Ϻ�,
				//1����ƫ�����ٶ�������0,
				//2��ֹͣ��ת��������ǰƫ���ǣ���Ҫ�˳�CLOCKWISE_TURNģʽ���Ƕ������Ż���Ч����Ϊ��ģʽû�ж�ƫ���ǶȽ��п���
				steer_gyro_expect=0;
				steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
			}
		}
		break;
		case ANTI_CLOCKWISE_TURN://��ĳһ���ٶ���ʱ����ת�೤ʱ��
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				//�����ƫ�����ٶȽ�������
				steer_gyro_expect=_flight_output->yaw_outer_control_output;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//��¼��ʼת����ʱ��				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//ִ����Ϻ�,
				//1����ƫ�����ٶ�������0,
				//2��ֹͣ��ת��������ǰƫ���ǣ���Ҫ�˳�CLOCKWISE_TURNģʽ���Ƕ������Ż���Ч����Ϊ��ģʽû�ж�ƫ���ǶȽ��п���
				steer_gyro_expect=0;
				steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
			}
		}
		break;
		default:
		{
			steer_gyro_expect=0;
			steer_angle_expect=smartcar_imu.rpy_deg[_YAW];
		}
	}
	//���ٶȿ���
	steer_gyro_ctrl();
	*output=-steer_gyro_output;
}

/***************************************************
������: void get_distance_error(vector2f t)
˵��:	��ȡĿ��λ�ú͵�ǰλ�õ�ƫ��
���:	vector2f t-Ŀ��λ��
����:	��
��ע:	��
����:	��������
****************************************************/
float get_distance_error(vector2f t)
{
	return sqrtf(sq(t.x-smartcar_imu.state_estimation.pos.x)+sq(t.y-smartcar_imu.state_estimation.pos.y));
}

vector2f target={0,0};
/***************************************************
������: void position_control(void)
˵��:	��άĿ��λ�ÿ���
���:	��
����:	��
��ע:	
		y+(��ʼ��ͷ���)
		*
		*
		*
		*
		*
		*
		***********x+(��ʼ��ͷ����)
����:	��������
****************************************************/
void position_control(float fixed_threshold_cm,uint16_t feed_times)//��������̬����Ŀ���,���ٶ�ĩ�˽Ƕ�Լ��
{
	if(ngs_nav_ctrl.update_flag==1)//����վ/ROS���ʹ��ڿ���ָ��
	{
		target.x=ngs_nav_ctrl.x;
		target.y=ngs_nav_ctrl.y;
		
		ngs_nav_ctrl.update_flag=0;
		ngs_nav_ctrl.ctrl_finish_flag=0;
		ngs_nav_ctrl.cnt=0;	
	}
	
	//�������
	float theta=atan2f((target.y-smartcar_imu.state_estimation.pos.y),(target.x-smartcar_imu.state_estimation.pos.x));//Ŀ�꺽��
	//�Ƕȿ���
	azimuth_ctrl.expect=theta*RAD2DEG;
	azimuth_ctrl.feedback=smartcar_imu.rpy_deg[_YAW];
	azimuth_ctrl.error=azimuth_ctrl.expect-azimuth_ctrl.feedback;
	/***********************ƫ����ƫ���+-180����*****************************/
	if(azimuth_ctrl.error<-180) azimuth_ctrl.error=azimuth_ctrl.error+360;
	if(azimuth_ctrl.error> 180) azimuth_ctrl.error=azimuth_ctrl.error-360;				
	azimuth_ctrl.error=constrain_float(azimuth_ctrl.error,-30,30);
	azimuth_ctrl.output=azimuth_ctrl.kp*azimuth_ctrl.error;
	
	//��������������ٶȣ�������ױ�Ħ����Сʱ���´�ֵ���ܼ�С��̼Ƶľ������
	steer_angle_output=constrain_float(azimuth_ctrl.output,-100,100);
	
	steergyro_ctrl.expect=steer_angle_output;//����
	steergyro_ctrl.feedback=smartcar_imu.yaw_gyro_enu;//ƫ���Ƕȷ���
	pid_control_run(&steergyro_ctrl);		  //����������
	steer_gyro_output=steergyro_ctrl.output;

	//�������
	distance_ctrl.expect=0;
	distance_ctrl.feedback=-get_distance_error(target);
	distance_ctrl.error=distance_ctrl.expect-distance_ctrl.feedback;
  distance_ctrl.error=constrain_float(distance_ctrl.error,-50,50);
	distance_ctrl.output=distance_ctrl.kp*distance_ctrl.error;
	distance_ctrl.output=constrain_float(distance_ctrl.output,-20,20);//���ٶ������������ƣ�������̥�������̼����
	
	//�������������־�ж�
	if(ngs_nav_ctrl.cnt<feed_times)//����50ms����
	{
		if(ABS(distance_ctrl.error)<=fixed_threshold_cm) ngs_nav_ctrl.cnt++;
		else ngs_nav_ctrl.cnt/=2;
	}
	else
	{
		ngs_nav_ctrl.ctrl_finish_flag=1;
		ngs_nav_ctrl.cnt=0;
	}
	
	if(ngs_nav_ctrl.ctrl_finish_flag==1)//����Ŀ��λ�ú��ٶ�������0
	{
		steer_gyro_output=0;
		distance_ctrl.output=0;
	}
}


/***************************************************
������: void distance_control(void)
˵��:	�������
���:	�� 
����:	��
��ע:	��
����:	��������
****************************************************/
void distance_control(void)
{
	//�������
	distance_ctrl.feedback=smartcar_imu.state_estimation.distance;
	distance_ctrl.error=distance_ctrl.expect-distance_ctrl.feedback;
  distance_ctrl.error=constrain_float(distance_ctrl.error,-50,50);
	distance_ctrl.output=3.0f*distance_ctrl.error;//
	distance_ctrl.output=constrain_float(distance_ctrl.output,-20,20);//���ٶ������������ƣ�������̥�������̼����
}


void distance_control_with_speed_limit(float speed_limit)
{
	//�������
	distance_ctrl.feedback=smartcar_imu.state_estimation.distance;
	distance_ctrl.error=distance_ctrl.expect-distance_ctrl.feedback;
  distance_ctrl.error=constrain_float(distance_ctrl.error,-100,100);
	distance_ctrl.output=5.0f*distance_ctrl.error;//
	distance_ctrl.output=constrain_float(distance_ctrl.output,-speed_limit,speed_limit);//���ٶ������������ƣ�������̥�������̼����
}






