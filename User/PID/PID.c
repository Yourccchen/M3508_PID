//
// Created by DELL on 2023/8/29.
//
/**
  ******************************************************************************
  * @file	PID.c
  * @author
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "usart.h"
#include "CAN_Recv.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x)

//PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;


/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
        PID_TypeDef * pid,
        PID_ID   id,
        uint16_t maxout,
        uint16_t intergral_limit,
        float deadband,
        uint16_t period,
        int16_t  max_err,
        int16_t  target,

        float 	kp,
        float 	ki,
        float 	kd)
{
    pid->id = id;

    pid->ControlPeriod = period;
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->Max_Err = max_err;
    pid->target = target;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output = 0;
}

/*中途更改参数设定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/


static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;

    pid->lasttime = pid->thistime;
    pid->thistime = HAL_GetTick();
    pid->dtime = pid->thistime-pid->lasttime;
    pid->measure = measure;
    //	pid->target = target;

    pid->last_err  = pid->err;
    pid->last_output = pid->output;

    pid->err = pid->target - pid->measure;

    //是否进入死区
    if((ABS(pid->err) > pid->DeadBand))
    {
        pid->pout = pid->kp * pid->err;
        pid->iout += (pid->ki * pid->err);


        pid->dout =  pid->kd * (pid->err - pid->last_err);

        //积分是否超出限制
        if(pid->iout > pid->IntegralLimit)
            pid->iout = pid->IntegralLimit;
        if(pid->iout < - pid->IntegralLimit)
            pid->iout = - pid->IntegralLimit;

        //pid输出和
        pid->output = pid->pout + pid->iout + pid->dout;


        //pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波
        if(pid->output>pid->MaxOutput)
        {
            pid->output = pid->MaxOutput;
        }
        if(pid->output < -(pid->MaxOutput))
        {
            pid->output = -(pid->MaxOutput);
        }

    }


    return pid->output;
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    pid->f_cal_pid = pid_calculate;
}

PID_TypeDef motor_pid[4];

int32_t set_angle[4] = {0};
int32_t set_speed[4] = {0};

/**
  *@breif      位置环
  *@param      none
  *@retval     none
  */
void Pid_Pos(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    for(int i=0; i<4; i++)
    {
        pid_init(&motor_pid[i]);
        motor_pid[i].f_param_init(&motor_pid[i],PID_Position,16383,5000,10,0,8000,target_vofa, kp_vofa ,ki_vofa ,kd_vofa);
    }
    set_angle[0] = motor1;
    set_angle[1] = motor2;
    set_angle[2] = motor3;
    set_angle[3] = motor4;
    for(int i=0; i<4; i++)
    {
        motor_pid[i].target = set_angle[i];
        motor_pid[i].f_cal_pid(&motor_pid[i],motor_CAN1[i].real_ecd);
    }
    RM_CAN1_Transmit( 	  motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
                          motor_pid[1].output,
                          motor_pid[2].output,
                          motor_pid[3].output);
    HAL_Delay(10);      //PID控制频率100HZ
}

/**
  *@breif      速度环
  *@param      none
  *@retval     none
  */
void Pid_Speed(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    for(int i=0; i<4; i++)
    {
        pid_init(&motor_pid[i]);
        motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16383,5000,10,0,8000,target_vofa, kp_vofa ,ki_vofa ,kd_vofa);
    }
    set_speed[0] = motor1;
    set_speed[1] = motor2;
    set_speed[2] = motor3;
    set_speed[3] = motor4;
    for(int i=0; i<4; i++)
    {
        motor_pid[i].target = set_speed[i];
        motor_pid[i].f_cal_pid(&motor_pid[i],motor_CAN1[i].speed_rpm);
    }
    RM_CAN1_Transmit( 	  motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
                          motor_pid[1].output,
                          motor_pid[2].output,
                          motor_pid[3].output
                 );
    HAL_Delay(10);      //PID控制频率100HZ
}


