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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "oled.h"
//#define KEY  HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float Kp =330,Ki = 0.05, Kd = 0;                      //pid�����������
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;  //pidֱ������
float decide = 0;                                     //Ԫ���ж�
float last_error = 0, previous_I = 0;             //���ֵ
int sensor[6] = {0, 0, 0, 0, 0,0};                      //6����������ֵ������
static int initial_speed = 430;                  //��ʼ�ٶ�
int flag = 100 ;
int Key_flag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int ABS(int a)
{
    a = a>0?a:(-a);
    return a;
}
void Give_Motor_PWM(int F_MotorL_PWM,int F_MotorR_PWM,int B_MotorL_PWM,int B_MotorR_PWM);//���
void track_control(void);
void stop(void);//ͣ��
int limit_speed(int v);//����
void read_sensor(void);//��ȡ���⴫����


int key = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    key = 1;
}



int dis_flag = 0;
int pass_flag = 0;

int count=0;
void delay_us(uint32_t us)//��Ƶ72M
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
    {
        ;
    }
}

#define Trig_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
#define Trig_L HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)
int distant=100;      //��������
uint32_t measure_Buf[3] = {0};   //��Ŷ�ʱ������ֵ������
uint8_t  measure_Cnt = 0;    //״̬��־λ
uint32_t high_time;   //������ģ�鷵�صĸߵ�ƽʱ��

void SR04_GetData(void)
{
    switch (measure_Cnt) {
    case 0:
        Trig_L;
        delay_us(2);
        Trig_H;
        delay_us(10);
        Trig_L;
        measure_Cnt++;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	//�������벶��
        break;
    case 3:
        high_time = measure_Buf[1]- measure_Buf[0];    //�ߵ�ƽʱ��
        distant=(high_time*0.034)/2;  //��λcm
        if(distant<18)
        {
            dis_flag = 1;
        } else
        {
            dis_flag  =0;
        }
        measure_Cnt = 0;  //��ձ�־λ
        TIM3->CNT=0;     //��ռ�ʱ������
        break;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//
{

    if(TIM3 == htim->Instance)// �жϴ������жϵĶ�ʱ��ΪTIM3
    {
        switch(measure_Cnt) {
        case 1:
            measure_Buf[0] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
            measure_Cnt++;
            break;
        case 2:
            measure_Buf[1] = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//��ȡ��ǰ�Ĳ���ֵ.
            HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_3); //ֹͣ����
            measure_Cnt++;
        }
    }
}





#define  L1       262//�͵���do ��Ƶ��
#define  L2       296//�͵���re ��Ƶ��
#define  L3       330//�͵���mi ��Ƶ��
#define  L4       349//�͵���fa ��Ƶ��
#define  L5       392//�͵���sol ��Ƶ��
#define  L6       440//�͵���la ��Ƶ��
#define  L7       494//�͵���si ��Ƶ��

#define  M1       523//�е���do ��Ƶ��
#define  M2       587//�е���re ��Ƶ��
#define  M3       659//�е���mi ��Ƶ��
#define  M4       699//�е���fa ��Ƶ��
#define  M5       784//�е���sol��Ƶ��
#define  M6       880//�е���la ��Ƶ��
#define  M7       988//�е���si ��Ƶ��

#define  H1       1048//�ߵ���do ��Ƶ��
#define  H2       1176//�ߵ���re ��Ƶ��
#define  H3       1320//�ߵ���mi ��Ƶ��
#define  H4       1480//�ߵ���fa ��Ƶ��
#define  H5       1640//�ߵ���sol��Ƶ��
#define  H6       1760//�ߵ���la ��Ƶ��
#define  H7       1976//�ߵ���si ��Ƶ��
#define  Z0       0


int16_t solitary_brave[]=
{
    M6,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10,	//��������߰���
    H1,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10, 	//���㲻���ģ��
    H1,50,H2,50,H3,50,H2,50,H3,50,H2,50,H3,100,H3,50,
    H3,50,H2,50,H3,100,H5,100,H3,100,Z0,10 //������Ź��������Ͽ�һ��

};



int music_count = 0;
int music_i = 0;
int music_flag = 0;
int music_start_flag = 0;



int fputc(int c,FILE *stream)
{
    HAL_UART_Transmit(&huart3,(unsigned char *)&c,1,1000);
    return 1;
}

uint8_t RX_date;//���ձ���
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3,&RX_date,1);

        music_flag = 0;



    }

}


void read_data()
{
    HAL_Delay(10);
    sensor[0]=HAL_GPIO_ReadPin(L3_GPIO_Port,L3_Pin);
    sensor[1]=HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin);
    sensor[2]=HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin);
    sensor[3]=HAL_GPIO_ReadPin(R1_GPIO_Port,R1_Pin);
    sensor[4]=HAL_GPIO_ReadPin(R2_GPIO_Port,R2_Pin);
    sensor[5]=HAL_GPIO_ReadPin(R3_GPIO_Port,R3_Pin);

}

void avoid()
{
    stop();
    Give_Motor_PWM(-200,-200,-200,-200);
    HAL_Delay(400);
    Give_Motor_PWM(400,-400,400,-400);
    HAL_Delay(300);
    Give_Motor_PWM(480,480,480,480);
    HAL_Delay(420);
    Give_Motor_PWM(-450,450,-450,450);
    HAL_Delay(320);
    Give_Motor_PWM(500,500,500,500);
    HAL_Delay(350);
    Give_Motor_PWM(-600,600,-600,600);
    HAL_Delay(300);
    Give_Motor_PWM(440,440,440,440);
    HAL_Delay(330);
    Give_Motor_PWM(600,-600,600,-600);
    HAL_Delay(300);


}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
    MX_TIM1_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);



    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim2);






    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);

    HAL_UART_Receive_IT(&huart3,&RX_date,1);
    HAL_Delay(2000);

//	OLED_Init();

    char now_flag = 's';
    music_flag = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if(Key_flag)
        {
            stop();
        }
        else
        {
            switch(RX_date)
            {
            case 's'://ͣ��
                if(music_start_flag)
                {
                    music_flag = 1;
                }
                stop();
                now_flag = 's';
                break;
            case 'f'://ǰ��
                Give_Motor_PWM(300,300,300,300);
                break;
            case 'l'://��ת
                Give_Motor_PWM(-300,300,-300,300);
                break;
            case 'r'://��ת
                Give_Motor_PWM(300,-300,300,-300);
                break;
            case 'b'://����
                Give_Motor_PWM(-300,-300,-300,-300);
                break;
            case '2'://ѭ��
                now_flag = '2';
                if(music_start_flag)
                {
                    music_flag = 1;
                }
                if(dis_flag==1)
                {
                    avoid();//����
                }
                else
                {
                    track_control();//ѭ��
                }
                break;        
            case 'm':    //��ʼ��������
                music_flag =1;
                music_start_flag = 1;
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                music_count = 0;
                music_i = 0;
                __HAL_TIM_SET_AUTORELOAD(&htim4,1000000/solitary_brave[music_i*2]-1);
                __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,500000/solitary_brave[music_i*2]);
                RX_date = now_flag;
                break;
            case '4'://��ͣ����
                music_flag = 0;
                music_start_flag = 0;
                HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
                RX_date = now_flag;
                break;
            case '6'://������������
                music_flag = 1;
                music_start_flag = 1;
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                RX_date = now_flag;
                break;
            case '3'://����
                HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
                break;
            case '5'://�ص�
                HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
                break;
            }

        }


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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
int limit_speed(int v)
{
    if(v>999)
    {
        v=950;
    }
    if(v<-950)
    {
        v=-950;
    }

    return v;

}


void stop()
{
    HAL_GPIO_WritePin(F_AIN1_GPIO_Port, F_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_AIN2_GPIO_Port, F_AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_BIN1_GPIO_Port, F_BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_BIN2_GPIO_Port, F_BIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_AIN1_GPIO_Port, B_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_AIN2_GPIO_Port, B_AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_BIN1_GPIO_Port, B_BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_BIN2_GPIO_Port, B_BIN2_Pin, GPIO_PIN_RESET);
}

void Give_Motor_PWM(int F_MotorL_PWM,int F_MotorR_PWM,int B_MotorL_PWM,int B_MotorR_PWM)
{
    F_MotorL_PWM=limit_speed(F_MotorL_PWM);
    F_MotorR_PWM=limit_speed(F_MotorR_PWM);
    B_MotorL_PWM=limit_speed(B_MotorL_PWM);
    B_MotorR_PWM=limit_speed(B_MotorR_PWM);
    if (F_MotorL_PWM>0) //��ǰ�����ת
    {
        HAL_GPIO_WritePin(F_AIN1_GPIO_Port, F_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(F_AIN2_GPIO_Port, F_AIN2_Pin, GPIO_PIN_RESET);
    }
    else              //��ǰ�����ת
    {
        HAL_GPIO_WritePin(F_AIN1_GPIO_Port, F_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(F_AIN2_GPIO_Port, F_AIN2_Pin, GPIO_PIN_SET);
    }

    if (F_MotorR_PWM>0) //��ǰ�����ת
    {
        HAL_GPIO_WritePin(F_BIN1_GPIO_Port, F_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(F_BIN2_GPIO_Port, F_BIN2_Pin, GPIO_PIN_RESET);
    }
    else              //��ǰ�����ת
    {
        HAL_GPIO_WritePin(F_BIN1_GPIO_Port, F_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(F_BIN2_GPIO_Port, F_BIN2_Pin, GPIO_PIN_SET);
    }
    if (B_MotorL_PWM>0) //�������ת
    {
        HAL_GPIO_WritePin(B_AIN1_GPIO_Port, B_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B_AIN2_GPIO_Port, B_AIN2_Pin, GPIO_PIN_RESET);
    }
    else              //�������ת
    {
        HAL_GPIO_WritePin(B_AIN1_GPIO_Port, B_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_AIN2_GPIO_Port, B_AIN2_Pin, GPIO_PIN_SET);
    }

    if (B_MotorR_PWM>0) //�Һ�����ת
    {
        HAL_GPIO_WritePin(B_BIN1_GPIO_Port, B_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B_BIN2_GPIO_Port, B_BIN2_Pin, GPIO_PIN_RESET);
    }
    else              //�Һ�����ת
    {
        HAL_GPIO_WritePin(B_BIN1_GPIO_Port, B_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B_BIN2_GPIO_Port, B_BIN2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, ABS(F_MotorL_PWM));
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, ABS(F_MotorR_PWM));
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, ABS(B_MotorL_PWM));
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, ABS(B_MotorR_PWM));

}

float get_error()
{
    int p,q;
    sensor[0]=HAL_GPIO_ReadPin(L3_GPIO_Port,L3_Pin);
    sensor[1]=HAL_GPIO_ReadPin(L2_GPIO_Port,L2_Pin);
    sensor[2]=HAL_GPIO_ReadPin(L1_GPIO_Port,L1_Pin);
    sensor[3]=HAL_GPIO_ReadPin(R1_GPIO_Port,R1_Pin);
    sensor[4]=HAL_GPIO_ReadPin(R2_GPIO_Port,R2_Pin);
    sensor[5]=HAL_GPIO_ReadPin(R3_GPIO_Port,R3_Pin);

    p=-4*sensor[0]+(-2.5)*sensor[1]+(-1.5)*sensor[2]+1.5*sensor[3]+2.5*sensor[4]+4*sensor[5];//��ֵ
    q = sensor[0]+sensor[1]+sensor[2]+sensor[3]+sensor[4]+sensor[5];//ʶ�𵽺��ߵĴ�����������
    if(q!=0)
    {
        p=p/q;
        return p;
    }
    else
    {
        return 0;
    }

}





void track_control(void)
{
    int L_speed ;
    int R_speed ;

    if(sensor[0]==1&&sensor[2]==1&&sensor[3]==0&&sensor[4]==0&&sensor[5]==0)//����ֱ��
    {
        stop();
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
        L_speed = -550;
        R_speed = -550;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);
        HAL_Delay(35);

        L_speed =-650;
        R_speed = 650;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);

        while(!(sensor[2]==1||sensor[3]==1||sensor[4]==1||sensor[5]==1))
        {

            read_data();
            HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
    }

    else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==1&&sensor[5]==1)
    {


        stop();
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
        L_speed = -550;
        R_speed = -550;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);
        HAL_Delay(35);


        L_speed = 650;
        R_speed =-650;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);


        while(!(sensor[0]==1||sensor[1]==1||sensor[2]==1||sensor[3]==1))
        {
            read_data();
            HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        }

        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);


    }
    else if(sensor[0]==1&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0&&sensor[5]==0)
    {

        stop();
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
        L_speed = -500;
        R_speed = -500;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);
        HAL_Delay(50);

        L_speed = -700;
        R_speed = 700;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);

        while(!(sensor[1]==1||sensor[2]==1||sensor[3]==1||sensor[4]==1||sensor[5]==1))
        {
            read_data();
            HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        }
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);



    }

    else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0&&sensor[5]==1)
    {
        stop();
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
        L_speed = -500;
        R_speed = -500;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);
        HAL_Delay(50);
        L_speed = 700;
        R_speed = -700;
        Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);


        while(!(sensor[0]==1||sensor[1]==1||sensor[2]==1||sensor[3]==1||sensor[4]==1))
        {
            read_data();
            HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
        }



    }


    if(L_speed - R_speed >400)
    {
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
    } else if(R_speed - L_speed >400)
    {
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);

    } else {
        HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
    }


    L_speed = initial_speed + PID_value;
    R_speed = initial_speed - PID_value;

    Give_Motor_PWM(L_speed,R_speed,L_speed,R_speed);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        //������ģ�����
        count++;
        if(count ==10)
        {
            count = 0;
            SR04_GetData();

        }

        //ѭ��������
        error =get_error();

        P = error;
        I = I + error;
        D = error - last_error;

        PID_value = (Kp * P) + (Ki * I) + (Kd * D);

        last_error = error;

        //���������ֿ���
        if(music_flag == 1)
        {
            music_count ++;

            if(music_count == solitary_brave[music_i*2+1])
            {
                music_count=0;
                music_i++;

                if(music_i == 30)
                {
                    music_i = 0;

                }
                __HAL_TIM_SET_AUTORELOAD(&htim4,1000000/solitary_brave[music_i*2]-1);
                __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,500000/solitary_brave[music_i*2]);

            }
        }


    }

    if(key)//��������
    {
        key =0 ;
        if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==0)
        {
            Key_flag=!Key_flag;
            if(Key_flag)
            {

                HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(L_LED_GPIO_Port, L_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(R_LED_GPIO_Port,R_LED_Pin, GPIO_PIN_RESET);
            }
        }

    }


}







/* USER CODE END 0 */


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
