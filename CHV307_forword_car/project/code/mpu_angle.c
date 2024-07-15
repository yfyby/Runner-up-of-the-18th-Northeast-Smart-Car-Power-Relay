/*
 * mpu_angle.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "mpu_angle.h"
#include "math.h"
#include "zf_device_icm20602.h"
#include "zf_driver_delay.h"
/***************************************************
*  ��̬�����ļ�
*  author: USTH_б������
***************************************************/
float temp_gyrox,temp_gyroy,temp_gyroz;     //���ٶȴ�����ԭʼ����
float gyrox,gyroy,gyroz;    //������ԭʼ����
uint8 flag_offset=0;     //���ٶȵ�0��ɱ�־λ

extern float Accel_x;        //X����ٶ�
extern float Accel_y;       //Y����ٶ�
extern float Accel_z;      //Z����ٶ�
extern float Gyro_x;      //X��������
extern float Gyro_y;     //Y��������
extern float Gyro_z;    //Z��������

float Angle_x_temp=0;       //�ɼ��ٶȼ����x��б�Ƕ�
float Angle_y_temp=0;       //�ɼ��ٶȼ����y��б�Ƕ�
float Angle_X_Final=0;      //X������б�Ƕ�//������
float Angle_Y_Final=0;      //Y������б�Ƕ�//�����
float Turn_speed=0;       //���ٶ�,�������ٲ�ʹ�ã��������
float  Gyro_Y_Offset = 0, Gyro_X_Offset = 0, Gyro_Z_Offset=0;    //��̬��λֵ,�����Ǿ����ۼ����,��Ҫ����ʱ����
//�Ƕȼ���
void Angle_Calcu(void)
{
    //float accx,accy,accz;    //������Ǽ��ٶ�ֵ
    static float ratio = 0.048f;    //���ϵ��
    static float temp_turn_speed[3] = {0.0f, 0.0f, 0.0f};

    icm20602_get_acc();    //��ȡ�������ֵ
    icm20602_get_gyro();   //��ȡ����������ֵ
//    accx=icm20602_acc_transition(Accel_x);
//    accy=icm20602_acc_transition(Accel_y);
//    accz=icm20602_acc_transition(Accel_z);
//    //���ٶȷ����й�ʽ�����������ˮƽ������ϵ֮��ļн�
//    Angle_x_temp=(atan(accy/accz))*57.32;//*180/3.14
//    Angle_y_temp=(atan(accx/accz))*57.32;//*180/3.14
    if(flag_offset==1){    //���ٶȵ�0���
    temp_gyrox=icm20602_gyro_transition(Gyro_x)-Gyro_X_Offset;
    temp_gyroy=icm20602_gyro_transition(Gyro_y)-Gyro_Y_Offset;
    temp_gyroz=icm20602_gyro_transition(Gyro_z)-Gyro_Z_Offset;
    }else{
    temp_gyrox=icm20602_gyro_transition(Gyro_x);
    temp_gyroy=icm20602_gyro_transition(Gyro_y);
    temp_gyroz=icm20602_gyro_transition(Gyro_z);
    }
    gyrox=Mpu_Filter_GyroX(temp_gyrox);
    gyroy=Mpu_Filter_GyroY(temp_gyroy);
    gyroz=Mpu_Filter_GyroZ(temp_gyroz);
    Turn_speed = (gyroz) * ratio;    //Gryo_Y��Ϲ�ʽ
    //��ת����ٶȽ�����ֵ�˲�
    temp_turn_speed[2] = temp_turn_speed[1];
    temp_turn_speed[1] = temp_turn_speed[0];
    temp_turn_speed[0] = Turn_speed;
    if (temp_turn_speed[0] > temp_turn_speed[1])
    {
        if (temp_turn_speed[1] > temp_turn_speed[2])
            Turn_speed = temp_turn_speed[1];
        else if (temp_turn_speed[1] < temp_turn_speed[2] && temp_turn_speed[2] < temp_turn_speed[0])
            Turn_speed = temp_turn_speed[2];
        else
            Turn_speed = temp_turn_speed[0];
    }
    else
    {
        if (temp_turn_speed[2] > temp_turn_speed[1])
            Turn_speed = temp_turn_speed[1];
        else if (temp_turn_speed[1] > temp_turn_speed[2] && temp_turn_speed[2] > temp_turn_speed[0])
            Turn_speed = temp_turn_speed[2];
        else
            Turn_speed = temp_turn_speed[0];
    }
  //Kalman_Filter_X(Angle_x_temp,gyrox);  //�������˲�����X���,������
  //Kalman_Filter_Y(Angle_y_temp,gyroy);  //�������˲�����Y���,�����
}
/***************************************************
*                 ��̬�˲��� - Gyro_Z
*  ���ͣ�˽��
*  ����ֵ��int16_�˲����Gyro_Xֵ
*  ���ܣ��˲�
***************************************************/
static float Mpu_Filter_GyroZ(float para)
{
    int32 para_filitered = 0;
    static float Filter_GyroZ[3] = {0, 0, 0};

    Filter_GyroZ[2] = Filter_GyroZ[1];
    Filter_GyroZ[1] = Filter_GyroZ[0];
    Filter_GyroZ[0] = para;

    para_filitered = (Filter_GyroZ[0] * 6 + Filter_GyroZ[1] * 3 + Filter_GyroZ[2] * 1) / 10;

    return para_filitered;
}

/***************************************************
*                 ��̬�˲��� - Gyro_X
*  ���ͣ�˽��
*  ����ֵ��int16_�˲����Gyro_Xֵ
*  ���ܣ��˲�
***************************************************/
static float Mpu_Filter_GyroX(float para)
{
    float para_filitered = 0;
    static float Filter_GyroX[3] = {0, 0, 0};

    Filter_GyroX[2] = Filter_GyroX[1];
    Filter_GyroX[1] = Filter_GyroX[0];
    Filter_GyroX[0] = para;

    para_filitered = (Filter_GyroX[0] * 6 + Filter_GyroX[1] * 3 + Filter_GyroX[2] * 1) / 10;

    return para_filitered;
}

/***************************************************
*                 ��̬�˲��� - Gyro_Y
*  ���ͣ�˽��
*  ����ֵ��int16_�˲����Gyro_Xֵ
*  ���ܣ��˲�
***************************************************/
static float Mpu_Filter_GyroY(float para)
{
    float para_filitered = 0;
    static float Filter_GyroY[3] = {0, 0, 0};

    Filter_GyroY[2] = Filter_GyroY[1];
    Filter_GyroY[1] = Filter_GyroY[0];
    Filter_GyroY[0] = para;

    para_filitered = (Filter_GyroY[0] * 6 + Filter_GyroY[1] * 3 + Filter_GyroY[2] * 1) / 10;

    return para_filitered;
}
/***************************************************
*                ������������ʼֵ
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�����ǰ���㣬�����������Է�������һ��ʼ�����0Ư��ͨ������ɼ�һ��ʱ��0Ư�ټ�ȥ����
*  ʹ�ã���ʼ��ʱ���е���
***************************************************/
void gyroOffset_init()
{
    float gyro_y_offset_sum = 0, gyro_x_offset_sum = 0,gyro_z_offset_sum=0;
    for (uint16_t i = 0; i < 100; ++i) {
        icm20602_get_gyro();
        gyro_x_offset_sum  +=icm20602_gyro_transition(Gyro_x);
        gyro_y_offset_sum  +=icm20602_gyro_transition(Gyro_y);
        gyro_z_offset_sum  +=icm20602_gyro_transition(Gyro_z);
        system_delay_ms(10);
    }
    Gyro_Z_Offset = (float)(gyro_z_offset_sum / 100.0);
    Gyro_Y_Offset = (float)(gyro_y_offset_sum / 100.0);
    Gyro_X_Offset = (float)(gyro_x_offset_sum / 100.0);
    flag_offset   = 1;//��ɵ���
}
/***************************************************
*               ����ƫ���Ƕ�
*  ���ͣ�����
*  ��ڲ��������ٶ�
*  ����ֵ��ƫ����
*  ���ܣ�����ƫ���Ƕ�
***************************************************/
float Calculate_Turn_Angle(float turn_speed)
{
    static float Angle_Car = 0.0f;
    Angle_Car += turn_speed * 0.02; //20ms�Ĳɼ�ʱ��
    return Angle_Car;
}

//����������
float Q_angle = 0.001;      //�Ƕ��������Ŷȣ��Ƕ�������Э����
float Q_gyro  = 0.003;      //���ٶ��������Ŷȣ����ٶ�������Э����
float R_angle = 0.5;        //���ٶȼƲ���������Э����
float dt      = 0.02;       //�˲��㷨�������ڣ��ɶ�ʱ����ʱ20ms
char  C_0     = 1;          //H����ֵ
float Q_bias, Angle_err;    //Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ��
float PCt_0, PCt_1, E;      //����Ĺ�����
float K_0, K_1, t_0, t_1;   //����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ�� t_0/1:�м����
float P[4] ={0,0,0,0};  //����Э��������΢�־����м����
float PP[2][2] = { { 1, 0 },{ 0, 1 } };//����Э�������P

void Kalman_Filter_X(float Accel,float Gyro) //����������
{
    //����һ���������
    //��ʽ��X(k|k-1) = AX(k-1|k-1) + BU(k)
    //X = (Angle,Q_bias)
    //A(1,1) = 1,A(1,2) = -dt
    //A(2,1) = 0,A(2,2) = 1
    Angle_X_Final += (Gyro - Q_bias) * dt; //״̬����,�Ƕ�ֵ�����ϴ����ŽǶȼӽ��ٶȼ���Ư�����

    //��������������Э��������΢�־���
    //��ʽ��P(k|k-1)=AP(k-1|k-1)A^T + Q
    //Q(1,1) = cov(Angle,Angle) Q(1,2) = cov(Q_bias,Angle)
    //Q(2,1) = cov(Angle,Q_bias)    Q(2,2) = cov(Q_bias,Q_bias)
    P[0]= Q_angle - PP[0][1] - PP[1][0];
    P[1]= -PP[1][1];// ����������Э����
    P[2]= -PP[1][1];
    P[3]= Q_gyro;


    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_X_Final;  //Z(k)������� ����Ƕ�ƫ��

    //�����������㿨��������
    //��ʽ��Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
    //Kg = (K_0,K_1) ��ӦAngle,Q_bias����
    //H = (1,0) ����z=HX+v���z:Accel
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    //�����ģ�����������Э����
    //��ʽ��P(k|k)=(I-Kg(k)H)P(k|k-1)
    //Ҳ��дΪ��P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    //�����壬�������Ž��ٶ�ֵ
    //��ʽ��X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
    Angle_X_Final += K_0 * Angle_err;    //������ƣ��������Ź���ֵ
    Q_bias        += K_1 * Angle_err;    //������ƣ��������Ź���ֵƫ��
    Gyro_x         = Gyro - Q_bias;
}

void Kalman_Filter_Y(float Accel,float Gyro)
{
    Angle_Y_Final += (Gyro - Q_bias) * dt;
    P[0]=Q_angle - PP[0][1] - PP[1][0];
    P[1]=-PP[1][1];
    P[2]=-PP[1][1];
    P[3]=Q_gyro;
    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_Y_Final;
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
    Angle_Y_Final   += K_0 * Angle_err;
    Q_bias  += K_1 * Angle_err;
    Gyro_y   = Gyro - Q_bias;
}



