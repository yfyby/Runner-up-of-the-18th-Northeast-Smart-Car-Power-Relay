/*
 * inductor.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "inductor.h"
#include "zf_driver_adc.h"
#include "some_algorithm.h"
#include "math.h"
#include "system.h"
int16 Inductor_Value[4] = {100, 100, 100, 100};//ˮƽ ��ֱ ˮƽ  ��ֱ  ˮƽ
int16 Inductor_Threshold[4] = {223, 223, 223, 223};  //�����ֵ�ɼ�
int16 Inductor_Value_Temp[4][3] = {0};//ˮƽ ��ֱ ˮƽ  ��ֱ  ˮƽ
float Level_Deviation=0;               //ˮƽ������
float Vert_Deviation=0;               //��ֱ������
int16 Left_1=0,Left_2=0,Right_1=0,Right_2=0;   //5�����ֵ
uint8 inductor_check=1;      //������Ƿ�������־λ
uint8 inductor_check_count=1; //����������
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ѹADC��ʼ��
//-------------------------------------------------------------------------------------------------------------------
void Init_Voltage_AD()
{
    adc_init(ADC1_IN15_C5,ADC_8BIT);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ѹADC�ɼ�
//-------------------------------------------------------------------------------------------------------------------
float Get_Voltage_value()
{
    float value;
    value=adc_convert(ADC1_IN15_C5);
    return value;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ADC��ʼ��
//-------------------------------------------------------------------------------------------------------------------
void Init_Magenet_AD()                //��г�ʼ��
{
      adc_init(ADC1_IN10_C0,ADC_8BIT);//S2
      adc_init(ADC1_IN11_C1,ADC_8BIT);//S3
      adc_init(ADC1_IN12_C2,ADC_8BIT);//S4
      adc_init(ADC1_IN13_C3,ADC_8BIT);//S5
}
/***************************************************
*                   adc�ɼ����ֵ
*  ���ͣ�����
*  ��ڲ���������ֵ������Inductor_Value
*  ����ֵ����
*  ���ܣ���ֵ
***************************************************/
void Get_InductValue(int16 value[])
{
    uint8 i;
    for (i = 0; i < 3; i++)
    {
        Inductor_Value_Temp[0][i] = adc_convert(ADC1_IN10_C0);
        Inductor_Value_Temp[1][i] = adc_convert(ADC1_IN11_C1);
        Inductor_Value_Temp[2][i] = adc_convert(ADC1_IN12_C2);
        Inductor_Value_Temp[3][i] = adc_convert(ADC1_IN13_C3);
    }
    //��ֵ
    for (i = 0; i < 4; i++)
        value[i] = (Inductor_Value_Temp[i][0]+Inductor_Value_Temp[i][1] + Inductor_Value_Temp[i][2]) / 3.0;
}
/***************************************************
*                   �����ֵ�Ƿ��쳣
*  ���ͣ�����
*  ����ֵ���쳣 0 ���� 1
***************************************************/
uint8 Is_InductorValue_Normal()
{
    int16 sum_val = 0;
    sum_val =Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3];
    if (sum_val > 3)            //(��Ҫ����)
    {
        return 1;
    }
    else
        return 0;
}

/***************************************************
*                 ���ֵ�˲���
*  ���ͣ�����
*  ��ڲ��������ֵ����Inductor_Value[5],ȡq=10.0f,r=10.0f
*  ����ֵ����
*  ���ܣ��˲�
***************************************************/
void InductorVal_Filter_Kalman(int16 value[],float q,float r)  //(��Ҫ����)q,rֵ
{
    Para_Type type;
    type = Inductor_Hori_Left;
    value[0] = First_Order_KalmanFilter(value[0], type, q, r);
    type = Inductor_Vert_Left;
    value[1] = First_Order_KalmanFilter(value[1], type, q, r);
    type = Inductor_Vert_Right;
    value[2] = First_Order_KalmanFilter(value[2], type, q, r);
    type = Inductor_Hori_Right;
    value[3] = First_Order_KalmanFilter(value[3], type, q, r);
}
//==============================================================================
//  @brief      ���ֵ��һ��
//  @return     void
//==============================================================================
void Normalize_Induct()
{
    Inductor_Value[0]  = (Inductor_Value[0]) * 100 / (Inductor_Threshold[0]);     // ��ƫ������һ����0--100����
    Inductor_Value[1]  = (Inductor_Value[1]) * 100 / (Inductor_Threshold[1]);
    Inductor_Value[2] = (Inductor_Value[2]) * 100 / (Inductor_Threshold[2]);
    Inductor_Value[3] = (Inductor_Value[3]) * 100 / (Inductor_Threshold[3]);
    if (Inductor_Value[0] > 100)   Inductor_Value[0] = 100;
    if (Inductor_Value[1] > 100)   Inductor_Value[1] = 100;
    if (Inductor_Value[2] > 100)   Inductor_Value[2] = 100;
    if (Inductor_Value[3] > 100)   Inductor_Value[3] = 100;
}

//==============================================================================
//  @brief      ˮƽ���ƫ��
//==============================================================================
void Calculate_Deviation()
{
    if((Inductor_Value[0] - Inductor_Value[3])!=0)                       //��ֹ����Ϊ0
    {
    Level_Deviation=((sqrt(Inductor_Value[0]) - sqrt(Inductor_Value[3]))/(Inductor_Value[0] + Inductor_Value[3]))*20.0;
    Level_Deviation=LIMIT(Level_Deviation,2.1,-2.1);
    }else
    {
        Level_Deviation=0;
    }
}

//==============================================================================
//  @brief      ��ֱ�����ƫ��
//==============================================================================
void Calculate_Vert_Deviation()
{
    if((Inductor_Value[1] - Inductor_Value[2])!=0)                       //��ֹ����Ϊ0
    {
    Vert_Deviation=((sqrt(Inductor_Value[1]) - sqrt(Inductor_Value[2]))/(Inductor_Value[1] + Inductor_Value[2]))*20.0;//��ֱ��еĲ�Ⱥ���Ϊƫ��
    Vert_Deviation=LIMIT(Vert_Deviation,2.1,-2.1);
    }else
    {
        Vert_Deviation=0;
    }
}

 /***************************************************
*                   ������ֵб��
*  ���ͣ�����
*  ��ڲ��������ֵ
*  ����ֵ��б��
************************************************/
float Cal_Mid_Val_Slope(int16 mid_val)
{

    static float Record[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8 Cnt = 0, Time_ms = 0;
    static float Slope = 0.0f;
    uint8 i = 0;

    Time_ms++;

    if (Time_ms >= 1)
    {
        Time_ms = 0;                      //��������ʼ���÷�ֹ���ݲ���
        Cnt++;
        //ѭ����¼���ֵ
        for (i = 0; i < 14; i++)
            Record[i] = Record[i + 1];
        Record[14] = (float)mid_val;

        if (Cnt < 15)
            Slope = 0.0f;
        else
        {
            Cnt = 15;
            Slope = Slope_Calculate(0, 14, Record);
        }
    }

    return Slope;
}
//==============================================================================
//  @brief      ƫ���˲���
//  @param      ʵʱƫ��ֵ
//  @return     �˲����ƫ��
//  @since      v1.0
//  @author
//==============================================================================
float Position_Filter(Fitter_err *para,float err)
{
    float para_filtered ;

    para->fitter_value[3] = para->fitter_value[2];
    para->fitter_value[2] = para->fitter_value[1];
    para->fitter_value[1] = para->fitter_value[0];
    para->fitter_value[0] = err;

    para_filtered = (4.0 * para->fitter_value[0] + 3.0 * para->fitter_value[1]
                     + 2.0 * para->fitter_value[2] + 1.0 * para->fitter_value[3]) / 10.0;
    return para_filtered;
}
/***************************************************
*                 ƫ���¼
*  ���ͣ�����
*  ��ڲ�����ƫ���¼���� ʵʱƫ��ֵ,�����С
*  ����ֵ����
*  ���ܣ���¼������n��ƫ��
***************************************************/
void Position_Record(float queue[], float para, uint8 n)
{
    uint8 i = 0;

    for (i = n-1; i > 0; i--)
      queue[i] = queue[i - 1];
    queue[0] = para;
}
/***************************************************
*                 ��е��ú���
*  ���ͣ�����
*  ��ڲ�������
*  ����ֵ����
*  ���ܣ��ɼ����ֵ,�������
***************************************************/
void Get_ADC()
{
      Get_InductValue(Inductor_Value);                   //��ȡ��ʼ���ֵ
      InductorVal_Filter_Kalman(Inductor_Value,10.0,10.0);//���ֵ����һ�׿������˲�
      Normalize_Induct();                              //���ֵ��һ��
      Calculate_Deviation();
      if(Distance>1)
      inductor_check=Is_InductorValue_Normal();
}



