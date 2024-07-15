/*
 * fuzzy.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_common_typedef.h"
#include "math.h"
#include "fuzzy.h"
float kp_m=0;
float kd_m=0;
//ʹ�õ�ģ��PID�����м������ⲽ�裬����Ĳ���EΪerr��ECΪerr��΢��
float KP_Fuzzy(float E,float EC)
{

    int rule_p[7][7]=
    {
        { 6 , 5 , 4 , 4 , 3 , 0 , 0},//-36
        { 6 , 4 , 3 , 3 , 2 , 0 , 0},//-24
        { 4 , 3 , 2 , 1 , 0 , 1 , 2},//-12
        { 2 , 1 , 1 , 0 , 1 , 1 , 2},//0
        { 2 , 1 , 0 , 1 , 2 , 3 , 4},//12
        { 0 , 0 , 2 , 3 , 3 , 4 , 6},//24
        { 0 , 1 , 3 , 4 , 4 , 5 , 6},//36
    };//ģ������� P



    uint8 i2;
    /*������P����ֵ������*/
    float EFF[7]={-30,-20,-10,0,10,20,30};        //err�ķ�Χ��-30��30
    /*������D����ֵ������*/
    float DFF[7]={-15,-10,-5,0,5,10,15};
    /*�����U����ֵ������(������������ѡ��ͬ�����ֵ)*/
    float UFF[7];

    for(i2=0;i2<7;i2++)
        UFF[i2]=kp_m/6*i2;


    float U=0;  /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*ƫ��,ƫ��΢���Լ����ֵ��������*/
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*�����ȵ�ȷ��*/
    /*����PD��ָ������ֵ�����Ч������*/
    if(E>EFF[0] && E<EFF[6])
    {
        if(E<=EFF[1])
        {
            Pn=-2;
            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
        }
        else if(E<=EFF[2])
        {
            Pn=-1;
            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
        }
        else if(E<=EFF[3])
        {
            Pn=0;
            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
        }
        else if(E<=EFF[4])
        {
            Pn=1;
            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
        }
        else if(E<=EFF[5])
        {
            Pn=2;
            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
        }
        else if(E<=EFF[6])
        {
            Pn=3;
            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
        }
    }

    else if(E<=EFF[0])
    {
        Pn=-2;
        PF[0]=1;
    }
    else if(E>=EFF[6])
    {
        Pn=3;
        PF[0]=0;
    }

    PF[1]=1-PF[0];


    //�ж�D��������
    if(EC>DFF[0]&&EC<DFF[6])
    {
        if(EC<=DFF[1])
        {
            Dn=-2;
            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
        }
        else if(EC<=DFF[2])
        {
            Dn=-1;
            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
        }
        else if(EC<=DFF[3])
        {
            Dn=0;
            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
        }
        else if(EC<=DFF[4])
        {
            Dn=1;
            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
        }
        else if(EC<=DFF[5])
        {
            Dn=2;
            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
        }
        else if(EC<=DFF[6])
        {
            Dn=3;
            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
        }
    }
    //���ڸ�����������
    else if (EC<=DFF[0])
    {
        Dn=-2;
        DF[0]=1;
    }
    else if(EC>=DFF[6])
    {
        Dn=3;
        DF[0]=0;
    }

    DF[1]=1-DF[0];

    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
    /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
    /*һ�㶼���ĸ�������Ч*/
    Un[0]=rule_p[Pn+2][Dn+2];
    Un[1]=rule_p[Pn+3][Dn+2];
    Un[2]=rule_p[Pn+2][Dn+3];
    Un[3]=rule_p[Pn+3][Dn+3];

    if(PF[0]<=DF[0])    //��С
        UF[0]=PF[0];
    else
        UF[0]=DF[0];
    if(PF[1]<=DF[0])
        UF[1]=PF[1];
    else
        UF[1]=DF[0];
    if(PF[0]<=DF[1])
        UF[2]=PF[0];
    else
        UF[2]=DF[1];
    if(PF[1]<=DF[1])
        UF[3]=PF[1];
    else
        UF[3]=DF[1];
    /*ͬ���������������ֵ���*/
    if(Un[0]==Un[1])
    {
        if(UF[0]>UF[1])
            UF[1]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[2])
    {
        if(UF[0]>UF[2])
            UF[2]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[3])
    {
        if(UF[0]>UF[3])
            UF[3]=0;
        else
            UF[0]=0;
    }
    if(Un[1]==Un[2])
    {
        if(UF[1]>UF[2])
            UF[2]=0;
        else
            UF[1]=0;
    }
    if(Un[1]==Un[3])
    {
        if(UF[1]>UF[3])
            UF[3]=0;
        else
            UF[1]=0;
    }
    if(Un[2]==Un[3])
    {
        if(UF[2]>UF[3])
            UF[3]=0;
        else
            UF[2]=0;
    }
    t1=UF[0]*UFF[Un[0]];
    t2=UF[1]*UFF[Un[1]];
    t3=UF[2]*UFF[Un[2]];
    t4=UF[3]*UFF[Un[3]];
    temp1=t1+t2+t3+t4;
    temp2=UF[0]+UF[1]+UF[2]+UF[3];//ģ�������
    if(temp2!=0)
        U=temp1/temp2;
    else {
        U=0;
    }

    return U;
}



float Kd_Fuzzy(float EC)
{
    float out=0;
    uint8 i=0;
    float degree_left = 0,degree_right = 0;
    uint8 degree_left_index = 0,degree_right_index = 0;
    float DFF[7]={-15,-10,-5,0,5,10,15};                  //ƫ�����Կ��Ը�Ϊ�����ǵ�ƫ��
    float UFF[7];

        int rule_d[7] = { 6 , 5 , 3 , 2 , 3 , 5 , 6};//ģ������� D
    for(i=0;i<7;i++)
            UFF[i]=kd_m/6*i;

    if(EC<DFF[0])
    {
        degree_left = 1;
        degree_right = 0;
        degree_left_index = 0;
    }
    else if (EC>DFF[6]) {
        degree_left = 1;
        degree_right = 0;
        degree_left_index = 6;
    }
    else {
        for(i=0;i<6;i++)
        {
            if(EC>=DFF[i]&&EC<DFF[i+1])
            {
                degree_left = (float)(DFF[i+1] - EC)/(DFF[i+1] - DFF[i]);
                degree_right = 1 - degree_left;
                degree_left_index = i;
                degree_right_index = i+1;
                break;
            }
        }
    }

    out = UFF[rule_d[degree_left_index]]*degree_left+UFF[rule_d[degree_right_index]]*degree_right;

    return out;
}

/*************ʹ�÷���
    float E=0,EC=0;
    E  = err;
    EC = err - err_last;
    float Kp,Kd,output;
    //kp_mΪ��̬��Kp���ֵ��kd_mΪ��̬�����Kdֵ,����ģ��pidʱҲֻ�����������
    kp_m = 11.8 ;
    kd_m = 13.0  ;
    Kp = KP_Fuzzy(E,EC);
    Kd = Kd_Fuzzy(EC) ;
    output = -( Kp*E+Kd*(EC) );
 }*/

