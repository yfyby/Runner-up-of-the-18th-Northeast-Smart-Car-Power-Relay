/*
 * image_deal.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "image_deal.h"
#include "zf_device_mt9v03x_dvp.h"
#include "math.h"
#include "system.h"
/***************************************************
��̬ѧ������������ȥ���ڰ�
***************************************************/
void morph_open(uint8(*image)[W]) {
    uint8 i, j;
    uint8 temp[H][W];
    int16 sum = 0;
    int k, l;
    int SE[3][3] = {{1, 1, 1},
                    {1, 1, 1},
                    {1, 1, 1}};  //������ʴ�����͵�3x3�ṹԪ��
    //��ʴ����
    for (i = 1; i < H-1; i++) {
        for (j = 1; j < W-1; j++) {
                      sum=0;
            for (k = -1; k <= 1; k++) {
                for (l = -1; l <= 1; l++) {
                    sum += image[i+k][j+l] * SE[1+k][1+l];
                }
            }
            if (sum == 2295) {
                temp[i][j] = 255;
            } else {
                temp[i][j] = 0;
            }
        }
    }
    // ���Ͳ���
    for (i = 1; i < H-1; i++) {
        for (j = 1; j < W-1; j++) {
            sum = 0;
            for (k = -1; k <= 1; k++) {
                for (l = -1; l <= 1; l++) {
                    sum += temp[i+k][j+l] * SE[1+k][1+l];
                }
            }
            if (sum > 0) {
                image[i][j] = 255;
            } else {
                image[i][j] = 0;
            }
        }
    }
}
/***************************************************
��̬ѧ�����ز���
***************************************************/
void dilate(uint8(*image)[W]) {
  uint8 temp[H][W];
  int SE[3][3] = {{1, 1, 1},
                  {1, 1, 1},
                  {1, 1, 1}};
  int i, j, k, l;
    int maxval = 0,val=0,minval=0;
  for (i = 1; i < H-1; i++) {
    for (j = 1; j < W-1; j++) {
      maxval = 0;
      for (k = -1; k <= 1; k++) {
        for (l = -1; l <= 1; l++) {
          val = image[i+k][j+l] + SE[1+k][1+l];
          if (val > maxval) {
            maxval = val;
          }
        }
      }
      temp[i][j] = (maxval > 0) ? 255 : 0;
    }
  }
  for (i = 1; i < H-1; i++) {
    for (j = 1; j < W-1; j++) {
      minval = 255;
      for (k = -1; k <= 1; k++) {
        for (l = -1; l <= 1; l++) {
          val = temp[i+k][j+l] - SE[1+k][1+l];
          if (val < minval) {
            minval = val;
          }
        }
      }
      image[i][j] = (minval < 255) ? 0 : 255;
    }
  }
}

/***************************************************
����ͼ������
***************************************************/
void Copy_image(uint8(*in_IMG)[W], uint8(*out_IMG)[W]){
   uint8 i,j;
     for (i = 0; i < 60; i++)
    {
        for (j = 0; j <94; j++)
        {
            out_IMG[i][j] = in_IMG[i][j];
        }
    }
}

/***************************************************
ԭʼͼ������һ��
 ***************************************************/
void Get_Use_Image(uint8(*in_IMG)[Width], uint8(*out_IMG)[W])
{
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < 120; i += 2)
    {
        for (j = 0; j < 188; j += 2)
        {
            out_IMG[row][line] = in_IMG[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

/***************************************************
 * ͼ��ü�
 * 188*120->160*120
 ***************************************************/
void Get_cut_image(uint8** image, uint8** out_image,uint8 height, uint8 width)
{
    uint8 i=0,j=0;
    for(i=0;i<height;i++)
    {
        for(j=0;j<width;j++)
        {
            out_image[i][j]=image[i][j];
        }
    }
}
/*************************************************************************
 *  �������ƣ�my_adapt_threshold(uint8 *image, uint16 col, uint16 row))
 *  ����˵�����Ż��������ֵ��С
 *  ����˵����tmImage �� ͼ������
 *  �������أ���
 *  �޸�ʱ�䣺2023��3��15��
 *  ��    ע��  my_adapt_threshold(image,94, 60));//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
        �ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
        ������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
*************************************************************************/
uint8 my_adapt_threshold(uint8(*tmImage)[Width],uint8(*out_tmImage)[W], uint16 col, uint16 row)                     //ע�������ֵ��һ��Ҫ��ԭͼ��
{
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = tmImage;  //ָ���������ݵ�ָ��
    uint8* out_data = out_tmImage;  //ָ���������ݵ�ָ��
    short  Row = 0, line = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = 0; i < Hight; i+=2)
    {
        for (j = 0; j < Width; j+=2)
        {
            pixelCount[(int)data[i * Width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * Width + j];       //�Ҷ�ֵ�ܺ�
            out_data[Row * width + line] = data[i * Width + j];
            line++;
        }
                line = 0;
                Row++;
    }
    //����ÿ������ֵ�ĵ�������ͼ���еı���
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //�����Ҷȼ�[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
                w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
                u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

               w1=1-w0;
               u1tmp=gray_sum/pixelSum-u0tmp;

                u0 = u0tmp / w0;              //����ƽ���Ҷ�
                u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
                u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
                deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
                if (deltaTmp > deltaMax)
                {
                    deltaMax = deltaTmp;
                    threshold = j;
                }
                if (deltaTmp < deltaMax)
                {
                break;
                }
         }
    return threshold;
}
/*************************************************************************
 *  �������ƣ�my_adapt_part_threshold(uint8 *image, uint16 col, uint16 row))
 *  ����˵�����ֲ�ͼ����
 *  ����˵����tmImage �� ͼ������
 *  �������أ���
 *  �޸�ʱ�䣺2023��3��15��
 *  ��    ע��  my_adapt_part_threshold(image,94, 60));//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
        �ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
        ������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
*************************************************************************/
uint8 my_adapt_part_threshold(uint8(*tmImage)[W], uint16 col, uint16 row,uint16 begin_col, uint16 begin_row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
    uint16 width = col;
    uint16 height = row;
    uint16 begin_width=begin_col;
    uint16 begin_height=begin_row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = (width-begin_width) * (height-begin_height);
    uint8 threshold = 0;
    uint8* data = tmImage;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = begin_height; i < height; i++)
    {
        for (j = begin_width; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }

    //����ÿ������ֵ�ĵ�������ͼ���еı���

    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //�����Ҷȼ�[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;


        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {

                w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
                u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

               w1=1-w0;
               u1tmp=gray_sum/pixelSum-u0tmp;

                u0 = u0tmp / w0;              //����ƽ���Ҷ�
                u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
                u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
                deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
                if (deltaTmp > deltaMax)
                {
                    deltaMax = deltaTmp;
                    threshold = j;
                }
                if (deltaTmp < deltaMax)
                {
                break;
                }

         }

    return threshold;

}

// ����Ӧ��ֵ��(ƽ����)
// img0 ����ͼ��
// img1 ���ͼ��
// col ��
// row ��
// block_size ������Χsize*size������
// down        �����ָ�Ч��
void adaptive_threshold(uint8(*img0)[W],uint8(*img1)[W], uint16 col, uint16 row, int block_size, int down_value)
{
    int16 half = block_size >> 1;
    int16 pointi = 0, pointj = 0;
    int16 size = block_size * block_size;
    int16 imgstep = 0;
    uint16 width = col;
    uint16 height = row;
    uint8 thres=0;
    uint8* data0 = img0;  //ָ���������ݵ�ָ��
    uint8* data1 = img1;  //ָ���������ݵ�ָ��
    for (uint8 i = 0; i <height; i++)
    {
        for (uint8 j = 0; j <width; j++)
        {
            thres = 0;
            for (int16 di = -half; di <= half; di++)
            {
                for (int16 dj = -half; dj <= half; dj++)
                {
                    pointi = i + di;
                    pointj = j + dj;
                    if ((pointi < 0) || (pointj < 0) || (pointi >=height) || (pointj >=width))
                        continue;

                    thres +=data0[width * pointi + pointj];
                }
            }

            thres /= size;
            thres -= down_value;
            imgstep = width * i + j;
            data1[imgstep] = (data0[imgstep] < thres) ? 0 : 255;
        }
    }
}


// ����Ӧ��ֵ��,ʹ������Сһ���ͼ����
// img0 ����ͼ��
// x ������
// y ������
// block_size ������Χsize*size������
// down        �����ָ�Ч��
uint8 adaptive_thres(uint8 *img0, uint8 x,uint8 y,int16 block_size, int8 down_value)
{
    int16 half = block_size >> 1;
    uint8 pointi = 0, pointj = 0;
    uint16 size = block_size * block_size;
    uint16 thres=0;
    uint8* data0 = img0;  //ָ���������ݵ�ָ��
    for (int16 di = -half; di <= half; di++)
    {
        for (int16 dj = -half; dj <= half; dj++)
        {
            if ((pointi < 0) || (pointj < 0) || (pointi >=H) || (pointj >=W))
            continue;
            pointi = y + di;
            pointj = x + dj;
            thres +=data0[W * pointi + pointj];

        }
    }
   thres /= size;
   thres -= down_value;
   return thres;
}
//������������ȺͱȽ�
uint8 Gray_Search_Line(uint8(*img)[W],uint8 i1,uint8 j1,uint8 i2,uint8 j2,uint8 thres)
{
    int16 pixel_sum=img[i1][j1]+img[i2][j2];
    int16 pixel_deff=(img[i1][j1]>img[i2][j2]) ?
                     (img[i1][j1]-img[i2][j2]) :
                     (img[i2][j2]-img[i1][j1]) ;
    if(thres*pixel_sum<=pixel_deff*100)           //���ڱ߽�
    {
      return 1;                                  //����������
    }else{
      return 0;
    }
}
/*************************************************************************
 *  �������ƣ�short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  ����˵�����������ֵ��С
 *  ����˵����tmImage �� ͼ������
 *  �������أ���
 *  �޸�ʱ�䣺2023��3��15��
 *  ��    ע��  GetOSTU(Image_Use);//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
        �ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
        ������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
*************************************************************************/
short GetOSTU (uint8(*tmImage)[W])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < H; j++)
    {
        for (i = 0; i < W; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}
/*
 *    ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ��
 *    image    ��������
 *    out_image   �������      ����Ķ�ֵ����ı�����Ϣ
 */
void sobelAutoThreshold (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t)
{
    /** ����˴�С */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = Width_t - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = Height_t - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* ���㲻ͬ�����ݶȷ�ֵ  */
            temp[0] = -(short) image[i - 1][j - 1] + (short) image[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) image[i][j - 1] + (short) image[i][j + 1]       // {-1, 0, 1},
            - (short) image[i + 1][j - 1] + (short) image[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) image[i - 1][j - 1] + (short) image[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) image[i - 1][j] + (short) image[i + 1][j]       // { 0,  0,  0},
            - (short) image[i - 1][j + 1] + (short) image[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) image[i - 1][j] + (short) image[i][j - 1]       //  0, -1, -1
            - (short) image[i][j + 1] + (short) image[i + 1][j]       //  1,  0, -1
            - (short) image[i - 1][j + 1] + (short) image[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) image[i - 1][j] + (short) image[i][j + 1]       // -1, -1,  0
            - (short) image[i][j - 1] + (short) image[i + 1][j]       // -1,  0,  1
            - (short) image[i - 1][j - 1] + (short) image[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);
                            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
            temp[3] = (short) image[i - 1][j - 1] + (short) image[i - 1][j] + (short) image[i - 1][j + 1]
                    + (short) image[i][j - 1] + (short) image[i][j] + (short) image[i][j + 1]
                    + (short) image[i + 1][j - 1] + (short) image[i + 1][j] + (short) image[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)  //12.0���Ե��������߻ᵼ�±�Ե��ⲻ���������⣬�����ȹ��ͻᵼ�±�Ե��������ͼ���д��ڽ϶�ķǱ�Ե���
            {
                out_image[i][j] = 0;   //��
            }
            else
            {
                out_image[i][j] = 255;    //��
            }

        }
    }
}
/**************ֻʹ��X��Y����ľ����*************************************/
void sobelAutoThreshold_test (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t)
{
    /** ����˴�С */
    short KERNEL_SIZE = 1;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = Width_t - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = Height_t - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[2];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* ���㲻ͬ�����ݶȷ�ֵ  */
            temp[0] = -(short) image[i - 1][j] + (short) image[i + 1][j]; // xƫ�þ���ˣ�{{-1, 0, 1}};
            temp[1] = -(short) image[i][j - 1] + (short) image[i][j + 1]; // yƫ�þ���ˣ�{{-1}, {0}, {1}};

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
                            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 2; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ��������Ϊ��ֵ */
          temp[1] = (short) image[i - 1][j - 1] + (short) image[i - 1][j] + (short) image[i - 1][j + 1]
                       + (short) image[i][j - 1] + (short) image[i][j] + (short) image[i][j + 1]
                        + (short) image[i + 1][j - 1] + (short) image[i + 1][j] + (short) image[i + 1][j + 1];
            if (temp[0] > temp[1] * 0.1)
                 {
                    out_image[i][j] = 0;
                 }
            else
                 {
                     out_image[i][j] = 255;
                  }
          }
        }
   }
/***************************************************
*  ͼ���ֵ��
*  ��ڲ�����ͼ��Ҷ����� ,�������,��ֵ����ֵ
*  ����ֵ����
***************************************************/
void Binaryation(uint8 *image, uint16 Height1, uint16 Width1,uint8 threshold)
{
   uint8 i,j;
   uint8 *p = image; //ָ��ͼ�������ָ��
   Black_Point_Num=0;        //���
   for (i = 0; i < Height1; i++)
    {
      for (j = 0; j < Width1; j++)
       {
          if (*p > threshold)
        {
           *p = 255;
        }
        else
        {
           *p = 0;
           Black_Point_Num++;           //�ڵ����
        }
           p++;
      }
  }
}
/***************************************************
*  �������(���Ѿ���ֵ����ͼ��ʹ��)��������� ��Χ�����ݽ����޸�Ϊͬһ��ֵ
*  ��ڲ�����ͼ��Ҷ����� ,ͼ��߶ȣ�ͼ����
***************************************************/
void Bin_Image_Filter (uint8(*tmImage)[W])
{
    uint8 i,j;
    for (i = 1; i < Hight/2; i++)
    {
        for (j = 1; j < Width/2; j++)
        {
            if ((tmImage[i][j] == 0)
                    && (tmImage[i - 1][j] + tmImage[i + 1][j] + tmImage[i][j + 1] + tmImage[i][j - 1] > 510))
            {
                tmImage[i][j] = 255;
            }
            else if ((tmImage[i][j] == 255)
                    && (tmImage[i - 1][j] + tmImage[i + 1][j] + tmImage[i][j + 1] + tmImage[i][j - 1] < 510))
            {
                tmImage[i][j] = 0;
            }
        }
    }
}
/*---------------------------------------------------------------
 ����    �ܡ��ҶȲ�Ⱥ��жϵײ��߽��Ƿ���
 ����    ������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void gray_check_bounday_lose(uint8(*image)[94],uint8 thres)
{
    l_gray_point=0;
    r_gray_point=0;
    vert_gray_point=0;
    uint8 l_x=0,r_x=93;//��ʼ������������
    static uint8 mid_begin=47;
    for(uint8 i=mid_begin;i>3;i--)  //�ж����
    {
       if(Gray_Search_Line(image,start_line_point_row,i,start_line_point_row,i-3,thres))
       {
           l_x=i;
           break;
       }
    }

    for(uint8 i=mid_begin;i<90;i++)  //�ж��ұ�
    {
       if(Gray_Search_Line(image,start_line_point_row,i,start_line_point_row,i+3,thres))
       {
           r_x=i;
           break;
       }
    }

    for(uint8 i=start_line_point_row;i>=45;i--)   //�����ж��Ƿ�����·
    {
        if(Gray_Search_Line(image, i, mid_begin, i-3, mid_begin, thres))
        {
            vert_gray_point=1;             //�������ϱ߽��
            break;
        }
    }


    mid_begin=(l_x+r_x)/2;
    if(l_x>3)
    {
        l_gray_point=1;
    }
    if(r_x<90)
    {
        r_gray_point=1;
    }
}
/***************************************************
*  ͼ������
*  ��ڲ�����ͼ��Ҷ����� ,ͼ��������飬�Ƿ�����ֵ��
*  ����ֵ����
***************************************************/
void Image_Deal(uint8(*image)[Width],uint8(*out_image)[W],uint8 type,uint8 clip_value)
{
    uint8 threshold=0;
    threshold=my_adapt_threshold(image,out_image,94,60)+clip_value;                 //�����ֵ
    if(Break_road_state)
    {
        threshold+= Break_road_add_thres;
    }
    gray_check_bounday_lose(out_image,bottom_gray_threa);                                        //�ײ��ҶȲ�Ⱥ��жϱ߽��Ƿ�Ͽ�
    if(type)
    {
      Binaryation((uint8 *)out_image,60,94,threshold);               //��ֵ��
    }
}
//����3��Ƕ�
float getAngle(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    float x;
    float a;
    int16 Vectorx1,Vectorx2,Vectory1,Vectory2;
    Vectorx1 = (int16)(x1- x2);
    Vectory1 = (int16)(y1 - y2);
    Vectorx2 = (int16)(x3 - x2);
    Vectory2 = (int16)(y3 - y2);
    x = (float)((Vectorx1 * Vectorx2 + Vectory1 * Vectory2) / (sqrt((Vectorx1 * Vectorx1 + Vectory1 * Vectory1) * (Vectorx2* Vectorx2+ Vectory2 * Vectory2))));
    a = ( acos(x) * 57.32);//(����ת�Ƕ�)
    return (a);
}
//��С���˷�����б��
float Slope_calculate(uint8 begin,uint8 end,uint8 *p)
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
  uint8 i=0;
  float result=0;
  static float Resultlast=0;
  p=p+begin;
  for(i=begin;i<end;i++)
  {
      xsum+=i;
      ysum+=*p;
      xysum+=i*(*p);
      x2sum+=i*i;
      p=p+1;
  }
 if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
 {
   result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
   Resultlast=result;
 }
 else
 {
  result=Resultlast;
 }
 return result;
}
//��������
unsigned int my_sqrt(int x)
{
 uint8 ans=0,p=0x80;
 while(p!=0)
 {
 ans+=p;
 if(ans*ans>x)
 {
 ans-=p;
 }
 p=(uint8)(p/2);
 }
 return(ans);
}
float process_curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    float K;
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    //����ķ��ű�ʾ����
    char q1 = (char)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    char AB = my_sqrt(q1);
    q1 = (char)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    char BC = my_sqrt(q1);
    q1 = (char)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    char AC = my_sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}
//��ؾ�b
//�����y=startline��y=endline
float Intercept(int startline, int endline,uint8 *Line,float k)
{
    int i = 0;
      float b=0;
    int sumlines = endline - startline;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    Line=Line+startline;
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY +=*Line;
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x��ƽ��ֵ
            averageY = sumY / sumlines;     //y��ƽ��ֵ
        }
        else
        {
            averageX = 0;     //x��ƽ��ֵ
            averageY = 0;     //y��ƽ��ֵ
        }
       b = averageY - k * averageX;            //�ؾ�
           return b;
}

float parameterB=0;    //б��
float parameterA=0;    //�ؾ�
//���߶ν���������ϣ����׼ȷ��
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2)
{
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Mid_Line[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Mid_Line[i];
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Mid_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Mid_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;            //б��
        parameterA = averageY - parameterB * averageX;//�ؾ�

    }
    else if (type == 1)     //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Left_Line[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Left_Line[i];
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Left_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Left_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Right_Line[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Right_Line[i];
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Right_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Right_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }

}
//�޷�����
int limit(int value,int pos,int low)
{
    int mTemp = value;
    mTemp = (mTemp>pos)? pos:mTemp;
    mTemp = (mTemp<low)? low:mTemp;
    return mTemp;
}
//������ȡ����߶�
void  Match_line(uint8 begin,uint8 end,float parameterB,float parameterA,uint8 *line)
{
    uint8 i=0;
    int8 temp=0;
    for(i=end;i>=begin;i--)
    {
        temp=(int)(parameterB * i + parameterA);;
        line[i]=limit(temp,0,93);
    }
}
//���߶����
void regression(int type, int startline, int endline)
{
    int i = 0;
    int sumlines = endline - startline;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)      //�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Mid_Line[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x��ƽ��ֵ
            averageY = sumY / sumlines;     //y��ƽ��ֵ
        }
        else
        {
            averageX = 0;     //x��ƽ��ֵ
            averageY = 0;     //y��ƽ��ֵ
        }
        for (i = startline; i < endline; i++)
        {
            sumUp += (Mid_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 1)//�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Left_Line[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x��ƽ��ֵ
        averageY = sumY / sumlines;     //y��ƽ��ֵ
        for (i = startline; i < endline; i++)
        {

            sumUp += (Left_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)//�������
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Right_Line[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x��ƽ��ֵ
        averageY = sumY / sumlines;     //y��ƽ��ֵ
        for (i = startline; i < endline; i++)
        {
            sumUp += (Right_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
}

