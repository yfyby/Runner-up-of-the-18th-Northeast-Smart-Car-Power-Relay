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
形态学处理，开操作，去除黑斑
***************************************************/
void morph_open(uint8(*image)[W]) {
    uint8 i, j;
    uint8 temp[H][W];
    int16 sum = 0;
    int k, l;
    int SE[3][3] = {{1, 1, 1},
                    {1, 1, 1},
                    {1, 1, 1}};  //用于侵蚀和膨胀的3x3结构元件
    //侵蚀操作
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
    // 膨胀操作
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
形态学处理，关操作
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
复制图像数组
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
原始图像缩放一半
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
 * 图像裁剪
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
 *  函数名称：my_adapt_threshold(uint8 *image, uint16 col, uint16 row))
 *  功能说明：优化大津法求阈值大小
 *  参数说明：tmImage ： 图像数据
 *  函数返回：无
 *  修改时间：2023年3月15日
 *  备    注：  my_adapt_threshold(image,94, 60));//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
        的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
        景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
*************************************************************************/
uint8 my_adapt_threshold(uint8(*tmImage)[Width],uint8(*out_tmImage)[W], uint16 col, uint16 row)                     //注意计算阈值的一定要是原图像
{
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = tmImage;  //指向像素数据的指针
    uint8* out_data = out_tmImage;  //指向像素数据的指针
    short  Row = 0, line = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < Hight; i+=2)
    {
        for (j = 0; j < Width; j+=2)
        {
            pixelCount[(int)data[i * Width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * Width + j];       //灰度值总和
            out_data[Row * width + line] = data[i * Width + j];
            line++;
        }
                line = 0;
                Row++;
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
                w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
                u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

               w1=1-w0;
               u1tmp=gray_sum/pixelSum-u0tmp;

                u0 = u0tmp / w0;              //背景平均灰度
                u1 = u1tmp / w1;              //前景平均灰度
                u = u0tmp + u1tmp;            //全局平均灰度
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
 *  函数名称：my_adapt_part_threshold(uint8 *image, uint16 col, uint16 row))
 *  功能说明：局部图像大津
 *  参数说明：tmImage ： 图像数据
 *  函数返回：无
 *  修改时间：2023年3月15日
 *  备    注：  my_adapt_part_threshold(image,94, 60));//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
        的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
        景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
*************************************************************************/
uint8 my_adapt_part_threshold(uint8(*tmImage)[W], uint16 col, uint16 row,uint16 begin_col, uint16 begin_row)   //注意计算阈值的一定要是原图像
{
    uint16 width = col;
    uint16 height = row;
    uint16 begin_width=begin_col;
    uint16 begin_height=begin_row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = (width-begin_width) * (height-begin_height);
    uint8 threshold = 0;
    uint8* data = tmImage;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = begin_height; i < height; i++)
    {
        for (j = begin_width; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }

    //计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;


        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {

                w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
                u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

               w1=1-w0;
               u1tmp=gray_sum/pixelSum-u0tmp;

                u0 = u0tmp / w0;              //背景平均灰度
                u1 = u1tmp / w1;              //前景平均灰度
                u = u0tmp + u1tmp;            //全局平均灰度
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

// 自适应二值化(平均法)
// img0 输入图像
// img1 输出图像
// col 宽
// row 高
// block_size 像素周围size*size个区域
// down        减弱分割效果
void adaptive_threshold(uint8(*img0)[W],uint8(*img1)[W], uint16 col, uint16 row, int block_size, int down_value)
{
    int16 half = block_size >> 1;
    int16 pointi = 0, pointj = 0;
    int16 size = block_size * block_size;
    int16 imgstep = 0;
    uint16 width = col;
    uint16 height = row;
    uint8 thres=0;
    uint8* data0 = img0;  //指向像素数据的指针
    uint8* data1 = img1;  //指向像素数据的指针
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


// 自适应二值化,使用在缩小一半的图像里
// img0 输入图像
// x 列坐标
// y 行坐标
// block_size 像素周围size*size个区域
// down        减弱分割效果
uint8 adaptive_thres(uint8 *img0, uint8 x,uint8 y,int16 block_size, int8 down_value)
{
    int16 half = block_size >> 1;
    uint8 pointi = 0, pointj = 0;
    uint16 size = block_size * block_size;
    uint16 thres=0;
    uint8* data0 = img0;  //指向像素数据的指针
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
//两点像素做差比和比较
uint8 Gray_Search_Line(uint8(*img)[W],uint8 i1,uint8 j1,uint8 i2,uint8 j2,uint8 thres)
{
    int16 pixel_sum=img[i1][j1]+img[i2][j2];
    int16 pixel_deff=(img[i1][j1]>img[i2][j2]) ?
                     (img[i1][j1]-img[i2][j2]) :
                     (img[i2][j2]-img[i1][j1]) ;
    if(thres*pixel_sum<=pixel_deff*100)           //存在边界
    {
      return 1;                                  //返回列坐标
    }else{
      return 0;
    }
}
/*************************************************************************
 *  函数名称：short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  功能说明：大津法求阈值大小
 *  参数说明：tmImage ： 图像数据
 *  函数返回：无
 *  修改时间：2023年3月15日
 *  备    注：  GetOSTU(Image_Use);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
        的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
        景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
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
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < H; j++)
    {
        for (i = 0; i < W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}
/*
 *    基于soble边沿检测算子的一种自动阈值边沿检测
 *    image    输入数组
 *    out_image   输出数组      保存的二值化后的边沿信息
 */
void sobelAutoThreshold (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t)
{
    /** 卷积核大小 */
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
            /* 计算不同方向梯度幅值  */
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
                            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short) image[i - 1][j - 1] + (short) image[i - 1][j] + (short) image[i - 1][j + 1]
                    + (short) image[i][j - 1] + (short) image[i][j] + (short) image[i][j + 1]
                    + (short) image[i + 1][j - 1] + (short) image[i + 1][j] + (short) image[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)  //12.0可以调整，过高会导致边缘检测不出来的问题，灵敏度过低会导致边缘检测输出的图像中存在较多的非边缘噪点
            {
                out_image[i][j] = 0;   //黑
            }
            else
            {
                out_image[i][j] = 255;    //白
            }

        }
    }
}
/**************只使用X和Y方向的卷积核*************************************/
void sobelAutoThreshold_test (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t)
{
    /** 卷积核大小 */
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
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) image[i - 1][j] + (short) image[i + 1][j]; // x偏好卷积核：{{-1, 0, 1}};
            temp[1] = -(short) image[i][j - 1] + (short) image[i][j + 1]; // y偏好卷积核：{{-1}, {0}, {1}};

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
                            /* 找出梯度幅值最大值  */
            for (k = 1; k < 2; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例作为阈值 */
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
*  图像二值化
*  入口参数：图像灰度数组 ,输出数组,二值化阈值
*  返回值：无
***************************************************/
void Binaryation(uint8 *image, uint16 Height1, uint16 Width1,uint8 threshold)
{
   uint8 i,j;
   uint8 *p = image; //指向图像数组的指针
   Black_Point_Num=0;        //清空
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
           Black_Point_Num++;           //黑点计数
        }
           p++;
      }
  }
}
/***************************************************
*  过滤噪点(对已经二值化的图像使用)三面或四面 被围的数据将被修改为同一数值
*  入口参数：图像灰度数组 ,图像高度，图像宽度
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
 【功    能】灰度差比和判断底部边界是否丢线
 【参    数】无
 【返 回 值】无
 ----------------------------------------------------------------*/
void gray_check_bounday_lose(uint8(*image)[94],uint8 thres)
{
    l_gray_point=0;
    r_gray_point=0;
    vert_gray_point=0;
    uint8 l_x=0,r_x=93;//初始化，左右两边
    static uint8 mid_begin=47;
    for(uint8 i=mid_begin;i>3;i--)  //判断左边
    {
       if(Gray_Search_Line(image,start_line_point_row,i,start_line_point_row,i-3,thres))
       {
           l_x=i;
           break;
       }
    }

    for(uint8 i=mid_begin;i<90;i++)  //判断右边
    {
       if(Gray_Search_Line(image,start_line_point_row,i,start_line_point_row,i+3,thres))
       {
           r_x=i;
           break;
       }
    }

    for(uint8 i=start_line_point_row;i>=45;i--)   //向上判断是否进入断路
    {
        if(Gray_Search_Line(image, i, mid_begin, i-3, mid_begin, thres))
        {
            vert_gray_point=1;             //存在向上边界点
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
*  图像处理函数
*  入口参数：图像灰度数组 ,图像输出数组，是否开启二值化
*  返回值：无
***************************************************/
void Image_Deal(uint8(*image)[Width],uint8(*out_image)[W],uint8 type,uint8 clip_value)
{
    uint8 threshold=0;
    threshold=my_adapt_threshold(image,out_image,94,60)+clip_value;                 //大津法阈值
    if(Break_road_state)
    {
        threshold+= Break_road_add_thres;
    }
    gray_check_bounday_lose(out_image,bottom_gray_threa);                                        //底部灰度差比和判断边界是否断开
    if(type)
    {
      Binaryation((uint8 *)out_image,60,94,threshold);               //二值化
    }
}
//计数3点角度
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
    a = ( acos(x) * 57.32);//(弧度转角度)
    return (a);
}
//最小二乘法计算斜率
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
 if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
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
//计算曲率
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
    //面积的符号表示方向
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
//求截距b
//计算从y=startline到y=endline
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
            averageX = sumX / sumlines;     //x的平均值
            averageY = sumY / sumlines;     //y的平均值
        }
        else
        {
            averageX = 0;     //x的平均值
            averageY = 0;     //y的平均值
        }
       b = averageY - k * averageX;            //截距
           return b;
}

float parameterB=0;    //斜率
float parameterA=0;    //截距
//对线段进行两段拟合，提高准确性
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
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
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
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
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
        else parameterB = sumUp / sumDown;            //斜率
        parameterA = averageY - parameterB * averageX;//截距

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
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
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
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
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
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
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
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
//限幅函数
int limit(int value,int pos,int low)
{
    int mTemp = value;
    mTemp = (mTemp>pos)? pos:mTemp;
    mTemp = (mTemp<low)? low:mTemp;
    return mTemp;
}
//重新求取拟合线段
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
//单线段拟合
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
    if (type == 0)      //拟合中线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Mid_Line[i];
        }
        if (sumlines != 0)
        {
            averageX = sumX / sumlines;     //x的平均值
            averageY = sumY / sumlines;     //y的平均值
        }
        else
        {
            averageX = 0;     //x的平均值
            averageY = 0;     //y的平均值
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
    else if (type == 1)//拟合左线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Left_Line[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {

            sumUp += (Left_Line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)//拟合右线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += Right_Line[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
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

