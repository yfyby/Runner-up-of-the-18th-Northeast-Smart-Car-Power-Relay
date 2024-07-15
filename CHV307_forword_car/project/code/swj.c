/*
 * GL-Shw智能车上位机  qq群:297809802 qq1849057843
 *
 *目录：
 *|||||||||||||||||||【以下图传协议只能选择一种】
 * 一、灰度图传
 *  ①普通灰度图传
 *  ②抗干扰灰度图传
 *  ③压缩灰度图传
 * 二、二值化图传（*该协议带有40倍左右的压缩率和校验 ）[强烈推荐]
 *  ①二值化图传
 *  ②带校验二值化图传
 *  ③修改图像大小的二值化图传
 *
 * ||||||||||||||||||【以下协议可以与图传协议并行使用】
 * 三、图传边界（寻线的边界信息，推荐搭配二值图传，实时查看赛道与寻线补线）[推荐]
 * 四、数据标签（数据示波，数据颜色标定，数据监视，仪表盘）[强烈推荐]
 *                          更新时间：2022年12月9日
 */
#include "swj.h"

//需要修改的地方
#define  sw_write_byte(dat)  uart_write_byte(UART_7,dat)//串口发送字节
#define  sw_write_buffer(dat,len)  uart_write_buffer(UART_7,dat,len)//无线串口发送数组或串口发送数组

/*
 * 修改说明:
 * sw_write_byte 去找串口库中的发送字节函数
 * sw_write_buffer 去找 *无线串口 库 中的发送数组函数
 *
 * 如果使用的逐飞的库 和 逐飞的无线模块 sw_write_buffer这个注意一下
 * */




//--------------普通灰度图传-------------------//
//*image图像地址 width图像宽 height图像高
//例：sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H);
//数据包大小:6+width * height(图传一帧的字节数)
void sendimg(uint8* image, uint8 width, uint8 height)
{
    uint8 dat[6] = { 0x21, 0x7A, width, height, 0x21, 0x7A };
    sw_write_buffer(dat, 6);
    sw_write_buffer(image, width * height);
}
//--------------抗干扰灰度图传-------------------//
//当丢失数据的情况下，该协议能重新定位行来实现一定程度抗干扰能力
//*image图像地址 width图像宽 height图像高
//例：sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H);
//数据包大小:6+（width+3） * height(图传一帧的字节数)
void sendimg_A( uint8* image, uint8 width, uint8 height)
{

    sw_write_byte(0x21); sw_write_byte(0x7A);
    sw_write_byte(width);sw_write_byte(height);
    sw_write_byte((width+height)/2);sw_write_byte(0x7A);

    uint8 line=0,col=0;
    for(line=0;line<width;line++)
        {
        sw_write_byte(21);
        sw_write_byte(line);
        sw_write_byte(133);
           for(col=0;col<height;col++)
           {
               sw_write_byte(*(image+line*height+col));

           }

        }
}
//--------------压缩灰度图传-------------------//
//发送压缩图像 例如 120x180的图像太大 传输速度慢  则可以发送 60x90的图像来提高速度
//*image图像地址 width图像宽 height图像高dis_width压缩后的图像宽 dis_height压缩后的图像高
//例：sendimg(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H,MT9V03X_DVP_W/2,MT9V03X_DVP_H/2);
//数据包大小:6+dis_width * dis_height(图传一帧的字节数)
void sendimg_zoom(uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height)
{
    uint8 dat[6] = { 0x21, 0x7A, dis_width, dis_height, 0x21, 0x7A };
    sw_write_buffer(dat, 6);
    uint8 i,j;
    for(j=0;j<dis_height;j++)
    {
    for(i=0;i<dis_width;i++)
    {
       sw_write_byte(*(image+(j*height/dis_height)*width+i*width/dis_width));//读取像素点

    }
    }
}

//--------------二值化图传-------------------//
//image图像地址 width图像宽 height图像高 otu二值化阈值
//例：sendimg_binary(mt9v03x_image_dvp[0], MT9V03X_DVP_W, MT9V03X_DVP_H,100);
void sendimg_binary( uint8* image, uint8 width, uint8 height,uint8 otu)
{

    uint8 dat[6]={0x7A,0x21,width,height,0x7A,0x21};
    sw_write_buffer(dat,6);
    int databool=255;uint8 lon=1;int data=255;
    uint8 line=0,col=0;

    for(line=0;line<width;line++)
    {
        for(col=0;col<height;col++)
        {
            if(*(image+line*height+col)>otu)data=255;
            else data=0;
            if(data==databool)
            {lon++;}else{sw_write_byte(lon);lon=1;}
            if(lon==190){sw_write_byte(lon-1);sw_write_byte(0);lon=1;}
            databool=data;
        }
    }
}
//压缩二值图传
//uint16 dis_width, uint16 dis_height 要压缩图像大小
void sendimg_binary_zoom( uint8* image, uint8 width, uint8 height, uint8 dis_width, uint8 dis_height,uint8 otu)
{

    uint8 dat[6]={0x7A,0x21,dis_width,dis_height,0x7A,0x21};
    sw_write_buffer(dat,6);
    int databool=255;uint8 lon=1;int data=255;
    uint8 i,j;
    for(j=0;j<dis_height;j++)
    {
    for(i=0;i<dis_width;i++)
    {
    if(*(image+(j*height/dis_height)*width+i*width/dis_width)>otu)//读取像素点
    data=255;
    else data=0;
    if(data==databool)
    {lon++;}
    else{sw_write_byte(lon);lon=1;}
    if(lon==190){sw_write_byte(lon-1);sw_write_byte(0);lon=1;}
    databool=data;
    }
    }

}
//带有校验的二值图传
//chk值越大 抗干扰越强 值0-55
//请根据实际使用情况进行调整
void sendimg_binary_CHK(uint8* image, uint8 width, uint8 height,uint8 otu,uint8 chk)
{
    chk=chk>0?chk:0;
    chk=chk<56?chk:55;
    uint8 dat[7]={0x7A,0x21,width,height,0x7A,0x21,200+chk};
      sw_write_buffer(dat,7);
      int databool=255;uint8 lon=0;int data=255;
      uint8 line=0,col=0;
      int imglon=0;
      int imgdatlo=width*height/chk;
      uint8 CHK=0;
      for(line=0;line<width;line++)
          {
             for(col=0;col<height;col++)
             {imglon++;

                if(*(image+line*height+col)>otu)data=255;
                else data=0;
                if(data==databool)
                {lon++;}
                else{sw_write_byte(lon);lon=1;}

                if(imglon==imgdatlo)
                {CHK++;sw_write_byte(lon);data=255; databool=255;sw_write_byte(200+CHK);lon=0;imglon=0;}
                if(lon==190){sw_write_byte(lon);sw_write_byte(0);lon=0;}
               databool=data;
             }
          }
}

//--------------数据标签（数据示波，数据颜色标定，数据监视，仪表盘）-------------------//
//一个数据占一个地址,会直接出现在图传页面的右栏 点一下可以示波，右键可以设置颜色标定 设定好
//阈值会颜色标定数据 快速监视条件有没有触发，也可以绑定表盘，速度更直观。录制时会同步录制
//类似于黑匣子，小车什么状态一眼便知
//name数据标识(通道、地址)[0-255]  dat:数据
//例:int a=0;put_int32(0,a);

//带有校验的数据示波
void put_int32(uint8 name, int dat)
{
    uint8 datc[10] = { 197, name,1,0,0,0,0,0,0,198};
    datc[3] = (uint8)(dat & 0xFF);
    datc[4] = (uint8)((dat & 0xFF00) >> 8);
    datc[5] = (uint8)((dat & 0xFF0000) >> 16);
    datc[6] = (uint8)((dat >> 24) & 0xFF);
    uint8 crc[6] = { name, 1,datc[3],datc[4],datc[5],datc[6]};
    uint16 CRC16 =  swj_CRC(crc,0,6);
    datc[7] = (uint8)(CRC16&0xff);
    datc[8] = (uint8)(CRC16>>8);
    sw_write_buffer(datc,10);
}

void put_float(uint8 name, float dat)
{
    uint8 datc[10] = { 197, name,2,0,0,0,0,0,0,198};
    char farray[4] = {0};

    *(float*)farray=dat;
    unsigned char *p = (unsigned char*)&dat + 3;
    datc[3] = *(p-3);
    datc[4] = *(p-2);
    datc[5] = *(p-1);
    datc[6] = *p;
   
    uint8 crc[6] = { name, 2,datc[3],datc[4],datc[5],datc[6]};
    uint16 CRC16 =  swj_CRC(crc,0,6);
    datc[7] = (uint8)(CRC16&0xff);
    datc[8] = (uint8)(CRC16>>8);
    sw_write_buffer(datc,10);
}


//--------------传线-------------------//
/*
 * 可以将寻线得到的左右边线，拟的中线发送到上位机查看
 * 例如：
 * 处理 图像img[H][W];得到左右边线存放在zx[H] yx[H] 拟出来的中线为w[H]
 * sendline_clear(swj_BLACK,H,W);//清屏 背景黑色
 * sendline(swj_WHITE,zx,H);//发送左边线
 * sendline(swj_WHITE,yx,H);//发送右边线
 * sendline(swj_WHITE,wx,H);//发送中线
 * 例如：
 * sendline_clear(swj_BLACK,H,W);//清屏 背景黑色
 * sendline_xy(zx,yx,H)//发送左右边线
 * sendline(swj_WHITE,wx,H);//发送中线
 *
 * 如上两个方法效果一致的，但下面可以在上位机上对应使用赛道还原功能
 * 注意：
 * ①每发送完一帧的图像边线 就要调用sendline_clear进行清屏
 * ②如果调用sendline_xy函数并使用上位机赛道还原功能时，若再调用sendline 一定放在sendline_xy后面 防止被覆盖
 * */
#define swj_BLACK 0
#define swj_WHITE 1
#define swj_RED 2
#define swj_GREEN 3
#define swj_BLUE 4
#define swj_PURPLE 5
#define swj_YELLOW 6
#define swj_CYAN 7
#define swj_ORANGE 8
//清空线条   color清屏后的背景颜色  uint16 width uint16 height 图像的大小
void sendline_clear( uint8 color,uint8 width, uint8 height)
{
    sw_write_byte(0x21); sw_write_byte(0x7A);
    sw_write_byte(width);sw_write_byte(height);
    sw_write_byte(color);sw_write_byte(0x21);
}
//图传赛道边界  uint8_t *zx:左边界   uint8_t *yx:右边界, uint32_t len发送的边线长度
//该函数与下放函数分别发送两个边线有何不同? 该函数可对应上位机还原赛道的功能*  注意先后顺序
void sendline_xy( uint8 *line_zx,uint8 *line_yx, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(9);
    sw_write_byte(len);
    sw_write_byte(255);
    sw_write_byte(255);
    sw_write_buffer(line_zx,len);
    sw_write_buffer(line_yx,len);
}
/*默认每行一个点*/
//绘制边线   color边线颜色  uint8_t *buff 发送的边线数组地址  len发送的边线长度
void sendline( uint8 color,uint8 *buff, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(color);
    sw_write_byte(len);
    sw_write_byte(255);
    sw_write_byte(255);
    sw_write_buffer(buff,len);
}
/*说明:
 * 例如有三个点 a(x1,y1)b(x2,y2)c(x3,y3)
 * 则 uint8 x[3]={x1,x2,x3};uint8 y[3]={y1,y2,y3};
 *  sendline2(swj_WHITE,x,y,3);
 *  sendline函数只能按顺序每行一点发送边界点
 *  sendline2函数针对于八邻域等 每行不固定点数的边界
  *           也适用于发送特殊点 例如拐点 灵活运用
 *
 * */
//无序绘制边线  color边线颜色 linex对应点的x坐标集合 liney对应点的y坐标集合  len发送的边线长度
void sendline2( uint8 color,uint8 *linex,uint8 *liney, uint32 len)
{
    sw_write_byte(0x21);
    sw_write_byte(color);
    sw_write_byte(len);
    sw_write_byte(254);
    sw_write_byte(255);
    sw_write_buffer(linex,len);
    sw_write_buffer(liney,len);

}

#define swj_point_type1 1 //小十字 3x3
#define swj_point_type2 2 //中十字 5x5
#define swj_point_type3 3 //大十字 7x7
#define swj_point_type4 4 //小  X  3x3
#define swj_point_type5 5 //中  X  5x5
#define swj_point_type6 6 //大  X  7x7
#define swj_point_type7 7 //全屏十字

//标志点（列如拐点 特殊补线的点）
//例如 点（10,11）sendpoint(swj_RED,10,11,swj_point_type1);//（10，11）处出现红色小十字
//颜色 坐标x 坐标y 点类型（见上面的宏定义）
void sendpoint(uint8 color,uint8 x,uint8 y, uint8 type)
{
    sw_write_byte(0x22);
    sw_write_byte(color);
    sw_write_byte(type);
    sw_write_byte(254);
    sw_write_byte(x);
    sw_write_byte(y);
    sw_write_byte(255);
}

//内部调用
uint16 swj_CRC(uint8 *Buf,uint8 CRC_sta, uint8 CRC_CNT)
{
    uint16 CRC_Temp;
    uint8 i,j;
    CRC_Temp = 0xffff;

    for (i=CRC_sta;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
//int32 ByteToInt(int8 *ndsk)
//{
//    int32 m = 0;
//    int8 *h = (int8 *)&m;
//    h[0] = ndsk[0];
//    h[1] = ndsk[1];
//    h[2] = ndsk[2];
//    h[3] = ndsk[3];
//
//    if(0 == m)
//    {
//        m = 1;
//    }
//
//    return m;
//}
//float ByteToFloat(unsigned char* byteArry)
//{
//return *((float*)byteArry);
//}
//--------------------------------------------------无线模块-----------------------------------------------//
/*
 * 无线模块 推荐使用逐飞的无线模块（usb to nrf）*购买时建议同时购买ttl，用于配置小车的无线模块
 * 波特率可以最高拉到460800 *对应小车上的程序,小车的无线模块，电脑上的无线模块，上位机波特率一致
 * 无线模块速度比wifi慢 但优点是稳定 对于灰度图传速度不太够用，但有二值化图传优化 适合边图传边示波器
 *
 */
