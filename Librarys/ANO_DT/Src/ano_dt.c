/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "ano_dt.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					    //需要发送数据的标志
uint8_t data_to_send[50];	//发送数据缓存

UART_HandleTypeDef *gAnoDtHuart;

void ANO_DT_Init(UART_HandleTypeDef *huart)
{
  gAnoDtHuart = huart;
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
#ifdef ANO_DT_USE_USB_HID
  Usb_Hid_Adddata(data_to_send,length);
#endif
#ifdef ANO_DT_USE_USART2
  Usart2_Send(data_to_send, length);
#endif
  HAL_UART_Transmit(gAnoDtHuart, dataToSend, length, 0xFF);
}

static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
  data_to_send[0]=0xAA;
  data_to_send[1]=0xAA;
  data_to_send[2]=0xEF;
  data_to_send[3]=2;
  data_to_send[4]=head;
  data_to_send[5]=check_sum;


  uint8_t sum = 0;
  for(uint8_t i=0;i<6;i++)
    sum += data_to_send[i];
  data_to_send[6]=sum;

  ANO_DT_Send_Data(data_to_send, 7);
}

void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
  uint8_t _cnt=0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x00;
  data_to_send[_cnt++]=0;

  data_to_send[_cnt++]=hardware_type;
  data_to_send[_cnt++]=BYTE1(hardware_ver);
  data_to_send[_cnt++]=BYTE0(hardware_ver);
  data_to_send[_cnt++]=BYTE1(software_ver);
  data_to_send[_cnt++]=BYTE0(software_ver);
  data_to_send[_cnt++]=BYTE1(protocol_ver);
  data_to_send[_cnt++]=BYTE0(protocol_ver);
  data_to_send[_cnt++]=BYTE1(bootloader_ver);
  data_to_send[_cnt++]=BYTE0(bootloader_ver);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
  uint8_t _cnt=0;
  int32_t _temp;
  int32_t _temp2 = alt;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x01;
  data_to_send[_cnt++]=0;

  _temp = (int)(angle_rol*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (int)(angle_pit*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (int)(((double) angle_yaw)*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  data_to_send[_cnt++]=BYTE3(_temp2);
  data_to_send[_cnt++]=BYTE2(_temp2);
  data_to_send[_cnt++]=BYTE1(_temp2);
  data_to_send[_cnt++]=BYTE0(_temp2);

  data_to_send[_cnt++] = fly_model;

  data_to_send[_cnt++] = armed;

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
  uint8_t _cnt=0;
  int16_t _temp;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;

  _temp = a_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  _temp = g_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  _temp = m_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++] = sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
  uint8_t _cnt=0;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x03;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=BYTE1(thr);
  data_to_send[_cnt++]=BYTE0(thr);
  data_to_send[_cnt++]=BYTE1(yaw);
  data_to_send[_cnt++]=BYTE0(yaw);
  data_to_send[_cnt++]=BYTE1(rol);
  data_to_send[_cnt++]=BYTE0(rol);
  data_to_send[_cnt++]=BYTE1(pit);
  data_to_send[_cnt++]=BYTE0(pit);
  data_to_send[_cnt++]=BYTE1(aux1);
  data_to_send[_cnt++]=BYTE0(aux1);
  data_to_send[_cnt++]=BYTE1(aux2);
  data_to_send[_cnt++]=BYTE0(aux2);
  data_to_send[_cnt++]=BYTE1(aux3);
  data_to_send[_cnt++]=BYTE0(aux3);
  data_to_send[_cnt++]=BYTE1(aux4);
  data_to_send[_cnt++]=BYTE0(aux4);
  data_to_send[_cnt++]=BYTE1(aux5);
  data_to_send[_cnt++]=BYTE0(aux5);
  data_to_send[_cnt++]=BYTE1(aux6);
  data_to_send[_cnt++]=BYTE0(aux6);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];

  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
  uint8_t _cnt=0;
  uint16_t temp;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0;

  temp = votage;
  data_to_send[_cnt++]=BYTE1(temp);
  data_to_send[_cnt++]=BYTE0(temp);
  temp = current;
  data_to_send[_cnt++]=BYTE1(temp);
  data_to_send[_cnt++]=BYTE0(temp);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];

  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
  uint8_t _cnt=0;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x06;
  data_to_send[_cnt++]=0;

  data_to_send[_cnt++]=BYTE1(m_1);
  data_to_send[_cnt++]=BYTE0(m_1);
  data_to_send[_cnt++]=BYTE1(m_2);
  data_to_send[_cnt++]=BYTE0(m_2);
  data_to_send[_cnt++]=BYTE1(m_3);
  data_to_send[_cnt++]=BYTE0(m_3);
  data_to_send[_cnt++]=BYTE1(m_4);
  data_to_send[_cnt++]=BYTE0(m_4);
  data_to_send[_cnt++]=BYTE1(m_5);
  data_to_send[_cnt++]=BYTE0(m_5);
  data_to_send[_cnt++]=BYTE1(m_6);
  data_to_send[_cnt++]=BYTE0(m_6);
  data_to_send[_cnt++]=BYTE1(m_7);
  data_to_send[_cnt++]=BYTE0(m_7);
  data_to_send[_cnt++]=BYTE1(m_8);
  data_to_send[_cnt++]=BYTE0(m_8);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];

  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
  uint8_t _cnt=0;
  int16_t _temp;

  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x10+group-1;
  data_to_send[_cnt++]=0;


  _temp = p1_p * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p1_i  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p1_d  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p2_p  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p2_i  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p2_d * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p3_p  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p3_i  * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = p3_d * 1000;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  data_to_send[3] = _cnt-4;

  uint8_t sum = 0;
  for(uint8_t i=0;i<_cnt;i++)
    sum += data_to_send[i];

  data_to_send[_cnt++]=sum;

  ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
