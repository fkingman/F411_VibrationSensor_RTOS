#ifndef __APP_ACQ_H__
#define __APP_ACQ_H__

#include "main.h" 
#include <stdint.h>

#define FIFO_WATERMARK          32               //水位
#define BYTES_PER_SAMPLE        6                //(X_L, X_H, Y_L, Y_H, Z_L, Z_H)6字节

// ================= 寄存器地址=================
#define KX134_WHO_AM_I      0x13
#define KX134_CNTL1         0x1B
#define KX134_ODCNTL        0x21
#define KX134_INC1          0x22
#define KX134_INC4          0x25
#define KX134_BUF_CNTL1     0x5E  
#define KX134_BUF_CNTL2     0x5F  
#define KX134_BUF_STATUS_1  0x60
#define KX134_BUF_READ      0x63  



extern volatile uint8_t g_acq_complete_flag; // 1 = 采集完成，数据已准备好
extern volatile uint8_t g_acq_running_flag;  // 1 = 正在采集中
extern uint16_t cfg_freq;
extern uint8_t cfg_odr;

uint8_t KX134_Init(void);
void KX134_Read_FIFO_DMA(uint8_t *target_buffer);
void KX134_CS_High(void); 

#endif
