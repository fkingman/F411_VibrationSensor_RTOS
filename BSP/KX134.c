#include "KX134.h"
#include <string.h>
#include "spi.h"  
#include "FreeRTOS.h"

extern TaskHandle_t DataTaskHandle;


// 内部使用的写寄存器函数 (阻塞式)
static void KX134_WriteReg(uint8_t Reg, uint8_t Val) {
    uint8_t data[2];
    data[0] = Reg; 
    data[1] = Val;
    
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}
// 内部使用的读寄存器函数
static uint8_t KX134_ReadReg(uint8_t Reg) {
    uint8_t tx_data = Reg | 0x80; 
    uint8_t rx_data;
    
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, 10);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
    
    return rx_data;
}

uint8_t KX134_Map_Freq_To_Reg(uint16_t freq)
{
    // 参考 KX134 数据手册 ODCNTL (0x21) 寄存器定义
    switch(freq)
    {
        case 25600: return 0x0F;
        case 12800: return 0x0E;
        case 6400:  return 0x0D;
        case 3200:  return 0x0C;
        case 1600:  return 0x0B;
        case 800:   return 0x0A;
        case 400:   return 0x09;
        case 200:   return 0x08;
        case 100:   return 0x07;
        case 50:    return 0x06;
        case 25:    return 0x05;
        case 12:    return 0x04; // 12.5Hz
        default:    return 0x06; // 默认给个 50Hz 安全值，或者 0x0F 最高速
    }
}

uint8_t KX134_Init(void) {
    // 1. 软件复位
    KX134_WriteReg(KX134_CNTL1, 0x00); // 必须先切到 Standby
    KX134_WriteReg(KX134_BUF_CNTL2, 0x80); // BUF_RESET
    HAL_Delay(5);
    KX134_WriteReg(KX134_BUF_CNTL2, 0x00); // 清除 reset
    
    // 2. 检查 ID (KX134 通常是 0x46)
    uint8_t wai = KX134_ReadReg(KX134_WHO_AM_I);
    if (wai != 0x46) {

    }
    
    // 3. 配置 ODR (输出速率 25600Hz)
    KX134_WriteReg(KX134_ODCNTL, 0x0F); 
    
    // 4. 配置中断引脚 INT1 (Active High, Pulse)
    KX134_WriteReg(KX134_INC1, 0x30); 
    
    // 5. 将 FIFO 水位中断 (WMI) 映射到 INT1
    KX134_WriteReg(KX134_INC4, 0x10); 
    
    // 6. 设置 FIFO 水位 (Watermark samples)
    KX134_WriteReg(KX134_BUF_CNTL1, FIFO_WATERMARK); 
    
    // 7. 开启 FIFO (Stream Mode)
    // Bit7(BUFE)=1, Bit6(BRES)=1(16bit), Bit5(BFIE)=1(开启中断)
    KX134_WriteReg(KX134_BUF_CNTL2, 0xE0);
    
    // 8. 启动传感器 (High Performance, +/-64g)
    // Bit7(PC1)=1, Bit6(RES)=1, Bit4/3(GSEL)=11(64g) -> 1101 1000 = 0xD8
    KX134_WriteReg(KX134_CNTL1, 0xD8); 
    
    return 1;
}

void KX134_Read_FIFO_DMA(uint8_t *target_buffer) {
    // 1. 拉低 CS
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    
    // 2. 发送读取命令 (Burst Read)
    uint8_t cmd = KX134_BUF_READ | 0x80;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    
    // 3. 启动 DMA 接收 (长度 = 水位 * 6字节)
    HAL_SPI_Receive_DMA(&hspi1, target_buffer, FIFO_WATERMARK * BYTES_PER_SAMPLE);
}

void KX134_CS_High(void) {
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}