#include "KX134.h"
#include <string.h>
#include "spi.h"  
#include "FreeRTOS.h"

extern TaskHandle_t DataTaskHandle;

static uint8_t KX134_FreqToHex(uint16_t freq) {
    uint8_t odr_setting = 0x0F; // 默认最高

    switch(freq) {
        case 25600: odr_setting = 0x0F; break;
        case 12800: odr_setting = 0x0E; break;
        case 6400:  odr_setting = 0x0D; break;
        case 3200:  odr_setting = 0x0C; break;
        case 1600:  odr_setting = 0x0B; break;
        case 800:   odr_setting = 0x0A; break;
        case 400:   odr_setting = 0x09; break;
        case 200:   odr_setting = 0x08; break;
        default:    odr_setting = 0x0F; break;
    }

    // 低通滤波 (LPRO = 1)
    // 0x40 (Bit 6) 对应 LPRO 位
    return odr_setting; 
		//return odr_setting | 0x40; 
}

// 内部写函数
static void KX134_WriteReg(uint8_t Reg, uint8_t Val) {
    uint8_t data[2];
    data[0] = Reg; 
    data[1] = Val;
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}

// 内部读函数
uint8_t KX134_ReadReg(uint8_t Reg) {
    uint8_t tx_data = Reg | 0x80; 
    uint8_t rx_data;
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, 10);
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
    return rx_data;
}

//uint8_t KX134_ReadReg(uint8_t Reg) {
//    uint8_t tx_data[2];
//    uint8_t rx_data[2];
//    
//    // 构造标准 SPI 读帧：[寄存器地址|0x80] + [Dummy数据]
//    tx_data[0] = Reg | 0x80; 
//    tx_data[1] = 0x00;       // 提供时钟用的 Dummy Byte
//    
//    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
//    
//    //使用 TransmitReceive 同时收发，保证时钟绝对连续
//    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 100);
//    
//    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
//    
//    // rx_data[0] 是发送地址时收到的（通常无效），rx_data[1] 才是数据
//    return rx_data[1];
//}

uint8_t KX134_Init(void) {
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		KX134_WriteReg(KX134_CNTL1, 0x00);
	  HAL_Delay(50);
		KX134_WriteReg(KX134_CNTL2, 0x80);
		HAL_Delay(50);
//		uint8_t check_reset = KX134_ReadReg(KX134_CNTL2);
//		KX134_WriteReg(KX134_CNTL1, 0xAA);
//		uint8_t test_val = KX134_ReadReg(KX134_CNTL1);
    // 1. 软件复位 (规格书 Page 2: 必须先写 0x00 到 CNTL1 进 Standby [cite: 45])
    KX134_WriteReg(KX134_CNTL1, 0x00); 
    HAL_Delay(50);

    // 复位 Buffer 指针
    KX134_WriteReg(KX134_BUF_CNTL2, 0x80); 
    HAL_Delay(50);
    KX134_WriteReg(KX134_BUF_CNTL2, 0x00);
    
    // 2. 检查 ID (KX134-1211 默认 0x46)
//    if (KX134_ReadReg(KX134_WHO_AM_I) != 0x46) {
//        return 0; // ID 错误
//    }
    
    // 3. 配置 ODR = 25600Hz
    // 虽然规格书只列了 0x06=50Hz [cite: 49]，但标准定义 0x0F=25.6kHz
    KX134_WriteReg(KX134_ODCNTL, 0x0F); 
    
    // 4. 配置中断引脚 INT1
    // [cite: 536-545] 0x38 = Enabled(Bit5), Active High(Bit4), Pulse(Bit3)
    // 使用脉冲模式，自动清除中断，无需读取 INT_REL
    KX134_WriteReg(KX134_INC1, 0x38); 
    
    // 5. 映射中断: 将 FIFO 水位中断 (WMI) 映射到 INT1
    // [cite: 133] INC4 0x20 = WMI on INT1 (注意: 如果用 BufferFull则是0x40)
    // 我们用水位中断 WMI (0x20) 以便源源不断读取
    KX134_WriteReg(KX134_INC4, 0x20); 
    
    // 6. 设置 FIFO 水位
    //  地址 0x5E
    KX134_WriteReg(KX134_BUF_CNTL1, FIFO_WATERMARK); 
    
    // 7. 开启 FIFO (Stream Mode)
    //  BUF_CNTL2 0xE0 = BUFE(1)|BRES(1)|BFIE(1)|BM(0)
    // 注意：这里我们只要 Buffer Enable 和 Resolution，中断已经在 INC4 映射了 WMI
    // 但 BFIE 位通常控制的是 "Buffer相关的中断总开关"，所以设为 1 是对的
    KX134_WriteReg(KX134_BUF_CNTL2, 0xE0);
    
    // 8. 启动传感器
    // [cite: 51] CNTL1 0xC0 是 +/-8g.
    // 我们需要 +/-64g -> GSEL=11 -> 0xE0 | 0x18 = 0xF8
    // PC1(1)|RES(1)|DRDYE(0)|GSEL(11) = 1101 1000 = 0xD8 (不带DRDY)
    // 或者 1111 1000 = 0xF8 (带 DRDY)
    // 既然用 FIFO，建议关掉 DRDY (Bit5=0) 以免 INT1 被 DRDY 信号干扰
    KX134_WriteReg(KX134_CNTL1, 0xD8); 
    
		__HAL_GPIO_EXTI_CLEAR_IT(KX134_INT1_Pin);  
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		
    return 1;
}

void KX134_Read_FIFO_DMA(uint8_t *target_buffer) {
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
    
    // [cite: 206] 规格书 Page 9 强调要用 Repeated Start 或者直接读
    // 对于 DMA，最简单的是发送寄存器地址后直接通过 SPI 时钟读数据
    // KX134_BUF_READ (0x63) | 0x80 = 0xE3
    uint8_t cmd = KX134_BUF_READ | 0x80;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    
		HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&hspi1, target_buffer, FIFO_WATERMARK * 6);		
		if (status != HAL_OK) {
					// Error_Handler(); 
					// 查看 hspi1.ErrorCode
			}
}

void KX134_CS_High(void) {
    HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
}

uint8_t KX134_SetODR(uint16_t freq_hz) {
    uint8_t odr_val = KX134_FreqToHex(freq_hz);
    
    // 1. 读取 CNTL1 当前值
    uint8_t ctrl1 = KX134_ReadReg(KX134_CNTL1);
    
    // 2. 如果当前是工作模式 (PC1=1)，必须先切到 Standby (PC1=0)
    if (ctrl1 & 0x80) {
        KX134_WriteReg(KX134_CNTL1, ctrl1 & 0x7F); 
        // 规格书建议等待一小段时间，确保状态切换
        // HAL_Delay(1); 
    }
    
    // 3. 写入新的 ODR
    KX134_WriteReg(KX134_ODCNTL, odr_val);
    
    // 4. 恢复之前的模式 (如果是工作模式则恢复为工作模式)
    if (ctrl1 & 0x80) {
        KX134_WriteReg(KX134_CNTL1, ctrl1);
    }
    
    return 1;
}