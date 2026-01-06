#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h" 
// #include "stm32f4xx_hal.h" // 也可以直接引这个

#include <stdint.h>
#include <stdbool.h>
#include <string.h> 

/* ---- 扇区配置 (基于 STM32F411xE 512KB) ---- */
/* * STM32F411RE/CE (512KB Flash) 扇区分布:
 * Sector 0-3: 16KB
 * Sector 4:   64KB
 * Sector 5-7: 128KB
 * * 这里使用最后一个扇区 Sector 7 (0x08060000 - 0x0807FFFF)
 * 注意：如果是 F411xC (256KB Flash)，最后一个扇区是 Sector 5 (0x08020000)
 */
#define FLASH_CFG_SECTOR        FLASH_SECTOR_2      
#define FLASH_CFG_BASE_ADDR     ((uint32_t)0x08008000u)

/* ---- 默认参数定义 ---- */
#define FLASH_CFG_MAGIC         0xA5A55A5Au
#define FLASH_CFG_DEFAULT_ADDR  0x00
#define FLASH_CFG_DEFAULT_FREQ  25600
#define FLASH_CFG_DEFAULT_POINTS 4096u

/* ---- OTA 标志位定义 ---- */
#define OTA_FLAG_UPDATE_NEEDED  0x5A5A5A5A 

/* ---- 核心配置结构体 (32 Bytes) ---- */
/* 保持结构体对齐不变，确保跨平台兼容性 */
typedef struct __attribute__((packed,aligned(4))) {
    uint32_t magic;        
    uint8_t  version;      
    uint8_t  addr;         
    uint16_t samp_freq_hz; 
    uint16_t points;       
    
    uint32_t ota_flag;     
    uint32_t fw_len;       
    uint32_t fw_crc;       
    uint8_t  rsv[8];       

    uint16_t crc;          
} flash_dev_cfg_t;

#ifdef __cplusplus
extern "C" {
#endif

void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points);
HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points);

void Flash_ReadOTAInfo(uint32_t* out_flag, uint32_t* out_len);
HAL_StatusTypeDef Flash_SetOTAInfo(uint32_t flag, uint32_t len);

uint8_t Flash_ReadDeviceAddr(void);
HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr);
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq);
HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H__ */