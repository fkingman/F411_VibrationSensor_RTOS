#include "flash.h"

// 计算 CRC 
static uint16_t crc16_modbus_local(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// 内部函数：读取完整配置 (保持不变，F4 也支持直接内存寻址读取)
static void Flash_ReadWholeConfig(flash_dev_cfg_t* cfg)
{
    const flash_dev_cfg_t *p = (const flash_dev_cfg_t*)FLASH_CFG_BASE_ADDR;
    
    // 检查 Magic 和 CRC
    if (p->magic == FLASH_CFG_MAGIC) {
        uint16_t crc = crc16_modbus_local((const uint8_t*)p, sizeof(flash_dev_cfg_t)-2);
        if (crc == p->crc) {
            memcpy(cfg, p, sizeof(flash_dev_cfg_t));
            return;
        }
    }

    // 数据无效时填充默认值
    memset(cfg, 0xFF, sizeof(flash_dev_cfg_t));
    cfg->magic   = FLASH_CFG_MAGIC;
    cfg->version = 2;
    cfg->addr    = FLASH_CFG_DEFAULT_ADDR;
    cfg->samp_freq_hz = FLASH_CFG_DEFAULT_FREQ;
    cfg->points  = FLASH_CFG_DEFAULT_POINTS;
    cfg->ota_flag = 0; 
    cfg->fw_len   = 0;
}

// 内部函数：擦除并写入完整结构体 (针对 STM32F4 修改)
static HAL_StatusTypeDef Flash_ProgramWholeConfig(flash_dev_cfg_t* cfg)
{
    HAL_StatusTypeDef st;
    uint32_t err = 0;

    // 1. 计算新的 CRC
    cfg->crc = crc16_modbus_local((uint8_t*)cfg, sizeof(flash_dev_cfg_t)-2);

    // 2. 解锁 Flash
    HAL_FLASH_Unlock();

    // 3. 擦除 Sector
    FLASH_EraseInitTypeDef ei = {0};
    ei.TypeErase = FLASH_TYPEERASE_SECTORS;
    ei.Sector    = FLASH_CFG_SECTOR;
    ei.NbSectors = 1;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3; // F4 2.7V-3.6V 使用 Range 3 允许按字写入
    // F411 通常不需要设置 ei.Banks，除非是双 Bank 型号

    st = HAL_FLASHEx_Erase(&ei, &err);
    if (st != HAL_OK) { 
        HAL_FLASH_Lock(); 
        return st; 
    }

    // 4. 写入
    // 按 Word (32-bit / 4字节) 循环写入
    uint32_t *pData = (uint32_t*)cfg;
    uint32_t targetAddr = FLASH_CFG_BASE_ADDR;
    uint8_t  wordsToWrite = sizeof(flash_dev_cfg_t) / 4; // 32字节 / 4 = 8个字

    for (uint8_t i = 0; i < wordsToWrite; i++)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, targetAddr, pData[i]);
        if (st != HAL_OK) {
            break; // 写入出错，跳出
        }
        targetAddr += 4;
    }
    
    HAL_FLASH_Lock();
    return st;
}

/* ---- 对外接口 (与原逻辑完全一致) ---- */

void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg);

    if (out_addr)   *out_addr   = cfg.addr;
    if (out_freq)   *out_freq   = cfg.samp_freq_hz;
    if (out_points) *out_points = cfg.points;
}

void Flash_ReadOTAInfo(uint32_t* out_flag, uint32_t* out_len)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg);
    
    if (out_flag) *out_flag = cfg.ota_flag;
    if (out_len)  *out_len  = cfg.fw_len;
}

HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg); // 读旧数据保留 OTA 标志

    cfg.addr   = addr;
    cfg.samp_freq_hz = freq;
    cfg.points = points;

    return Flash_ProgramWholeConfig(&cfg);
}

HAL_StatusTypeDef Flash_SetOTAInfo(uint32_t flag, uint32_t len)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg); // 读旧数据保留配置参数

    cfg.ota_flag = flag;
    cfg.fw_len   = len;

    return Flash_ProgramWholeConfig(&cfg);
}

// 兼容接口
uint8_t Flash_ReadDeviceAddr(void) {
    uint8_t a; Flash_ReadConfig(&a, NULL, NULL); return a;
}
HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(new_addr, f, p);
}
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(a, new_freq, p);
}
HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(a, f, new_points);
}