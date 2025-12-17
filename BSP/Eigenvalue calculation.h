#ifndef EIGENVALUE_CALCULATION_H_
#define EIGENVALUE_CALCULATION_H_
#include "main.h"
#include "arm_math.h"

// ================== 参数定义 ==================
// KX134-1211 Range +/- 64g
// int16 范围 -32768 ~ +32767
// Sensitivity = 32768 / 64 = 512 LSB/g
#define KX134_SENSITIVITY_LSB_G   512.0f
#define KX134_SENSITIVITY         (1.0f / KX134_SENSITIVITY_LSB_G)

// FFT 配置
#define FFT_LEN           FFT_POINTS
#define SAMPLE_FREQ       25600.0f  // 25.6kHz

// ================== 数据结构 ==================
typedef struct
{
    float mean;                // 均值（4字节）
    float rms;                 // RMS（4字节）
    float pp;                  // 峰-峰值（4字节）
		float kurt;								 // 峭度（4字节）
		float peakFreq;						 // 主峰频率（4字节）
		float peakAmp;					   // 主频幅值（4字节）
		float amp2x;							 // 2x转频幅值（4字节）
    float envelope_vrms;       // 包络有效值（4字节）
    float envelope_peak;       // 包络峰值（4字节）
} AxisFeatureValue;
extern AxisFeatureValue X_data,Y_data,Z_data;
extern float g_z_offset_g;

void Calc_Init(void);// 用于在上电时调用一次，负责 FFT 表初始化和滤波器初始化
void Process_Data(int16_t *pRawData);
void print_FEATURE();

															

#endif /* EIGENVALUE_CALCULATION_H_ */
