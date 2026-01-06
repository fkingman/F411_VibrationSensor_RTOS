#include "Eigenvalue calculation.h"
#include <string.h>

static arm_rfft_fast_instance_f32 S_rfft;
static float32_t fftBuf[FFT_POINTS]; 
AxisFeatureValue X_data,Y_data,Z_data;
//static float g_WaveZ_Live[FFT_POINTS]; 
static float g_WaveZ_Tx[FFT_POINTS];
volatile uint8_t g_SnapshotReq = 0;       

float g_z_offset_g  = 0.0f;   // 0g 偏移

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
#define CFFT (&arm_cfft_sR_f32_len1024)

//计算初始化函数
void Calc_Init(void)
{
    arm_rfft_fast_init_f32(&S_rfft, FFT_POINTS);
}

void Create_Wave_Snapshot(void)
{
    // 进入临界区，防止拷贝到一半被算法任务打断
    //taskENTER_CRITICAL(); 
    //memcpy(g_WaveZ_Tx, g_WaveZ_Live, sizeof(g_WaveZ_Live));
    //taskEXIT_CRITICAL();
    g_SnapshotReq = 1;
}

const float* Algo_Get_Snapshot_Ptr(void)
{
    return g_WaveZ_Tx;
}

void Z_Calib_Z_Upright_Neg1G(float *gBuf, uint32_t N)
{
    float sum_g = 0.0f;

    for (uint32_t i = 0; i < N; ++i)
    {
        sum_g += gBuf[i];
    }

    float mean_current = sum_g / (float)N;

    const float target_g = -1.0f;
  
    g_z_offset_g += (mean_current - target_g);
}
/*
static inline CFFT_PTR_T pick_cfft_u32(uint32_t N)
{
    switch (N) {
        case 16:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len16;
        case 32:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len32;
        case 64:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len64;
        case 128:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len128;
        case 256:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len256;
        case 512:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len512;
        case 1024: return (CFFT_PTR_T)&arm_cfft_sR_f32_len1024;
        case 2048: return (CFFT_PTR_T)&arm_cfft_sR_f32_len2048;
        case 4096: return (CFFT_PTR_T)&arm_cfft_sR_f32_len4096;
        default:   return (CFFT_PTR_T)0;
    }
}*/

static void Remove_DC(float *data, uint32_t len)
{
    float sum = 0.0f;
    
    // 1. 计算均值
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    float mean = sum / (float)len;

    // 2. 减去均值
    for (uint32_t i = 0; i < len; i++) {
        data[i] -= mean;
    }
}

//时域特征计算 (Mean, RMS, PP, Kurt)
static void Calc_TimeDomain_Only(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    float32_t sum = 0.0f;
    float32_t sumSq = 0.0f;
    float32_t minVal = data[0];
    float32_t maxVal = data[0];
    float32_t mean, rms, pp, kurt;

    for (uint32_t i = 0; i < len; i++) {
        float32_t val = data[i];
        sum += val;
        sumSq += val * val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    mean = sum / (float32_t)len;
    rms = sqrtf(sumSq / (float32_t)len); // 这里是包含直流分量的 RMS
    pp = maxVal - minVal;

    // 第二遍循环：计算峭度 (Kurtosis) 需要中心矩
    // Kurtosis = E[(x - u)^4] / (sigma^4)
    float32_t m2 = 0.0f; // 二阶中心矩 (方差 * N)
    float32_t m4 = 0.0f; // 四阶中心矩

    for (uint32_t i = 0; i < len; i++) {
        float32_t diff = data[i] - mean;
        float32_t diff2 = diff * diff;
        m2 += diff2;
        m4 += diff2 * diff2;
    }
    if (m2 < 1e-9f) {
        kurt = 0.0f;
    } else {
        // 公式调整：N * m4 / (m2^2)
        kurt = ((float32_t)len * m4) / (m2 * m2);
    }
    result->mean = mean;
    result->rms  = rms;
    result->pp   = pp;
    result->kurt = kurt;
}
//积分
static void Integrate_Acc_To_Vel(float *data, uint32_t len)
{
    float dt = 1.0f / SAMPLE_FREQ; // 采样间隔 (例如 1/25600)
    float vel = 0.0f;
    float val_prev = data[0]; // 上一个点的加速度
    
    // 重力加速度常数: 1g ≈ 9806.65 mm/s²
    const float G_TO_MM_S2 = 9806.65f; 

    for (uint32_t i = 0; i < len; i++) {
        float val_curr = data[i];
        
        // 梯形积分公式: v = v + (a1 + a2) * dt / 2
        // 转换单位: g -> mm/s
        vel += (val_prev + val_curr) * 0.5f * dt * G_TO_MM_S2;
        
        val_prev = val_curr;
        
        // 原地覆盖：现在 data[i] 变成速度了 (mm/s)
        data[i] = vel;
    }

    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) sum += data[i];
    float mean = sum / (float)len;

    for (uint32_t i = 0; i < len; i++) {
        data[i] -= mean;
    }
    
}

static float Calc_RMS_Only(float *data, uint32_t len)
{
    float sumSq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        sumSq += data[i] * data[i];
    }
    return sqrtf(sumSq / (float)len);
}

//频域特征计算 (Z轴: PeakFreq, PeakAmp, 2xAmp)
static void Calc_FreqDomain_Z(float32_t *data, uint32_t len)
{
    // 注意：arm_rfft_fast_f32 输出格式是 packed 格式，需要临时 buffer 或原地处理
    // 这里为了省内存，我们复用 fftBuf 作为输出，但这需要小心处理
    // RFFT 的输出也是 len 长度的 float
    
    // 执行 RFFT
    // data 是输入 (时域)，也是输出 (频域 packed)
    arm_rfft_fast_f32(&S_rfft, data, data, 0);

    // 计算幅值 (Modulus)
    // 输入 len 个 float (复数 packed)，计算出 len/2 个幅值
    arm_cmplx_mag_f32(data, data, len / 2);

    // 归一化 & 找峰值
    float32_t norm = 2.0f / (float32_t)len;
    data[0] /= (float32_t)len; // DC
    
    float32_t maxAmp = 0.0f;
    uint32_t maxIndex = 0;

    for (uint32_t i = 1; i < len / 2; i++) {
        data[i] *= norm;
        
        // 避开 5 个点以内的低频干扰
        if (i > 5 && data[i] > maxAmp) {
            maxAmp = data[i];
            maxIndex = i;
        }
    }

    float32_t freq_res = SAMPLE_FREQ / (float32_t)len;
    Z_data.peakFreq = (float32_t)maxIndex * freq_res;
    Z_data.peakAmp  = maxAmp;

    // 2x 频
    uint32_t idx_2x = maxIndex * 2;
    if (idx_2x < len / 2) Z_data.amp2x = data[idx_2x];
    else Z_data.amp2x = 0.0f;
}
//去直流取绝对值
static void Remove_DC_And_Rectify(float *data, uint32_t len)
{
    Remove_DC(data, len);

    for (uint32_t i = 0; i < len; i++) {
        if (data[i] < 0.0f) {
            data[i] = -data[i];
        }
    }
}
//包络
static void Calc_Envelope_Stats(float32_t *env_data, uint32_t len)
{
    float32_t sum = 0.0f;
    float32_t sumSq = 0.0f;
    float32_t maxVal = 0.0f;
    
    for (uint32_t i = 0; i < len; i++) {
        float32_t val = env_data[i];
        
        // 累加计算均值 (对于包络信号，均值反映了整体噪声水平)
        sum += val;
        
        // 累加计算 RMS (能量)
        sumSq += val * val;
        
        // 寻找峰值 (冲击最大值)
        if (val > maxVal) maxVal = val;
    }
    // 包络 RMS
    Z_data.envelope_vrms = sqrtf(sumSq / (float32_t)len);
    
    // 包络峰值 (反映轴承缺陷的冲击强度)
    Z_data.envelope_peak = maxVal;
}


/*void print_FEATURE(void)
{
    printf("========== Vibration Analysis Result ==========\r\n");
    printf("[X-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", X_data.mean, X_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   X_data.pp,   X_data.kurt);

    printf("[Y-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", Y_data.mean, Y_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   Y_data.pp,   Y_data.kurt);

    printf("[Z-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", Z_data.mean, Z_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   Z_data.pp,   Z_data.kurt);
    printf("[Z-Freq] Main Freq = %5.1f Hz   Peak Amp = %.4f g\r\n", 
           Z_data.peakFreq, Z_data.peakAmp);
    printf("         2x Amp    = %.4f g\r\n", Z_data.amp2x);
    printf("[Z-Enve] Env Vrms  = %.3f g     Env Peak = %.3f g\r\n", 
           Z_data.envelope_vrms, Z_data.envelope_peak);
    printf("===============================================\r\n\n");
}*/
	
void Process_Data(int16_t *pRawData)
{	  
    for (int i = 0; i < FFT_POINTS; i++) {
        // 解交错 + 转换 float + 物理量变换
        fftBuf[i] = (float)pRawData[i * 3 + 0] * KX134_SENSITIVITY;
    }
    Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &X_data);

    // --- 处理 Y 轴 ---
    for (int i = 0; i < FFT_POINTS; i++) {
        fftBuf[i] = (float)pRawData[i * 3 + 1] * KX134_SENSITIVITY;
    }
    Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &Y_data);

    // --- 处理 Z 轴 (含频域) ---
    for (int i = 0; i < FFT_POINTS; i++) {
        float val = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;
        fftBuf[i] = val;
        //g_WaveZ_Live[i] = val;
    }
    Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &Z_data);
    Z_data.mean =  Z_data.mean - 1;
    Remove_DC(fftBuf, FFT_POINTS);
    //Apply_Median_Filter_3(fftBuf, FFT_POINTS);
    //快照
    if (g_SnapshotReq == 1) {
        taskENTER_CRITICAL();
        memcpy(g_WaveZ_Tx, fftBuf, FFT_POINTS * sizeof(float));
        taskEXIT_CRITICAL();
        g_SnapshotReq = 0; 
    }
    Calc_FreqDomain_Z(fftBuf, FFT_POINTS);

    for (int i = 0; i < FFT_POINTS; i++) {
        fftBuf[i] = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;
    }
    Remove_DC(fftBuf, FFT_POINTS);       
    //Apply_Median_Filter_3(fftBuf, FFT_POINTS); // 去毛刺
    Integrate_Acc_To_Vel(fftBuf, FFT_POINTS);       // 积分为速度
    Z_data.rms = Calc_RMS_Only(fftBuf, FFT_POINTS); // 覆盖为速度 RMS

    //fftbuf为频谱数据
    for (int i = 0; i < FFT_POINTS; i++) {
        // 再次从源头读取
        float val = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;
        fftBuf[i] = val;
    }
    Remove_DC_And_Rectify(fftBuf, FFT_POINTS);
    Calc_Envelope_Stats(fftBuf, FFT_POINTS);
}


/*Test
void print_g_data(float *buf, uint32_t N)
{
    Eigen_Separate_And_Convert(ADC_Buffer_Z, ADC_Buffer_XY);
    for (uint32_t i = 0; i < N; i++) 
	{
        printf("%.3f \n", buf[i]);
	}
}*/



