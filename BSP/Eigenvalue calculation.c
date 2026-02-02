#include "Eigenvalue calculation.h"
#include <string.h>

#define MIN_VALID_PEAK_AMP  0.04f	//主频幅值门限
#define INT_ACC_DEADZONE  0.08f	//噪声积分门限

static float g_hpf_alpha = 0.99f; // 高通系数
static float g_lpf_alpha = 0.8f; // 低通系数
static arm_rfft_fast_instance_f32 S_rfft;
static float32_t fftBuf[FFT_POINTS]; 
AxisFeatureValue X_data, Y_data,Z_data;
//static float g_WaveZ_Live[FFT_POINTS]; 
static float g_WaveZ_Tx[FFT_POINTS];
volatile uint8_t g_SnapshotReq = 0;       

float g_z_offset_g  = 0.0f;   // 0g 偏移

//extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
//#define CFFT (&arm_cfft_sR_f32_len1024)



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
//中值滤波
static void Apply_Median_Filter_3(float *data, uint32_t len)
{
    // 只处理中间部分

    float prev = data[0];
    float curr = data[1];
    float next;
    
    for(uint32_t i = 1; i < len - 1; i++)
    {
        next = data[i+1]; // 预读取下一个点
        
        // 核心逻辑：找 prev, curr, next 三者中的中间值
        float median;
        
        if (prev > curr) {
            if (curr > next) {
                median = curr;      // prev > curr > next
            } else if (prev > next) {
                median = next;      // prev > next > curr
            } else {
                median = prev;      // next > prev > curr
            }
        } else { // curr >= prev
            if (curr < next) {
                median = curr;      // next > curr >= prev
            } else if (prev < next) {
                median = next;      // curr >= next > prev
            } else {
                median = prev;      // curr >= prev >= next
            }
        }
        
        // 更新当前点
        data[i] = median;
                
        prev = data[i]; // 使用滤波后的值作为下一轮的 prev (IIR特性，去噪更强)
        curr = next;    // 加载新的 curr
    }
}

//低通系数更新
void Algo_Update_LPF_Coeff(uint16_t sample_rate_hz)
{
    // 目标截止频率: 1000Hz
    const float cut_off_freq = 1000.0f;
    const float pi_2 = 6.2831853f; // 2 * Pi
    
    if (sample_rate_hz == 0) return; // 防止除零

    // 公式: alpha = Fs / (Fs + 2*pi*Fc)
    float denom = (float)sample_rate_hz + (pi_2 * cut_off_freq);
    g_lpf_alpha = (float)sample_rate_hz / denom;
    
}

// 1kHz 低通滤波器
static void LowPassFilter_1kHz(float *data, uint32_t len)
{
    // 初始化：用第0个点作为初始状态，避免滤波器启动时的跳变
    float val_prev = data[0]; 
    
    for (uint32_t i = 1; i < len; i++)
    {
        float val_curr = data[i];
        
        // 核心公式：输出 = (系数 * 上一次输出) + ((1-系数) * 本次输入)
        // 这里的 LPF_ALPHA (0.8) 代表“惯性”，即 80% 保持原样，只接受 20% 的新变化
        float val_out = g_lpf_alpha * val_prev + (1.0f - g_lpf_alpha) * val_curr;
        
        // 更新数据：原地覆盖，节省内存
        data[i] = val_out;
        
        // 保存当前输出供下一次迭代使用
        val_prev = val_out;
    }
}
//高通系数更新
void Algo_Update_HPF_Coeff(uint16_t sample_rate_hz)
{
    const float cut_off_freq = 10.0f;
    const float pi_2 = 6.2831853f; 

    if (sample_rate_hz == 0) return;

    // Alpha = RC / (RC + dt)
    // RC = 1 / (2 * pi * fc)
    // dt = 1 / Fs
    // Alpha = Fs / (Fs + 2*pi*fc)
    
    float denom = (float)sample_rate_hz + (pi_2 * cut_off_freq);
    g_hpf_alpha = (float)sample_rate_hz / denom;
    
}
//10hz高通
/*static void HighPassFilter_10Hz(float *data, uint32_t len)
{
    float alpha = g_hpf_alpha; // 使用动态计算的系数
    float last_in = data[0];
    float last_out = 0.0f; // 初始状态假设为0 (去直流后通常接近0)
    
    // 首点处理：简单置零或设为 data[0] 去直流
    data[0] = 0.0f; 
    
    for(uint32_t i = 1; i < len; i++) {
        float input = data[i];
        
        // 核心差分公式
        // 理解：(input - last_in) 是信号的变化量
        float output = alpha * (last_out + input - last_in);
        
        data[i] = output; // 原地更新
        
        last_out = output;
        last_in = input;
    }
}*/
//强力去噪固定0.99
static void HighPassFilter_10Hz(float *data, uint32_t len)
{
    // 强力去噪模式：固定系数 0.99
    // 在 25.6kHz 下，截止频率约为 40Hz
    // 这能让速度 RMS 从 0.78 降到 0.2 左右，接近 MATLAB 结果
    float alpha = 0.99755f; 
    
    float last_in = data[0];
    float last_out = 0.0f;
    data[0] = 0.0f;
    
    for(uint32_t i = 1; i < len; i++) {
        float input = data[i];
        float output = alpha * (last_out + input - last_in);
        data[i] = output;
        last_out = output;
        last_in = input;
    }
}

//计算初始化函数
void Calc_Init(void)
{
    arm_rfft_fast_init_f32(&S_rfft, FFT_POINTS);
		Algo_Update_LPF_Coeff(g_cfg_freq_hz);
		Algo_Update_HPF_Coeff(g_cfg_freq_hz); // 10Hz 高通
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
    result->rms  = sqrtf(m2 / (float32_t)len);;
    result->pp   = pp;
    result->kurt = kurt;
}


//积分
static void Integrate_Acc_To_Vel(float *data, uint32_t len)
{
    if (g_cfg_freq_hz == 0) return;
    float dt = 1.0f / (float)g_cfg_freq_hz; // 采样间隔 (例如 1/25600)
    float vel = 0.0f;
    float val_prev = data[0]; // 上一个点的加速度
    
    //HighPassFilter_10Hz(data, FFT_POINTS);  

    // 重力加速度常数: 1g ≈ 9806.65 mm/s²
    const float G_TO_MM_S2 = 9806.65f; 

    for (uint32_t i = 0; i < len; i++) {
        float val_curr = data[i];
        
				if (fabsf(val_curr) < INT_ACC_DEADZONE) { //噪声积分门限
            val_curr = 0.0f;
        }
			
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

static void Calc_RMS_Only(float *data, uint32_t len , AxisFeatureValue *result)
{
    float sumSq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        sumSq += data[i] * data[i];
    }
    result->rms =  sqrtf(sumSq / (float)len);
}

//频域特征计算 (Z轴: PeakFreq, PeakAmp, 2xAmp)
static void Calc_FreqDomain_Z(float32_t *data, uint32_t len , AxisFeatureValue *result)
{   
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
		if (maxAmp < MIN_VALID_PEAK_AMP) {
		// 如果最大值都没超过门限，说明是静置噪音
		result->peakFreq = 0.0f; // 强制置零
		result->peakAmp  = 0.0f; // 或者保留 maxAmp 作为底噪参考，看你需求
		result->amp2x    = 0.0f;
		} else {
    float32_t freq_res = g_cfg_freq_hz / (float32_t)len;
    result->peakFreq = (float32_t)maxIndex * freq_res;
    result->peakAmp  = maxAmp;

    // 2x 频
		uint32_t idx_2x = maxIndex * 2;
		float32_t amp2x_val = 0.0f;

		// 在左右各搜 1 个点 (共3个点) 防止泄露导致的峰值偏移
		if (idx_2x > 1 && idx_2x < (len/2 - 1)) {
				float32_t a = data[idx_2x - 1];
				float32_t b = data[idx_2x];
				float32_t c = data[idx_2x + 1];
				// 取三者最大
				amp2x_val = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
		} else if (idx_2x < len/2) {
				amp2x_val = data[idx_2x];
		}
		result->amp2x = amp2x_val;
		}
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
static void Calc_Envelope_Stats(float32_t *env_data, uint32_t len, AxisFeatureValue *result)
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
    //result->envelope_vrms = sqrtf(sumSq / (float32_t)len);
    result->envelope_vrms = sqrtf(sumSq / (float32_t)len) * 1.414f;
    
    // 包络峰值 (反映轴承缺陷的冲击强度)
    result->envelope_peak = maxVal;
}


void Process_Data(int16_t *pRawData)
{	  
    for (int i = 0; i < FFT_POINTS; i++) {
        fftBuf[i] = (float)pRawData[i * 3 + 0] * KX134_SENSITIVITY;
    }
		Apply_Median_Filter_3(fftBuf, FFT_POINTS);//去毛刺
		LowPassFilter_1kHz(fftBuf, FFT_POINTS);		//1khz低通
		Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &X_data);
		HighPassFilter_10Hz(fftBuf, FFT_POINTS);		//10hz高通		
		Remove_DC(fftBuf, FFT_POINTS);             // 去直流
    Integrate_Acc_To_Vel(fftBuf, FFT_POINTS);       
    Calc_RMS_Only(fftBuf, FFT_POINTS, &X_data);

    // --- 处理 Y 轴 ---
    for (int i = 0; i < FFT_POINTS; i++) {
        fftBuf[i] = (float)pRawData[i * 3 + 1] * KX134_SENSITIVITY;
    }
		Apply_Median_Filter_3(fftBuf, FFT_POINTS);//去毛刺
		LowPassFilter_1kHz(fftBuf, FFT_POINTS);		//1khz低通
		Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &Y_data);
		HighPassFilter_10Hz(fftBuf, FFT_POINTS);		//10hz高通	
		Remove_DC(fftBuf, FFT_POINTS);             // 去直流		
    Integrate_Acc_To_Vel(fftBuf, FFT_POINTS);      
    Calc_RMS_Only(fftBuf, FFT_POINTS, &Y_data);

    // --- 处理 Z 轴 (含频域) ---
    for (int i = 0; i < FFT_POINTS; i++) {
        fftBuf[i] = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;        
    }
		Apply_Median_Filter_3(fftBuf, FFT_POINTS);
		Remove_DC(fftBuf, FFT_POINTS);             // 去直流
    if (g_SnapshotReq == 1) {//快照保存高频
        taskENTER_CRITICAL();
        memcpy(g_WaveZ_Tx, fftBuf, FFT_POINTS * sizeof(float));
        taskEXIT_CRITICAL();
        g_SnapshotReq = 0; 
    }
		Calc_FreqDomain_Z(fftBuf, FFT_POINTS, &Z_data);//fft计算
		for (int i = 0; i < FFT_POINTS; i++) {//重新取
        fftBuf[i] = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;
    }
		Apply_Median_Filter_3(fftBuf, FFT_POINTS);
    LowPassFilter_1kHz(fftBuf, FFT_POINTS);
		Calc_TimeDomain_Only(fftBuf, FFT_POINTS, &Z_data);
		Z_data.mean = 	Z_data.mean - 1;	
		HighPassFilter_10Hz(fftBuf, FFT_POINTS);   // 10Hz 高通
		Remove_DC(fftBuf, FFT_POINTS);             // 去直流
    Integrate_Acc_To_Vel(fftBuf, FFT_POINTS);  // 积分
    Calc_RMS_Only(fftBuf, FFT_POINTS, &Z_data);// 速度 RMS

		for (int i = 0; i < FFT_POINTS; i++) {//包络再重新取
        fftBuf[i] = (float)pRawData[i * 3 + 2] * KX134_SENSITIVITY;
    }
    Remove_DC_And_Rectify(fftBuf, FFT_POINTS);
    Calc_Envelope_Stats(fftBuf, FFT_POINTS, &Z_data);
}



