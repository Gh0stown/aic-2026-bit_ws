/*
 * MLX90640 解算库的 C 包装，给 Python ctypes 调用。
 *
 * 编译（Ubuntu 20.04）：
 *   gcc -O2 -fPIC -shared -o libmlx90640.so mlx_wrap.c MLX90640_API.c -lm
 */

#include <stdint.h>
#include "MLX90640_API.h"

/* 主机侧不做 I2C，给厂商 API 提供空实现 */
int MLX90640_I2CRead(uint8_t a, uint16_t r, uint16_t n, uint16_t *d)
{
    (void)a; (void)r; (void)n; (void)d;
    return -1;
}

int MLX90640_I2CWrite(uint8_t a, uint16_t r, uint16_t v)
{
    (void)a; (void)r; (void)v;
    return -1;
}

void MLX90640_I2CInit(void) { }
void MLX90640_I2CFreqSet(int f) { (void)f; }

static paramsMLX90640 g_params;
static int g_ready = 0;

/* 收到 EEPROM 包后调用一次。返回 0 成功。 */
int mlx_init(const uint16_t *ee)
{
    if (!ee)
        return -1;

    if (MLX90640_ExtractParameters((uint16_t *)ee, &g_params) != 0)
    {
        g_ready = 0;
        return -2;
    }

    g_ready = 1;
    return 0;
}

/*
 * 每帧调用：
 *   frame        834 个 uint16（0..831 像素，832 控制寄存器，833 子页号）
 *   emissivity   0.95 左右
 *   tr_shift     Ta - tr_shift 作为反射温度，厂商默认 8.0
 *   out          768 个 float，行优先 24x32
 *   ta_out       返回环境温度，可为 NULL
 */
int mlx_frame_to_temp(const uint16_t *frame, float emissivity, float tr_shift,
                      float *out, float *ta_out)
{
    float ta;

    if (!g_ready || !frame || !out)
        return -1;

    ta = MLX90640_GetTa((uint16_t *)frame, &g_params);
    MLX90640_CalculateTo((uint16_t *)frame, &g_params, emissivity, ta - tr_shift, out);

    if (ta_out)
        *ta_out = ta;

    return 0;
}

/* 完整的两个子页解算后调用；控制寄存器 bit12: 1=chess, 0=interleaved。 */
void mlx_bad_pixels(float *to, uint16_t control_register)
{
    int mode = (control_register >> 12) & 1;

    if (g_ready && to)
    {
        MLX90640_BadPixelsCorrection(g_params.brokenPixels, to, mode, &g_params);
        MLX90640_BadPixelsCorrection(g_params.outlierPixels, to, mode, &g_params);
    }
}
