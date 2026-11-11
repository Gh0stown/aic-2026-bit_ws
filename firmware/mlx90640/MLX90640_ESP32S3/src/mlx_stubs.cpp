#include <stdint.h>

/*
 * ESP32 端只调用 MLX90640_API 的解算函数，不通过 API 读 I2C，
 * 但链接时这些符号必须存在，这里给空实现。
 */
extern "C" {

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

}
