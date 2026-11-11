/*
 * MLX90640 原始数据转发 + 本地 OLED 显示 —— ESP32-S3-N16R8 版
 *
 * 角色：ESP32 读原始帧、打包、从串口发走；温度解算在 ROS 侧。
 *       串口协议与 STM32 版完全一致（AA 55 | TYPE | SEQ | LEN | PAYLOAD | CRC16），
 *       上位机/ROS 侧的解析代码一行都不用改。
 *
 * 接线：
 *   MLX90640  VCC -> 3V3      GND -> GND
 *             SDA -> GPIO8    SCL -> GPIO9
 *   OLED      VCC -> 3V3      GND -> GND
 *             SDA -> GPIO8    SCL -> GPIO9     （和 MLX 共用一条 I2C，地址 0x3C / 0x33 不冲突）
 *   串口：板子上的 UART USB-C 口（板载 CH340）接 ROS 机，460800 8N1
 *
 * 编译：Arduino IDE，开发板 "ESP32S3 Dev Module"
 *       Flash Size 16MB, PSRAM "OPI PSRAM"（N16R8）
 *       USB CDC On Boot: Disabled（走 UART 口）
 *
 * 注意：GPIO19/20 是原生 USB，GPIO0/45/46 是 strapping，GPIO26~37 被 flash/PSRAM 占用，
 *       这些都不要用来接 I2C。
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "src/MLX90640_API.h"

#define MLX_ADDR      0x33
#define OLED_ADDR     0x3C
#define PIN_SDA       8
#define PIN_SCL       9
#define SERIAL_BAUD   460800

#define REG_EEPROM    0x2400
#define REG_FRAME     0x0400
#define REG_STATUS    0x8000
#define REG_CTRL      0x800D

#define EE_WORDS      832
#define FRAME_WORDS   834
#define CHUNK_WORDS   32      /* 32 字 = 64 字节，避免超过 Wire 默认缓冲 */

Adafruit_SSD1306 display(128, 64, &Wire, -1);

static uint16_t ee[EE_WORDS];
static uint16_t frame[FRAME_WORDS];

static paramsMLX90640 params;
static float to[768];
static float curMax = 0, curMin = 0, curAmb = 0;
static bool  hasData = false;
static bool  displayOk = false;

static uint8_t  seq = 0;
static uint32_t frames = 0;
static uint32_t errors = 0;
static uint32_t lastStatus = 0;
static bool     eeOk = false;
static uint8_t  beat = 0;

/* API 需要的 I2C 桩函数放在 src/mlx_stubs.cpp，避免 extern "C" 影响 setup/loop 的链接属性 */

/* ---------------- I2C 底层 ---------------- */

static bool mlxReadWords(uint16_t reg, uint16_t *out, uint16_t words)
{
    uint16_t done = 0;

    while (done < words)
    {
        uint16_t n = words - done;
        if (n > CHUNK_WORDS) n = CHUNK_WORDS;

        uint16_t r = reg + done;

        Wire.beginTransmission(MLX_ADDR);
        Wire.write((uint8_t)(r >> 8));
        Wire.write((uint8_t)(r & 0xFF));
        if (Wire.endTransmission(false) != 0)
            return false;

        uint16_t want = (uint16_t)(n * 2);
        uint16_t got = Wire.requestFrom((uint16_t)MLX_ADDR, (size_t)want, (bool)true);
        if (got != want)
            return false;

        for (uint16_t i = 0; i < n; i++)
        {
            uint8_t hi = (uint8_t)Wire.read();
            uint8_t lo = (uint8_t)Wire.read();
            out[done + i] = ((uint16_t)hi << 8) | lo;
        }
        done += n;
    }
    return true;
}

static bool mlxWriteWord(uint16_t reg, uint16_t val)
{
    Wire.beginTransmission(MLX_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
}

static bool mlxWaitData(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    uint16_t st;

    while (millis() - t0 < timeout_ms)
    {
        if (mlxReadWords(REG_STATUS, &st, 1) && (st & 0x0008))
            return true;
        delay(1);
    }
    return false;
}

static void mlxSetRefreshRate(uint8_t code)   /* 0x02 = 2 Hz */
{
    uint16_t ctrl;

    if (mlxReadWords(REG_CTRL, &ctrl, 1))
    {
        ctrl = (uint16_t)((ctrl & ~0x0380U) | ((uint16_t)(code & 0x07U) << 7));
        mlxWriteWord(REG_CTRL, ctrl);
    }
}

static void mlxSetChessMode(void)
{
    uint16_t ctrl;

    if (mlxReadWords(REG_CTRL, &ctrl, 1))
    {
        ctrl |= (uint16_t)(1U << 12);
        mlxWriteWord(REG_CTRL, ctrl);
    }
}

/* ---------------- 协议（与 STM32 版一致） ---------------- */

static uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void sendByteCrc(uint8_t b, uint16_t *crc)
{
    *crc = crc16_update(*crc, &b, 1);
    Serial.write(b);
}

static uint16_t sendHeader(uint8_t type, uint16_t len)
{
    uint8_t hdr[6];
    uint16_t crc = 0xFFFFU;

    hdr[0] = 0xAA;
    hdr[1] = 0x55;
    hdr[2] = type;
    hdr[3] = seq++;
    hdr[4] = (uint8_t)(len & 0xFF);
    hdr[5] = (uint8_t)(len >> 8);

    crc = crc16_update(crc, &hdr[2], 4);
    Serial.write(hdr, 6);
    return crc;
}

static void sendTail(uint16_t crc)
{
    uint8_t tail[2];
    tail[0] = (uint8_t)(crc & 0xFF);
    tail[1] = (uint8_t)(crc >> 8);
    Serial.write(tail, 2);
}

static void sendEeprom(void)
{
    uint16_t crc = sendHeader(0x01, (uint16_t)(EE_WORDS * 2));

    for (uint16_t i = 0; i < EE_WORDS; i++)
    {
        sendByteCrc((uint8_t)(ee[i] & 0xFF), &crc);
        sendByteCrc((uint8_t)(ee[i] >> 8), &crc);
    }
    sendTail(crc);
}

static void sendFrame(uint32_t ms, uint8_t subpage)
{
    uint16_t crc = sendHeader(0x02, (uint16_t)(6 + FRAME_WORDS * 2));

    sendByteCrc((uint8_t)(ms & 0xFF), &crc);
    sendByteCrc((uint8_t)((ms >> 8) & 0xFF), &crc);
    sendByteCrc((uint8_t)((ms >> 16) & 0xFF), &crc);
    sendByteCrc((uint8_t)((ms >> 24) & 0xFF), &crc);
    sendByteCrc(subpage, &crc);
    sendByteCrc(0, &crc);

    for (uint16_t i = 0; i < FRAME_WORDS; i++)
    {
        sendByteCrc((uint8_t)(frame[i] & 0xFF), &crc);
        sendByteCrc((uint8_t)(frame[i] >> 8), &crc);
    }
    sendTail(crc);
}

static void sendStatus(uint32_t ms, uint32_t fr, uint32_t er)
{
    uint8_t p[11];
    uint16_t crc;

    p[0] = (uint8_t)(ms & 0xFF);
    p[1] = (uint8_t)((ms >> 8) & 0xFF);
    p[2] = (uint8_t)((ms >> 16) & 0xFF);
    p[3] = (uint8_t)((ms >> 24) & 0xFF);

    p[4] = (uint8_t)(fr & 0xFF);
    p[5] = (uint8_t)((fr >> 8) & 0xFF);
    p[6] = (uint8_t)((fr >> 16) & 0xFF);
    p[7] = (uint8_t)((fr >> 24) & 0xFF);

    p[8] = (uint8_t)(er & 0xFF);
    p[9] = (uint8_t)((er >> 8) & 0xFF);

    p[10] = 0;

    crc = sendHeader(0x03, 11);
    Serial.write(p, 11);
    crc = crc16_update(crc, p, 11);
    sendTail(crc);
}

/* ---------------- OLED ---------------- */

static void renderDisplay(void)
{
    if (!displayOk)
        return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print("MLX BRIDGE");
    beat ^= 1;
    if (beat)
        display.print(" *");

    if (!eeOk)
    {
        display.setCursor(0, 24);
        display.print("EE INVALID");
    }
    else if (!hasData)
    {
        display.setCursor(0, 24);
        display.print("waiting...");
    }
    else
    {
        display.setCursor(0, 16);
        display.print("MAX ");
        display.print(curMax, 1);

        display.setCursor(0, 32);
        display.print("MIN ");
        display.print(curMin, 1);

        display.setCursor(0, 48);
        display.print("AMB ");
        display.print(curAmb, 1);
    }
    display.display();
}

/* 上位机命令：0x01 重发 EEPROM，0x02 立即发状态 */
static void handleHostCommand(void)
{
    while (Serial.available() > 0)
    {
        int c = Serial.read();
        if (c == 0x01)
            sendEeprom();
        else if (c == 0x02)
            sendStatus(millis(), frames, errors);
    }
}

/* ---------------- 主流程 ---------------- */

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(50);

    Wire.begin(PIN_SDA, PIN_SCL, 400000);
    delay(100);

    /* 第 4 个参数 false：不要再让库调用 Wire.begin()，否则可能把引脚重置成默认值 */
    displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false);
    if (displayOk)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("MLX BRIDGE");
        display.setCursor(0, 24);
        display.println("init...");
        display.display();
    }

    delay(100);

    uint32_t sum = 0;
    eeOk = mlxReadWords(REG_EEPROM, ee, EE_WORDS);
    if (eeOk)
    {
        for (uint16_t i = 0; i < EE_WORDS; i++)
            sum += ee[i];
        if (sum == 0 || sum == (uint32_t)EE_WORDS * 0xFFFFU)
            eeOk = false;
    }

    if (eeOk)
    {
        if (MLX90640_ExtractParameters(ee, &params) != 0)
            eeOk = false;
    }

    if (eeOk)
    {
        sendEeprom();
        mlxSetRefreshRate(0x02);   /* 2 Hz */
        mlxSetChessMode();
    }

    renderDisplay();
}

void loop()
{
    handleHostCommand();

    if (eeOk)
    {
        if (mlxWaitData(700))
        {
            if (mlxReadWords(REG_FRAME, frame, 832))   /* 只读 832 个像素字 */
            {
                uint16_t ctrl = 0, status = 0;

                /* frame[832]/[833] 不是帧 RAM 里的连续字，必须单独读 */
                mlxReadWords(REG_CTRL, &ctrl, 1);       /* 0x800D 控制寄存器 */
                mlxReadWords(REG_STATUS, &status, 1);   /* 0x8000 状态寄存器 */
                frame[832] = ctrl;
                frame[833] = (uint16_t)(status & 0x0001U);

                mlxWriteWord(REG_STATUS, 0x0030);   /* 清 data-ready */
                frames++;

                /* 先发原始数据，再做本地显示，保证上位机延迟最小 */
                sendFrame(millis(), (uint8_t)(frame[833] & 0x01U));

                curAmb = MLX90640_GetTa(frame, &params);
                MLX90640_CalculateTo(frame, &params, 0.95f, curAmb - 8.0f, to);

                curMax = -100.0f;
                curMin = 200.0f;
                for (int i = 0; i < 768; i++)
                {
                    if (to[i] > curMax) curMax = to[i];
                    if (to[i] < curMin) curMin = to[i];
                }
                hasData = true;
                renderDisplay();
            }
            else
            {
                errors++;
            }
        }
        else
        {
            errors++;
        }
    }

    if (millis() - lastStatus >= 1000U)
    {
        lastStatus = millis();
        sendStatus(lastStatus, frames, errors);
    }
}
