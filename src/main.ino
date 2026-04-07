#include <Wire.h>
#include <Arduino.h>

// ==================== 硬件配置 ====================

// I2C 多路复用器地址（7 位地址）
#define I2C_ADDR_1 0x70  // 第一片 PCA9548A（A0=A1=A2=GND）
#define I2C_ADDR_2 0x73  // 第二片 PCA9548A（A0=VCC, A1=A2=GND）

// I2C 总线引脚
#define SDA1_PIN 3       // I2C0 SDA
#define SCL1_PIN 8       // I2C0 SCL
#define SDA2_PIN 4       // I2C1 SDA
#define SCL2_PIN 5       // I2C1 SCL

// MMC5603 传感器地址
#define MMC5603_ADDR 0x30

// 采样频率
#define SENSOR_FREQ_HZ 120
#define SENSOR_PERIOD_US (1000000 / SENSOR_FREQ_HZ)  // ~8333µs

// ==================== 全局变量 ====================

uint32_t t_next = 0;

// 第二条 I2C 总线实例
TwoWire I2Cone = TwoWire(1);

// 数据缓冲区（16 个传感器的三轴数据）
struct SensorData {
  uint32_t x, y, z;
} sensor_data[16];

// ==================== I2C 操作函数 ====================

void writeReg(TwoWire &bus, uint8_t reg, uint8_t val) {
  bus.beginTransmission(MMC5603_ADDR);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}

uint8_t readReg(TwoWire &bus, uint8_t reg) {
  bus.beginTransmission(MMC5603_ADDR);
  bus.write(reg);
  bus.endTransmission(false);
  bus.requestFrom((uint8_t)MMC5603_ADDR, (uint8_t)1);
  return bus.read();
}

void readMulti(TwoWire &bus, uint8_t reg, uint8_t* buf, uint8_t len) {
  bus.beginTransmission(MMC5603_ADDR);
  bus.write(reg);
  bus.endTransmission(false);
  bus.requestFrom((uint8_t)MMC5603_ADDR, len);
  for (int i = 0; i < len; i++) {
    buf[i] = bus.read();
  }
}

// ==================== 多路复用器 ====================

void selectChannel(TwoWire &bus, uint8_t addr, uint8_t channel) {
  uint8_t controlByte = 1 << channel;
  bus.beginTransmission(addr);
  bus.write(controlByte);
  bus.endTransmission();
  delayMicroseconds(5);  // PCA9548A 切换延迟（< 1µs，留余量）
}

// ==================== 传感器操作 ====================

// 初始化所有 16 个传感器为连续测量模式（ODR=120Hz）
void initAllSensors() {
  for (uint8_t ch = 0; ch < 8; ch++) {
    // 总线 0：传感器 #0 ~ #7
    selectChannel(Wire, I2C_ADDR_1, ch);
    writeReg(Wire, 0x1C, 0x03);   // BW[1:0]=11，最大带宽（单通道 ~1.2ms）
    writeReg(Wire, 0x1A, SENSOR_FREQ_HZ);  // 设置 ODR = 120Hz
    writeReg(Wire, 0x1B, 0x20);   // Auto_SR_en=1（自动 SET/RESET 去磁）
    writeReg(Wire, 0x1B, 0xA0);   // bit7=Cmm_freq_en + bit5=Auto_SR_en
    writeReg(Wire, 0x1D, 0x10);   // bit4=Cmm_en（进入连续测量模式）
    delay(10);

    // 总线 1：传感器 #8 ~ #15
    selectChannel(I2Cone, I2C_ADDR_2, ch);
    writeReg(I2Cone, 0x1C, 0x03);
    writeReg(I2Cone, 0x1A, SENSOR_FREQ_HZ);
    writeReg(I2Cone, 0x1B, 0x20);
    writeReg(I2Cone, 0x1B, 0xA0);
    writeReg(I2Cone, 0x1D, 0x10);
    delay(10);
  }
}

// 读取单个传感器的三轴数据（20 位模式）
void readSensorData(TwoWire &bus, uint8_t mux_addr, uint8_t channel,
                    uint32_t *x, uint32_t *y, uint32_t *z) {
  selectChannel(bus, mux_addr, channel);

  uint8_t buf[9];
  readMulti(bus, 0x00, buf, 9);

  // 寄存器布局: Xout0(0x00) Xout1(0x01) Yout0(0x02) Yout1(0x03)
  //             Zout0(0x04) Zout1(0x05) Xout2(0x06) Yout2(0x07) Zout2(0x08)
  // Xout2/Yout2/Zout2 的低 4 位存放 Xout[3:0]/Yout[3:0]/Zout[3:0]
  *x = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)(buf[6] & 0x0F));
  *y = ((uint32_t)buf[2] << 12) | ((uint32_t)buf[3] << 4) | ((uint32_t)(buf[7] & 0x0F));
  *z = ((uint32_t)buf[4] << 12) | ((uint32_t)buf[5] << 4) | ((uint32_t)(buf[8] & 0x0F));
}

// 读取全部 16 个传感器
void readAllSensors() {
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t mux_idx = i / 8;
    uint8_t ch = i % 8;
    TwoWire &bus = (mux_idx == 0) ? Wire : I2Cone;
    uint8_t addr = (mux_idx == 0) ? I2C_ADDR_1 : I2C_ADDR_2;
    readSensorData(bus, addr, ch,
                   &sensor_data[i].x, &sensor_data[i].y, &sensor_data[i].z);
  }
}

// ==================== Arduino 入口 ====================

void setup() {
  Serial.begin(921600);

  // 初始化双 I2C 总线，400kHz
  Wire.begin(SDA1_PIN, SCL1_PIN, 400000);    // I2C0
  I2Cone.begin(SDA2_PIN, SCL2_PIN, 400000);  // I2C1
  delay(10);

  // 初始化所有传感器为连续测量模式
  initAllSensors();

  t_next = micros();
  Serial.println("Dual-I2C MMC5603 Continuous Mode Initialized (120Hz)");
}

void loop() {
  // 定时轮询：每 8333µs（120Hz）读取一次全部传感器
  if ((int32_t)(micros() - t_next) >= 0) {
    t_next += SENSOR_PERIOD_US;

    readAllSensors();

    // 输出一帧 CSV 数据
    Serial.print("F,");
    for (uint8_t i = 0; i < 16; i++) {
      Serial.print(sensor_data[i].x);
      Serial.write(',');
      Serial.print(sensor_data[i].y);
      Serial.write(',');
      Serial.print(sensor_data[i].z);
      if (i < 15) Serial.write(',');
    }
    Serial.println();
  }
}
