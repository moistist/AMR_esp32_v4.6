# AMR ESP32 v4.6 — 16 通道磁传感器阵列采集系统

## 项目简介

本项目基于 **ESP32-S3** 微控制器，通过 **2 片 PCA9548A I2C 多路复用器** 级联，驱动 **16 个 MMC5603NJ 三轴磁传感器**，实现高频率、非阻塞的磁场数据同步采集。系统采用状态机架构，以 CSV 帧格式通过串口输出 16 个传感器的 X/Y/Z 三轴原始数据。

### 应用场景

- 自主移动机器人（AMR）磁场导航
- 磁场梯度检测与定位
- 多点磁场同步监测
- 磁编码器 / 磁栅尺信号采集

---

## 硬件架构

```
┌──────────────┐
│   ESP32-S3   │
│  SDA: GPIO3  │
│  SCL: GPIO8  │
│  I2C: 400kHz │
└──────┬───────┘
       │ I2C 总线
       ├──────────────────────┐
       │                      │
┌──────▼───────┐    ┌────────▼───────┐
│  PCA9548A #1 │    │  PCA9548A #2  │
│  地址: 0x70  │    │  地址: 0x73   │
│ (A0=A1=A2=0) │    │(A0=1,A1=A2=0)│
├──────────────┤    ├────────────────┤
│ CH0: MMC5603 │    │ CH0: MMC5603  │
│ CH1: MMC5603 │    │ CH1: MMC5603  │
│ CH2: MMC5603 │    │ CH2: MMC5603  │
│ CH3: MMC5603 │    │ CH3: MMC5603  │
│ CH4: MMC5603 │    │ CH4: MMC5603  │
│ CH5: MMC5603 │    │ CH5: MMC5603  │
│ CH6: MMC5603 │    │ CH6: MMC5603  │
│ CH7: MMC5603 │    │ CH7: MMC5603  │
└──────────────┘    └────────────────┘
  传感器 #0 ~ #7      传感器 #8 ~ #15
```

### 关键器件

| 器件 | 型号 | 作用 |
|------|------|------|
| 主控 | ESP32-S3 | I2C 主机，数据采集与串口输出 |
| I2C 多路复用器 | PCA9548A × 2 | 8 通道 I2C 开关，2 片级联扩展至 16 通道 |
| 磁传感器 | MMC5603NJ × 16 | 三轴磁传感器，20 位分辨率，±30 Gauss 量程 |

### 硬件连接

| ESP32-S3 引脚 | 连接目标 | 说明 |
|---------------|---------|------|
| GPIO3 (SDA) | PCA9548A #1 & #2 的 SDA | I2C 数据线 |
| GPIO8 (SCL) | PCA9548A #1 & #2 的 SCL | I2C 时钟线 |
| 3.3V | 所有器件 VCC | 供电 |
| GND | 所有器件 GND | 共地 |

> **注意**：PCA9548A 的 A0/A1/A2 地址引脚必须通过外部电阻明确拉高或拉低（无内部上拉）。第一片 A0=A1=A2=GND（地址 0x70），第二片 A0=VCC, A1=A2=GND（地址 0x73）。

---

## 软件设计

### 开发环境

- **框架**：Arduino (PlatformIO)
- **配置文件**：`platformio.ini`
- **源码位置**：`src/main.ino`

### 状态机流程

程序采用**非阻塞三阶段状态机**，避免 `delay()` 阻塞，最大化采集帧率：

```
┌─────────────────┐
│ START_MEASURE   │  向 16 个传感器同时发出测量指令
│ (触发全部测量)   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│     WAIT        │  非阻塞等待 ADC 转换完成
│ (等待 ~6ms)     │  micros() 轮询，不阻塞 CPU
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│     READ        │  依次切换 MUX 通道，读取 16 个传感器数据
│ (读取全部数据)   │  输出一帧 CSV 数据到串口
└────────┬────────┘
         │
         └──────► 回到 START_MEASURE（循环）
```

### MMC5603NJ 寄存器配置

| 寄存器 | 地址 | 写入值 | 含义 |
|--------|------|--------|------|
| 内部控制寄存器 1 (0x1C) | 0x1C | `0x03` | BW[1:0]=11，最大带宽，单通道测量时间 ~1.2ms |
| 内部控制寄存器 0 (0x1B) | 0x1B | `0x20` (初始化) / `0x21` (测量) | Auto_SR_en=1（自动 SET/RESET 去磁），TM_M=1（触发测量） |

### 数据拼接说明（20 位模式）

MMC5603NJ 连续读取 9 字节（0x00~0x08），寄存器布局如下：

```
字节:    buf[0]    buf[1]    buf[2]    buf[3]    buf[4]    buf[5]    buf[6]    buf[7]    buf[8]
寄存器:  Xout0     Xout1     Yout0     Yout1     Zout0     Zout1     Xout2     Yout2     Zout2
内容:    X[19:12]  X[11:4]   Y[19:12]  Y[11:4]   Z[19:12]  Z[11:4]   X[3:0]    Y[3:0]    Z[3:0]
```

三轴 20 位数据拼接方式：

```cpp
x = (buf[0] << 12) | (buf[1] << 4) | (buf[6] & 0x0F);  // Xout2 低4位
y = (buf[2] << 12) | (buf[3] << 4) | (buf[7] & 0x0F);  // Yout2 低4位
z = (buf[4] << 12) | (buf[5] << 4) | (buf[8] & 0x0F);  // Zout2 低4位
```

### 数据输出格式

串口以 **921600 波特率** 输出 CSV 格式数据，每行一帧：

```
F,X0,Y0,Z0,X1,Y1,Z1,...,X15,Y15,Z15
```

- `F` — 帧头标识
- `Xn, Yn, Zn` — 第 n 个传感器的三轴原始数据（20 位无符号整数）
- 每帧包含 16 个传感器 × 3 轴 = 48 个数据值

---

## 实操指南

### 1. 环境搭建

#### 安装 PlatformIO

```bash
# 安装 PlatformIO CLI
pip install platformio

# 或在 VS Code 中安装 PlatformIO 插件
```

#### 克隆项目

```bash
git clone https://github.com/moistist/AMR_esp32_v4.6.git
cd AMR_esp32_v4.6
```

### 2. 编译与上传

```bash
# 编译项目
pio run

# 上传到 ESP32-S3
pio run --target upload

# 查看串口输出
pio device monitor -b 921600
```

### 3. 硬件组装步骤

1. **焊接 PCA9548A 模块** × 2，设置地址引脚：
   - 第一片：A0=A1=A2=GND → 地址 0x70
   - 第二片：A0=VCC, A1=A2=GND → 地址 0x73
2. **连接 I2C 总线**：ESP32-S3 的 GPIO3(SDA) 和 GPIO8(SCL) 并联连接到两片 PCA9548A 的 SDA/SCL
3. **连接 MMC5603NJ 传感器** × 16，每个传感器的 SDA/SCL 分别接到 PCA9548A 的 CH0~CH7
4. **供电**：确保 3.3V 电源能提供足够电流（16 个传感器，每个约 1.3mA，总计约 20mA）
5. **共地**：所有器件 GND 连接在一起

### 4. 数据接收与解析

#### Python 示例

```python
import serial
import csv
from datetime import datetime

ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1)

with open(f'mag_data_{datetime.now():%Y%m%d_%H%M%S}.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['frame'] + [f's{i}_{axis}' for i in range(16) for axis in 'XYZ'])

    frame_count = 0
    while frame_count < 1000:  # 采集 1000 帧
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('F,'):
            values = line[2:].split(',')
            writer.writerow([frame_count] + values)
            frame_count += 1
            print(f'\rFrame {frame_count}', end='')

ser.close()
print('\nDone!')
```

---

## 代码审查报告

> 以下审查基于 MMC5603NJ Datasheet Rev.B 和 PCA9548A Datasheet Rev.5.1

### ✅ 验证通过项

| 检查项 | 代码值 | 数据手册值 | 状态 |
|--------|--------|-----------|------|
| MMC5603 I2C 7 位地址 | `0x30` | `0x30` | ✅ 正确 |
| Auto_SR_en 位定义 | bit5 = `0x20` | 0x1B 寄存器 bit5 | ✅ 正确 |
| BW 带宽设置 | `0x03` (BW=11) | 0x1C 寄存器 bit1:0，1.2ms | ✅ 正确 |
| 单次测量触发 | `0x21` | TM_M(bit0)=1 + Auto_SR_en(bit5)=1 | ✅ 正确 |
| 数据寄存器起始地址 | `0x00`，读 9 字节 | 0x00~0x08，共 9 字节 | ✅ 正确 |
| 数据拼接（20位） | `& 0x0F` 取低4位 | Xout2/Yout2/Zout2 低4位有效 | ✅ 已修复 |
| I2C 速率 | 400kHz | 两款芯片均支持 Fast-mode 400kHz | ✅ 正确 |
| MUX 通道选择 | `1 << channel` | PCA9548A 按位选择 | ✅ 正确 |
| 非阻塞状态机 | START→WAIT→READ | 合理的采集流程 | ✅ 正确 |
| ADC 等待时间 | 6000µs | BW=11 三轴+Auto SR 约 5ms+ | ✅ 已修复 |

### 🔧 修复记录

#### 1. 数据拼接位操作 ✅ 已修复 (Commit 82c2f533)

**原问题**：`readSensorData()` 中使用 `buf[6] >> 4` 提取 X 轴低 4 位，但 MMC5603NJ 的 Xout2 寄存器（0x06）存放的是 Xout[3:0]，有效数据在**低 4 位**，应使用 `& 0x0F`。Y 轴（buf[7]）和 Z 轴（buf[8]）同理。

```cpp
// 修复前（错误：取高4位，结果始终为0，实际只有16位有效数据）
* x = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)(buf[6] >> 4));

// 修复后（正确：取低4位，完整20位数据）
* x = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)(buf[6] & 0x0F));
```

#### 2. ADC 等待时间 ✅ 已修复 (Commit 82c2f533)

**原问题**：等待时间 2000µs 不足以覆盖 BW=11 时三轴顺序测量（~3.6ms）加上 Auto SR 的 SET 操作时间（375ns + 1ms），总时间约 5ms+。

```cpp
// 修复前
if (micros() - t_start > 2000)

// 修复后（6000µs，留足余量）
if (micros() - t_start > 6000)
```

### 💡 改进建议（未修改）

#### PCA9548A 地址格式

当前使用 8 位写地址（`0xE0`, `0xE6`），在 ESP32 Arduino Wire 库中可以正常工作，但建议改为 7 位地址以提高代码可移植性：

```cpp
// 当前（8 位写地址，ESP32 可用但不可移植）
#define I2C_ADDR_1 0xE0
#define I2C_ADDR_2 0xE6

// 建议（7 位地址，标准做法）
#define I2C_ADDR_1 0x70
#define I2C_ADDR_2 0x73
```

---

## 参考文档

| 文档 | 说明 |
|------|------|
| [MMC5603NJDatasheetRev.B.pdf](./MMC5603NJDatasheetRev.B.pdf) | MMC5603NJ 三轴磁传感器数据手册 Rev.B |
| [PCA9548A.pdf](./PCA9548A.pdf) | PCA9548A 8 通道 I2C 多路复用器数据手册 Rev.5.1 |

---

## 许可证

本项目仅供学习和研究使用。
