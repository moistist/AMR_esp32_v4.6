# AMR ESP32 v4.6 — 16 通道磁传感器阵列采集系统

## 项目简介

本项目基于 **ESP32-S3** 微控制器，通过 **2 片 PCA9548A I2C 多路复用器**（分别接入 ESP32-S3 的两个独立 I2C 控制器），驱动 **16 个 MMC5603NJ 三轴磁传感器**，实现稳定 **200Hz** 的磁场数据同步采集。

传感器采用 **MMC5603 连续测量模式（ODR=200Hz）**，由芯片内部时钟自主采样，主机以固定周期定时轮询读取数据。采用**交错配对读取**策略，将同一通道的两条总线传感器背靠背读取，最小化采样时间偏差。

### 应用场景

- 自主移动机器人（AMR）磁场导航
- 磁场梯度检测与定位
- 多点磁场同步监测
- 磁编码器 / 磁栅尺信号采集

---

## 硬件架构

```
┌──────────────────────────────────────┐
│              ESP32-S3                │
│  I2C0: SDA=GPIO3, SCL=GPIO8        │
│  I2C1: SDA=GPIO4, SCL=GPIO5        │
│  双总线并行操作，400kHz              │
└──────┬───────────────┬───────────────┘
       │ I2C0          │ I2C1
┌──────▼───────┐ ┌─────▼────────┐
│  PCA9548A #1 │ │  PCA9548A #2 │
│  地址: 0x70  │ │  地址: 0x73  │
│ (A0=A1=A2=0) │ │(A0=1,A1=A2=0)│
├──────────────┤ ├────────────────┤
│ CH0: MMC5603 │ │ CH0: MMC5603  │
│ CH1: MMC5603 │ │ CH1: MMC5603  │
│ CH2: MMC5603 │ │ CH2: MMC5603  │
│ CH3: MMC5603 │ │ CH3: MMC5603  │
│ CH4: MMC5603 │ │ CH4: MMC5603  │
│ CH5: MMC5603 │ │ CH5: MMC5603  │
│ CH6: MMC5603 │ │ CH6: MMC5603  │
│ CH7: MMC5603 │ │ CH7: MMC5603  │
└──────────────┘ └────────────────┘
  传感器 #0 ~ #7      传感器 #8 ~ #15
```

### 关键器件

| 器件 | 型号 | 作用 |
|------|------|------|
| 主控 | ESP32-S3 | 双 I2C 主机，数据采集与串口输出 |
| I2C 多路复用器 | PCA9548A × 2 | 8 通道 I2C 开关，分别接入两个 I2C 控制器 |
| 磁传感器 | MMC5603NJ × 16 | 三轴磁传感器，20 位分辨率，±30 Gauss 量程，连续 200Hz 采样 |

### 硬件连接

| ESP32-S3 引脚 | 连接目标 | 说明 |
|---------------|---------|------|
| GPIO3 (SDA) | PCA9548A #1 SDA | I2C0 数据线 |
| GPIO8 (SCL) | PCA9548A #1 SCL | I2C0 时钟线 |
| GPIO4 (SDA) | PCA9548A #2 SDA | I2C1 数据线 |
| GPIO5 (SCL) | PCA9548A #2 SCL | I2C1 时钟线 |
| 3.3V | 所有器件 VCC | 供电 |
| GND | 所有器件 GND | 共地 |

> **注意**：PCA9548A 的 A0/A1/A2 地址引脚必须通过外部电阻明确拉高或拉低（无内部上拉）。第一片 A0=A1=A2=GND（地址 0x70），第二片 A0=VCC, A1=A2=GND（地址 0x73）。

---

## 软件设计

### 开发环境

- **框架**：Arduino (PlatformIO)
- **配置文件**：`platformio.ini`
- **源码位置**：`src/main.ino`

### 工作模式：连续测量 + 定时轮询 + 交错配对读取

```
MMC5603 传感器（内部自主采样）:
  [采样] [采样] [采样] [采样] [采样] ...  ← 200Hz 稳定采样

ESP32-S3 主机（定时轮询读取，交错配对）:
  [(#0,#8) → (#1,#9) → ... → (#7,#15)] [等待] [下一帧] ...
   |←———— 约 1.9ms ————→|← 3.1ms →|
```

与传统的"触发→等待→读取"状态机不同，本版本利用 MMC5603 的**连续测量模式**：

1. 初始化时配置 ODR=200Hz，传感器自主持续采样
2. 主机以 5ms（200Hz）的固定周期轮询读取全部 16 个传感器
3. **无需触发测量、无需等待 ADC 完成**，数据始终就绪

### 交错配对读取策略

为最小化传感器之间的采样时间偏差，采用**交错配对读取**：

```
读取顺序:  (#0, #8) → (#1, #9) → (#2, #10) → ... → (#7, #15)
            ↑ 背靠背读取同一通道的两条总线传感器
            配对间时间偏差 ~120µs（vs 顺序读取的 ~950µs）
```

同一通道的两个传感器（如 #0 和 #8）分别位于两条独立 I2C 总线上，背靠背连续读取，将时间偏差从 ~950µs 降至 ~120µs，有效减少跨帧不一致的风险。

### MMC5603 寄存器配置

| 寄存器 | 地址 | 写入值 | 含义 |
|--------|------|--------|------|
| 内部控制寄存器 1 (0x1C) | 0x1C | `0x03` | BW[1:0]=11，最大带宽，单通道测量时间 ~1.2ms |
| ODR 寄存器 (0x1A) | 0x1A | `200` (0xC8) | 设置输出数据速率为 200Hz |
| 内部控制寄存器 0 (0x1B) | 0x1B | `0x20` → `0xA0` | 先设 Auto_SR_en=1，再设 Cmm_freq_en=1 + Auto_SR_en=1 |
| 内部控制寄存器 2 (0x1D) | 0x1D | `0x10` | Cmm_en=1，进入连续测量模式 |

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
- 帧率：稳定 **200 Hz**

---

## 实操指南

### 1. 环境搭建

#### 安装 PlatformIO

```bash
pip install platformio
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
2. **连接 I2C0 总线**：ESP32-S3 的 GPIO3(SDA) 和 GPIO8(SCL) 连接到 PCA9548A #1
3. **连接 I2C1 总线**：ESP32-S3 的 GPIO4(SDA) 和 GPIO5(SCL) 连接到 PCA9548A #2
4. **连接 MMC5603NJ 传感器** × 16，每个传感器的 SDA/SCL 分别接到对应 PCA9548A 的 CH0~CH7
5. **供电**：确保 3.3V 电源能提供足够电流（16 个传感器，每个约 1.3mA，总计约 20mA）
6. **共地**：所有器件 GND 连接在一起

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

## 版本演进

### v4.6 — 双 I2C 总线 + 连续测量 200Hz + 交错配对读取（当前版本）

合并了 PollingMux 和 ContinuousMux_120Hz 两个分支的优点，并进一步优化：

| 改进项 | 说明 |
|--------|------|
| 双 I2C 总线 | ESP32-S3 的 I2C0 和 I2C1 分别驱动两片 PCA9548A，读取速度翻倍 |
| 连续测量模式 | MMC5603 以 200Hz 自主采样，主机定时轮询，帧率确定 |
| 交错配对读取 | 读取顺序 (#0,#8)→(#1,#9)→...，配对偏差从 ~950µs 降至 ~120µs |
| 去掉状态机 | 不再需要 START→WAIT→READ 三阶段，代码更简洁 |
| 7 位 MUX 地址 | 使用标准 7 位地址格式（0x70/0x73），提高可移植性 |

### 历史修复与优化

| Commit | 修复内容 |
|--------|---------|
| 82c2f533 | 修复数据拼接位操作（`>>4` → `& 0x0F`）和 ADC 等待时间（2000µs → 6000µs） |
| 78eedb24 | 升级为双总线 + 连续测量 120Hz 模式 |
| 36fab198 | 采样频率从 120Hz 提升到 200Hz |
| 7c224161 | 交错配对读取，最小化传感器间时间偏差 |

---

## 代码审查报告

> 以下审查基于 MMC5603NJ Datasheet Rev.B 和 PCA9548A Datasheet Rev.5.1

### ✅ 验证通过项

| 检查项 | 代码值 | 数据手册值 | 状态 |
|--------|--------|-----------|------|
| MMC5603 I2C 7 位地址 | `0x30` | `0x30` | ✅ |
| PCA9548A 7 位地址 | `0x70` / `0x73` | A0/A1/A2 编码 | ✅ |
| Auto_SR_en 位定义 | bit5 = `0x20` | 0x1B 寄存器 bit5 | ✅ |
| Cmm_freq_en 位定义 | bit7 = `0x80` | 0x1B 寄存器 bit7 | ✅ |
| Cmm_en 位定义 | bit4 = `0x10` | 0x1D 寄存器 bit4 | ✅ |
| BW 带宽设置 | `0x03` (BW=11) | 0x1C 寄存器 bit1:0 | ✅ |
| ODR 设置 | `200` | 0x1A 寄存器，范围 1~255 | ✅ |
| 数据拼接（20位） | `& 0x0F` 取低4位 | Xout2/Yout2/Zout2 低4位有效 | ✅ |
| I2C 速率 | 400kHz | 两款芯片均支持 Fast-mode 400kHz | ✅ |
| MUX 通道选择 | `1 << channel` | PCA9548A 按位选择 | ✅ |
| 连续测量初始化序列 | ODR→Cmm_freq_en→Cmm_en | 数据手册推荐流程 | ✅ |
| 交错配对读取 | 同通道背靠背 | 最小化时间偏差 ~120µs | ✅ |

---

## 参考文档

| 文档 | 说明 |
|------|------|
| [MMC5603NJDatasheetRev.B.pdf](./MMC5603NJDatasheetRev.B.pdf) | MMC5603NJ 三轴磁传感器数据手册 Rev.B |
| [PCA9548A.pdf](./PCA9548A.pdf) | PCA9548A 8 通道 I2C 多路复用器数据手册 Rev.5.1 |

## 相关仓库

| 仓库 | 说明 |
|------|------|
| [Aarrs1/MMC5603_PollingMux](https://github.com/Aarrs1/MMC5603_PollingMux) | 轮询单点测试版（双总线 + 单次触发模式） |
| [Aarrs1/MMC5603_ContinuousMux_120Hz](https://github.com/Aarrs1/MMC5603_ContinuousMux_120Hz) | 连续测量稳定 120Hz 版（双总线 + 连续测量模式） |

---

## 许可证

本项目仅供学习和研究使用。
