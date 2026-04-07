#!/usr/bin/env python3
"""
磁场相机 — 4×4 传感器阵列实时可视化工具

功能：
  - 从 ESP32-S3 串口读取 16 个 MMC5603NJ 传感器的 XYZ 三轴磁场数据
  - 以 4×4 彩色热力图实时显示磁场强度
  - 支持三种视图：总磁场强度 | X 轴分量 | Y 轴分量 | Z 轴分量
  - 支持实时模式和 CSV 文件回放模式

用法：
  实时模式：  python mag_camera.py --port /dev/ttyUSB0
  回放模式：  python mag_camera.py --file mag_data.csv

依赖：
  pip install numpy matplotlib pyserial
"""

import sys
import time
import argparse
import numpy as np

try:
    import serial
except ImportError:
    serial = None

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.gridspec import GridSpec

# ==================== 常量 ====================

BAUD_RATE = 921600
GRID_SIZE = 4  # 4×4 传感器阵列
NUM_SENSORS = GRID_SIZE * GRID_SIZE  # 16

# MMC5603NJ 20 位数据范围
# ±30 Gauss, 16384 counts/G, 零场 ≈ 524288 (2^19)
RAW_MAX = 1048576  # 2^20
RAW_CENTER = 524288  # 2^19 (零场)
COUNTS_PER_GAUSS = 16384.0

# ==================== 数据解析 ====================

def parse_frame(line: str) -> np.ndarray | None:
    """解析一行 CSV 帧数据，返回 (16, 3) 数组 [x, y, z]"""
    line = line.strip()
    if not line.startswith('F,'):
        return None
    values = line[2:].split(',')
    if len(values) != NUM_SENSORS * 3:
        return None
    try:
        data = np.array([int(v) for v in values], dtype=np.float64)
        return data.reshape(NUM_SENSORS, 3)
    except ValueError:
        return None

def raw_to_gauss(raw: np.ndarray) -> np.ndarray:
    """将 20 位原始值转换为 Gauss 值"""
    return (raw - RAW_CENTER) / COUNTS_PER_GAUSS

def compute_magnitude(data_gauss: np.ndarray) -> np.ndarray:
    """计算总磁场强度 |B| = sqrt(Bx² + By² + Bz²)"""
    return np.sqrt(np.sum(data_gauss ** 2, axis=1))

# ==================== 传感器布局映射 ====================

# 传感器编号 → 4×4 网格位置
# 传感器 #0~#7 在 PCA9548A #1，#8~#15 在 PCA9548A #2
# 假设物理布局如下（可根据实际硬件调整）：
#
#   PCA9548A #1 (I2C0)        PCA9548A #2 (I2C1)
#   CH0 CH1 CH2 CH3           CH0 CH1 CH2 CH3
#   #0  #1  #2  #3            #8  #9  #10 #11
#   CH4 CH5 CH6 CH7           CH4 CH5 CH6 CH7
#   #4  #5  #6  #7            #12 #13 #14 #15
#
# 4×4 网格排列：
#   [ 0  1  2  3 ]
#   [ 4  5  6  7 ]
#   [ 8  9  10 11]
#   [12 13 14 15]

SENSOR_TO_GRID = {
    0: (0, 0),  1: (0, 1),  2: (0, 2),  3: (0, 3),
    4: (1, 0),  5: (1, 1),  6: (1, 2),  7: (1, 3),
    8: (2, 0),  9: (2, 1), 10: (2, 2), 11: (2, 3),
   12: (3, 0), 13: (3, 1), 14: (3, 2), 15: (3, 3),
}

def to_grid(data: np.ndarray) -> np.ndarray:
    """将 (16,) 数据映射到 (4, 4) 网格"""
    grid = np.zeros((GRID_SIZE, GRID_SIZE))
    for i in range(NUM_SENSORS):
        r, c = SENSOR_TO_GRID[i]
        grid[r, c] = data[i]
    return grid

# ==================== 可视化 ====================

class MagCamera:
    """磁场相机可视化类"""

    VIEWS = ['|B| 总强度', 'Bx 分量', 'By 分量', 'Bz 分量']
    CMAPS = ['inferno', 'RdBu_r', 'PiYG', 'coolwarm']

    def __init__(self, auto_range: bool = True):
        self.auto_range = auto_range
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0

        # 创建图形
        self.fig = plt.figure(figsize=(14, 8), facecolor='#1a1a2e')
        self.fig.canvas.manager.set_window_title('磁场相机 — Magnetic Field Camera')

        gs = GridSpec(2, 3, figure=self.fig, width_ratios=[1, 1, 0.05],
                      hspace=0.35, wspace=0.3,
                      left=0.06, right=0.92, top=0.90, bottom=0.08)

        # 4 个子图：|B|, Bx, By, Bz
        self.axes = []
        self.images = []
        self.texts = []

        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for idx, (row, col) in enumerate(positions):
            ax = self.fig.add_subplot(gs[row, col])
            ax.set_facecolor('#16213e')

            # 初始化空图像
            dummy = np.zeros((GRID_SIZE, GRID_SIZE))
            im = ax.imshow(dummy, cmap=self.CMAPS[idx], interpolation='bilinear',
                           aspect='equal', vmin=-1, vmax=1)

            # 网格线和标签
            ax.set_xticks(range(GRID_SIZE))
            ax.set_yticks(range(GRID_SIZE))
            ax.set_xticklabels([f'CH{c}' for c in range(GRID_SIZE)], color='white', fontsize=8)
            ax.set_yticklabels([f'CH{r}' for r in range(GRID_SIZE)], color='white', fontsize=8)
            ax.tick_params(colors='white', length=0)
            ax.set_title(self.VIEWS[idx], color='white', fontsize=12, fontweight='bold', pad=8)

            # 在每个格子上显示数值
            texts = []
            for r in range(GRID_SIZE):
                row_texts = []
                for c in range(GRID_SIZE):
                    t = ax.text(c, r, '', ha='center', va='center',
                               fontsize=8, color='white', fontweight='bold',
                               bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.5))
                    row_texts.append(t)
                texts.append(row_texts)
            self.texts.append(texts)
            self.axes.append(ax)
            self.images.append(im)

        # 色标
        self.cbar_ax = self.fig.add_subplot(gs[:, 2])
        self.cbar = self.fig.colorbar(self.images[0], cax=self.cbar_ax)
        self.cbar.ax.yaxis.set_tick_params(color='white')
        self.cbar.ax.yaxis.label.set_color('white')
        plt.setp(self.cbar.ax.yaxis.get_ticklabels(), color='white')
        self.cbar.set_label('Gauss', color='white', fontsize=10)

        # 标题
        self.title = self.fig.suptitle('磁场相机 | Frame: 0 | FPS: --',
                                        color='#e94560', fontsize=14, fontweight='bold')

        plt.ion()
        plt.show()

    def update(self, data_raw: np.ndarray):
        """更新一帧数据"""
        self.frame_count += 1
        self.fps_frame_count += 1

        # 计算 FPS
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.fps_frame_count / (now - self.last_fps_time)
            self.fps_frame_count = 0
            self.last_fps_time = now

        # 转换为 Gauss
        data_gauss = raw_to_gauss(data_raw)

        # 计算各分量
        magnitude = compute_magnitude(data_gauss)
        bx = data_gauss[:, 0]
        by = data_gauss[:, 1]
        bz = data_gauss[:, 2]

        components = [magnitude, bx, by, bz]

        for idx, comp in enumerate(components):
            grid = to_grid(comp)

            # 更新图像
            self.images[idx].set_data(grid)

            # 自动范围或固定范围
            if self.auto_range:
                if idx == 0:  # |B| 总是非负
                    vmin, vmax = 0, max(np.max(grid), 0.1)
                else:
                    vmax = max(np.max(np.abs(grid)), 0.1)
                    vmin = -vmax
                self.images[idx].set_clim(vmin, vmax)

            # 更新数值文本
            for r in range(GRID_SIZE):
                for c in range(GRID_SIZE):
                    val = grid[r, c]
                    self.texts[idx][r][c].set_text(f'{val:.1f}')

        # 更新色标范围（跟随当前选中的视图）
        # 这里用 |B| 的范围
        mag_grid = to_grid(magnitude)
        self.cbar.mappable.set_clim(0, max(np.max(mag_grid), 0.1))
        self.cbar.set_label('Gauss', color='white', fontsize=10)

        # 更新标题
        self.title.set_text(
            f'磁场相机 | Frame: {self.frame_count} | '
            f'FPS: {self.fps:.1f} | '
            f'Max |B|: {np.max(magnitude):.2f} G | '
            f'Min |B|: {np.min(magnitude):.2f} G'
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        plt.close(self.fig)


# ==================== 实时模式 ====================

def run_realtime(port: str, num_frames: int = 0):
    """从串口实时读取并可视化"""
    if serial is None:
        print("错误：请安装 pyserial: pip install pyserial")
        sys.exit(1)

    print(f"正在连接 {port} ({BAUD_RATE} baud)...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    camera = MagCamera(auto_range=True)

    print("开始实时采集，按 Ctrl+C 停止")
    try:
        count = 0
        while num_frames == 0 or count < num_frames:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            data = parse_frame(line)
            if data is not None:
                camera.update(data)
                count += 1
    except KeyboardInterrupt:
        print(f"\n已停止，共采集 {count} 帧")
    finally:
        ser.close()
        camera.close()


# ==================== CSV 回放模式 ====================

def run_csv_playback(filepath: str, interval_ms: int = 5):
    """从 CSV 文件回放数据"""
    print(f"正在加载 {filepath}...")
    frames = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('F,'):
                data = parse_frame(line)
                if data is not None:
                    frames.append(data)

    if not frames:
        print("错误：文件中没有有效帧数据")
        sys.exit(1)

    print(f"加载了 {len(frames)} 帧，回放间隔 {interval_ms}ms")
    camera = MagCamera(auto_range=True)

    try:
        for i, frame in enumerate(frames):
            camera.update(frame)
            time.sleep(interval_ms / 1000.0)
    except KeyboardInterrupt:
        print(f"\n回放停止于第 {i} 帧")
    finally:
        camera.close()


# ==================== 入口 ====================

def main():
    parser = argparse.ArgumentParser(description='磁场相机 — 4×4 MMC5603NJ 传感器阵列可视化')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--port', type=str, help='串口设备路径（实时模式），如 /dev/ttyUSB0')
    group.add_argument('--file', type=str, help='CSV 文件路径（回放模式）')
    parser.add_argument('--frames', type=int, default=0, help='采集帧数（0=无限，仅实时模式）')
    parser.add_argument('--interval', type=int, default=5, help='回放间隔毫秒（仅回放模式）')

    args = parser.parse_args()

    if args.port:
        run_realtime(args.port, args.frames)
    elif args.file:
        run_csv_playback(args.file, args.interval)


if __name__ == '__main__':
    main()
