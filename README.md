# CarHUD — Cardputer Adv 车载 HUD 固件

> 娱乐性车载 HUD / 行车记录 / 简易导航，运行在 **M5Stack Cardputer Adv (K132-Adv)** 上，
> GNSS 数据来自 **Cap LoRa-1262** 背盖的卫星模块。

不接 OBD、不做校准——纯娱乐向：大字车速、罗盘、频谱律动、倾斜仪，
外加一套离线的 GPS 轨迹记录与路线导航。

![pages](docs/screenshot-placeholder.png)

## 功能一览

**页面 1 — HUD**
- 车速大字居中（Orbitron 字体，>80 km/h 变红，<2 km/h 归零滤波）
- 左上角罗盘（GNSS 航向驱动，红色 N 标记）
- 左下角 IMU 倾斜仪（侧视小车图标，随车身俯仰/侧倾实时旋转，度数超限变色，`R` 键一键归零）
- 右侧 DIR / ALT / TRIP 信息列 + 右上角电池电量（60s 缓存刷新防闪烁）
- 底部**频谱均衡器**：板载麦克风 16kHz 采样 → 256 点 FFT → 24 个对数频段，
  三种显示风格按 `N` 循环切换（柱状 / 山脊线 / 蝴蝶对称），
  低频保底噪声门 + 高频增益倾斜，人声和音乐高频也能轻松律动

**页面 2 — 地图 / 导航**
- 104×104 地图区：暗白经纬网格 + 比例尺 + 绿色路线 + 航点 + 旋转的橙色当前位置箭头
- **双视野模式**：浏览路线时自动缩放适配路线全貌；导航中固定比例（1px≈2.5m）跟车，转弯细节清晰
- 路线预览选择：地图上同时显示所有历史路线（绿色），选中路线橙色高亮
- **反向导航自动判定**：车速 >10km/h 时对比航向与路线延伸方向，反向自动亮橙色 `REV`（带迟滞防抖）
- 导航信息：进度条、偏航箭头、剩余里程、到达提示
- 轨迹记录（SD 卡存储，文件名按时间戳自动生成）、航点标记、航点管理

**页面 3 — 统计**
- 最高 / 平均车速、行驶时长、里程、卫星数 / 海拔、记录点数与 SD 状态
- 频谱条同样显示

## 按键说明

| 按键 | 功能 |
|---|---|
| `1` `2` `3` | 跳转到 HUD / 地图 / 统计页 |
| `←` `→` | 循环切换页面 |
| `R` | 倾斜仪归零（车辆停于水平地面时按） |
| `S` | GNSS 模块 开 / 关（省电） |
| `N` | 频谱风格循环（三页同步生效） |
| `H` | 全屏帮助覆盖层（任意键关闭；打开期间记录/GPS 不中断） |
| `G` | 开始 / 停止轨迹记录（记录中切页不中断） |
| `F` | 加载选中的路线进入导航 / 再次按退出导航 |
| `↑` `↓` | 浏览模式：选择历史路线（橙色高亮 + 视野跟随） |
| `M` | 在当前位置标记航点（输入名称，ENTER 确认） |
| `X` | 航点列表（↑↓ 选择，DEL 删除同名） |
| `DEL` | 浏览模式删除选中路线（3 秒内再按一次确认；记录中禁止） |
| `ENTER` | 确认 / 加载选中项 |

## 硬件

| 部件 | 说明 |
|---|---|
| 主机 | M5Stack Cardputer Adv (K132-Adv)：ESP32-S3 / ST7789V2 240×135 / BMI270 / ES8311 麦克风 / 键盘 |
| GNSS | Cap LoRa-1262 背盖（EXT UART：RX=G15 / TX=G13，波特率 9600/115200 自动探测） |
| 存储 | microSD 卡（SPI：SCK=40 / MOSI=14 / MISO=39 / CS=12），存轨迹 `/tracks/` 与航点 `/waypoints.csv` |

## 引脚映射

- EXT 总线 UART：RX=**G15**，TX=**G13** ← GNSS NMEA
- BMI270：I2C G8(SDA) / G9(SCL)
- ES8311 音频：I2C G8/G9，I2S BCLK=G41 / 数据=G46 / LRCK=G43
- LCD：BL=G38，RST=G33，DC=G34，MOSI=G35，SCK=G36，CS=G37
- SD 卡：SPI SCK=40 / MOSI=14 / MISO=39 / CS=12

## 编译 / 烧录

依赖 PlatformIO（VS Code 插件或 CLI）。`platformio.ini` 已固定依赖版本：
arduino-esp32 2.0.16（espressif32@6.7.0）、M5Unified、M5Cardputer、arduinoFFT v1.6.2。

```bash
pio run              # 编译
pio run -t upload    # 烧录（设备用 USB 连电脑）
pio device monitor   # 串口监视器 115200
```

如设备无法自动进入下载模式：电源开关拨到 OFF → 按住 G0 → 插 USB 通电 → 松开 G0。

也可以用 esptool 烧录合并固件（含 bootloader / 分区表，可直接网页烧录器使用）：

```bash
esptool.py --chip esp32s3 --port COMx --baud 460800 write_flash 0x0 carhud_merged.bin
# 合并命令参考：
# esptool.py --chip esp32s3 merge_bin -o carhud_merged.bin --flash_mode keep \
#   --flash_freq keep --flash_size 8MB 0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

## 使用提示

- GNSS 冷启动首次定位需要 1~3 分钟（开阔处），统计页可看卫星数辅助排查
- 金属贴膜车内 GNSS 信号会明显变差
- 记录路线：从 A 地 `G` 开始 → 行驶 → 到 B 地 `G` 停止；下次 `F` 选这条线即可导航
- 频谱只是娱乐性显示，不做声学校准
- 卡顿排查：HUD 页左下角偶发的橙色标签（G/R/F/H + 毫秒数）是慢段诊断，
  G=GNSS读取 / R=麦克风 / F=FFT / H=整循环

## Roadmap

- [ ] O 键 433MHz 射频开门（RMT 驱动 OOK 发射，重放 SD 卡上的固定码信号）
- [ ] 航点锚定的多路线拼接（路网导航）

## License

MIT — 见 [LICENSE](LICENSE)。
