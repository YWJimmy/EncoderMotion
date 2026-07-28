# EncoderMotion — 全新中断式编码器运动工程
> **v1.1 build correction:** `USER_IO` was split into three single-port SysConfig GPIO instances so generated `*_PORT` macros are valid with SysConfig 1.26.2 / MSPM0 SDK 2.11.00.07.


本工程面向 **LP-MSPM0G3507 + TB6612 + 双路AB相编码器**，使用 CCS、TI-Clang、MSPM0 SDK 和 SysConfig。

这不是旧工程的覆盖补丁。编码器、时间基准、运动控制、日志、OLED和按键模块均重新建立。

## 环境

- Code Composer Studio 21.x / Theia
- TI Arm Clang 5.1.1 LTS
- MSPM0 SDK 2.11.0.07
- SysConfig 1.26.2
- 目标：MSPM0G3507，LQFP-64
- 调试器：板载 XDS110

## 引脚

| 功能 | 引脚 |
|---|---|
| 左编码器A | PA28 |
| 左编码器B | PA31 |
| 右编码器A | PA12 |
| 右编码器B | PA13 |
| TB6612 AIN1/AIN2 | PB0/PB1 |
| TB6612 BIN1/BIN2 | PB2/PB3 |
| TB6612 STBY | PB4 |
| 左/右PWM | PB20/PB13，TIMA0 CC2/CC3 |
| OLED SCL/SDA | PB8/PB9 |
| 菜单选择/确认 | PA18/PB21 |
| XDS110串口TX | PA10，UART0，115200 8N1 |

## 编码器设计

- 四个输入全部启用内部上拉和GPIO迟滞。
- PA28、PA31、PA12、PA13均启用上升沿与下降沿中断。
- 使用16项AB状态转移表，每个合法四分之一跳变立即计数±1。
- 非法两位跳变：同步当前状态，但不计数。
- A和B在同一次ISR中同时挂起：无法恢复边沿先后顺序，只同步状态，不计数。
- ISR不循环排空，不在ISR中打印串口，不做浮点计算。
- 左右累计计数和周期增量通过一次临界区原子快照读取。

实测标定为 `1469 count/rev`，与四倍频计数保持一致。

## 运动控制

工程不使用未经实车标定的速度PI。控制结构为：

```text
剩余位置 -> 分段减速基础PWM -> 左右累计进度差P修正 -> TB6612 PWM
```

- 直线和原地转向使用独立参数。
- 同步控制只有P项，带死区和限幅。
- 两轮均进入位置容差后才制动。
- 一轮提前到达时，该轮PWM归零，另一轮继续低速追赶。
- 运动任务带超时和编码器停转保护。

## 导入和编译

不要把本包直接解压到旧项目目录。

1. 将旧 `EncoderMotion` 文件夹改名为备份。
2. 解压本包，保持顶层目录名为 `EncoderMotion_Rebuilt_v1`。
3. CCS：`File -> Import Projects from File System`。
4. 选择解压后的项目目录。
5. 打开 `empty.syscfg`，确认SDK产品后保存一次。
6. `Project -> Clean`。
7. `Build Project`。
8. 烧录后打开 XDS110 Application/User UART：115200、8N1。

若CCS提示SDK版本不匹配，在项目属性中选择本机已安装的兼容MSPM0 SDK，再保存 `empty.syscfg` 重新生成配置。

## 首次测试顺序

### 1. 仅上电静止60秒

确认：

- `lc`、`rc`不变化；
- `la/lb`、`ra/rb`不应高速增加；
- `lm/rm`接近0；
- `li/ri`接近0。

### 2. 手动转轮

每个轮子单独正向转50圈，计数应接近：

```text
1469 × 50 = 73450
```

方向若反了，只修改 `APP_LEFT_ENCODER_SIGN` 或 `APP_RIGHT_ENCODER_SIGN`，不要交换AB状态表。

### 3. 架空测试动作

依次测试：

- 前进500 mm；
- 后退200 mm；
- 左转90°；
- 右转90°。

### 4. 落地低速测试

保持PWM 250–300，确认计数和方向正确后再标定轮径、左右PWM trim和有效转向轮距。

## 关键配置

集中位于 `app_config.h`：

- `APP_ENCODER_COUNTS_PER_REV`
- `APP_WHEEL_DIAMETER_MM`
- `APP_WHEEL_TRACK_MM`
- `APP_LEFT_ENCODER_SIGN`
- `APP_RIGHT_ENCODER_SIGN`
- `APP_LEFT_PWM_TRIM_PERMILLE`
- `APP_RIGHT_PWM_TRIM_PERMILLE`
- 直线/转向减速与同步参数

## 目录说明

- `empty.c`：主程序与菜单状态机
- `encoder.c/.h`：GPIO双边沿中断和AB解码
- `motor.c/.h`：TB6612方向、PWM、制动
- `motion.c/.h`：定距与转向位置控制
- `app_time.c/.h`：1 ms单调时间基准
- `button.c/.h`：非阻塞按键消抖
- `oled.c/.h`：软件I2C OLED和分页面刷新
- `serial_log.c/.h`：非阻塞UART环形缓冲日志
- `empty.syscfg`：完整引脚和外设配置
- `targetConfigs/MSPM0G3507.ccxml`：XDS110目标配置
- `tests/`：主机侧静态和模型测试

## 重要硬件要求

编码器、MSPM0、TB6612和电池必须可靠共地。编码器模块电源处建议配置0.1 μF陶瓷电容；电机线与编码器线分开走线。软件无法修复断续地线或悬空输入。
