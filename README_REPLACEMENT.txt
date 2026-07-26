EncoderMotion 可替换文件包 v5 - 控制节拍与误停修复

适用版本
========
本包用于覆盖 v4_SpeedPI。若工程是 v3_UART，也应先保证其中已有编码器、OLED、
串口和 app_tick 模块。本包中的 app_tick.c/app_tick.h 必须覆盖旧版本。

问题根因
========
v4 将每次 motion_update() 调用都视为真实经过 5 ms。OLED 软件 I2C 刷新若超过
一个节拍，旧 app_tick 会累计多个待处理节拍。主循环随后快速补跑这些节拍，但
补跑期间没有真实时间经过，编码器增量通常为 0，于是：

1. 速度滤波器错误地认为车轮突然停止；
2. 速度 PI 迅速提高或降低 PWM；
3. 编码器 200 ms 故障窗口被虚假推进；
4. 偶发进入 ENCFAULT，只行驶很小一段，直行和转弯都会发生。

本次修复
========
1. app_tick 不再排队补跑多个控制节拍，只保留一个待处理标志。
2. motion_update() 使用 app_tick_now() 计算真实经过的 SysTick 数量。
3. 同一真实时间点的重复调用不会推进 PI、制动、超时或故障检测。
4. 编码器增量按真实 elapsed ticks 归一化，避免阻塞后速度估计虚高或归零。
5. PWM斜坡、速度PI积分、同步积分、制动时间和任务超时都按真实时间推进。
6. 编码器故障检测增加 500 ms 启动宽限期。
7. 故障检测窗口由 200 ms 延长到 500 ms，降低静摩擦和短转弯误报。
8. 串口 MOT 行新增 dt 和 fg：
   dt = 最近一次控制更新跨越的真实 5 ms 节拍数；正常为 1。
   fg = 编码器故障检测剩余启动宽限节拍。

需要覆盖的文件
==============
app_config.h
app_tick.c
app_tick.h
empty.c
motion.c
motion.h
serial_log.c
serial_log.h
.gitignore

CCS 操作
========
1. 备份当前工程。
2. 将本包所有文件复制到 EncoderMotion 工程根目录并覆盖。
3. 在 CCS 中 Refresh。
4. 确认 app_tick.c、motion.c、serial_log.c 未被 Exclude from Build。
5. Project -> Clean。
6. Build Project。
7. 烧录后打开 XDS110 UART，115200 8N1。

建议测试
========
先测试：
  motion_start_distance_mm(500, 300);
  motion_start_turn_deg(90, 250);
  motion_start_distance_mm(3000, 300);

重点观察串口：
  EVT 中不应无故出现 ENCFAULT。
  MOT 中 ef 应保持 0。
  dt 通常为 1；OLED刷新期间偶尔大于1是允许的。
  SYS 中 ov 可能增加，表示主循环确实错过过节拍，但控制器会按真实时间修正，
  不再快速补跑虚假5 ms周期。

新的故障保护参数
================
APP_ENCODER_FAULT_MIN_PWM = 180
APP_ENCODER_FAULT_STARTUP_GRACE_TICKS = 100  (500 ms)
APP_ENCODER_FAULT_WINDOW_TICKS = 100         (500 ms)
APP_ENCODER_FAULT_MIN_PROGRESS_COUNT = 12

若再次出现短距离停止
====================
请保留停止前后的 EVT、MOT、SYS 行。重点查看：
  状态是否为 ENCFAULT、DONE 或 TIMEOUT；
  ef、dt、fg、ov；
  lp/rp、tl/tr、sl/sr、pl/pr。
