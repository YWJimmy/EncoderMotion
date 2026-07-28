EncoderMotion 稳定控制修复包 v6

适用仓库：
https://github.com/YWJimmy/EncoderMotion

使用方法：
1. 备份当前工程。
2. 将本压缩包中的全部文件复制到 EncoderMotion 工程根目录。
3. 对所有同名文件选择覆盖。
4. CCS 中执行 Refresh、Project -> Clean、Build Project。
5. 首先测试 500 mm 直行，再测试正负 90 度转向，最后测试 3000 mm。

本包替换的文件：
- app_config.h
- app_tick.c / app_tick.h
- encoder.c / encoder.h
- motion.c / motion.h
- empty.c
- serial_log.c / serial_log.h

不需要删除任何 .c 或 .h 文件。
不要同时保留旧版 motion.c、wheel_speed_pi.c 等重复实现。仓库当前没有
wheel_speed_pi.c，因此正常情况下无需删除。

核心变化：
- 删除未经过实车标定的速度 PI 内环。
- 使用稳定的“位置减速轨迹 + P-only 左右同步”控制。
- 直行和转向使用独立的减速、最低 PWM 和同步参数。
- SysTick 只保留一个待处理请求，不补跑虚假 5 ms 控制周期。
- motion_update 使用真实 tick 间隔。
- 左右累计计数和增量在同一个临界区内原子读取。
- 完成条件要求左右轮都到达目标，避免平均值提前到达造成早停。
- 任何一轮到达目标后不会反转，只停止该轮输出，等待另一轮追上。
- 编码器失效保护改为 1 秒启动宽限 + 1 秒真实时间窗口。
- 新增 A/B 独立边沿次数和当前 AB 状态串口诊断。

重要标定：
- 编码器：1469 count/rev（基于左50圈73460、右58圈85193）。
- 有效轮径：66 mm。
- 转向有效轮距初值：114 mm，需用多次 360 度转向进一步标定。
- 为保证可控停车，直线命令最高限制为400，转向最高限制为300。即使调用时传入1000，也会按安全上限执行。

若直线仍稳定向右偏：
只在机械、轮胎、地线和编码器均稳定后，将
APP_STRAIGHT_RIGHT_TRIM_PERMILLE 从 1000 小幅提高到 1003~1010，
每次只改 2~3。不要先提高同步增益。
