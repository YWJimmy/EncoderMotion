# 三轮检查记录

## 第一轮：架构与接口检查

- 从零建立时间、编码器、电机、运动、按键、OLED和UART模块。
- 编码器仅使用GPIO双边沿中断。
- 每次ISR只读取一次pending、一次输入、清除一次状态。
- ISR中无循环排空、无串口、无OLED、无浮点。
- 16项状态表覆盖全部previous/current组合。
- A/B同时挂起不猜测顺序，不计数。
- 左右count和delta采用一次临界区原子快照。
- 运动层不使用速度PI、积分项或微分项。

## 第二轮：严格C11编译与单元测试

主机桩环境编译选项：

```text
-std=c11 -Wall -Wextra -Werror -pedantic
```

全部源文件通过语法和告警检查。

编码器测试：

- 正向完整AB循环和为+4；
- 反向完整AB循环和为-4；
- 00↔01单通道抖动100次净和为0；
- 00↔11、01↔10非法跳变返回0；
- 重复状态返回0。

## 第三轮：控制模型与工程完整性

模型假设右侧驱动能力比左侧高5%。结果：

```text
500 mm：left=3533, right=3544, difference=-11, state=DONE
90°左转：left=-628, right=638, state=DONE
后退200 mm：左右均为负，state=DONE
```

工程检查：

- `.project`、`.cproject`、`.ccsproject`和`.ccxml`均通过XML解析；
- SysConfig中所有已分配引脚唯一；
- 只有一个`main()`；
- 只有一个`SysTick_Handler()`；
- 只有一个`GROUP1_IRQHandler()`；
- 不包含`app_tick.c/.h`或速度PI模块；
- ZIP内文件名均为ASCII，避免中文文件名乱码。

## 尚未在本环境完成

本环境没有CCS、TI-Clang和MSPM0 SDK，不能声称完成目标板链接和烧录。导入CCS后必须保存一次`empty.syscfg`并进行目标编译。若SysConfig产品版本不同，选择本机已安装版本后重新生成。

## v1.1 port-macro regression check

- Removed all references to nonexistent `USER_IO_PORT`.
- Verified every GPIO SysConfig instance used through `*_PORT` contains pins from one physical GPIO port only.
- Compiled all application `.c` files against a generated-header-compatible stub that intentionally does not define `USER_IO_PORT`.
