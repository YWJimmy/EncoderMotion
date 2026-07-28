# 串口日志字段

串口：UART0 TX=PA10，115200 8N1。

## EVT

```text
EVT,t=8340,state=DIST
```

状态：`IDLE`、`DIST`、`TURN`、`BRAKE`、`DONE`、`TIMEOUT`、`ENCFAULT`。

## MOT

```text
MOT,t=...,st=...,lp=...,rp=...,tg=...,lr=...,rr=...,bp=...,sp=...,pl=...,pr=...,dl=...,dr=...,ef=...
```

- `lp/rp`：本次任务左右归一化进度count
- `tg`：目标count
- `lr/rr`：左右剩余count
- `bp`：基础PWM
- `sp`：左右同步P修正
- `pl/pr`：实际有符号PWM命令
- `dl/dr`：最近一次原子快照中的编码器增量
- `ef`：bit0左编码器停转，bit1右编码器停转

## SYS

```text
SYS,t=...,lc=...,rc=...,lv=...,li=...,ld=...,la=...,lb=...,lm=...,ls=...,rv=...,ri=...,rd=...,ra=...,rb=...,rm=...,rs=...,ol=...,oe=...,or=...,ov=...,qd=...
```

- `lc/rc`：左右累计count
- `lv/rv`：合法四分之一状态转移数量
- `li/ri`：非法两位跳变数量
- `ld/rd`：读取到重复AB状态的数量
- `la/lb/ra/rb`：各GPIO通道挂起边沿数量
- `lm/rm`：同一侧A/B同时挂起，无法确定先后顺序的次数
- `ls/rs`：当前AB状态，0–3
- `ol`：OLED在线
- `oe/or`：OLED错误和重连次数
- `ov`：保留的调度超限计数
- `qd`：UART完整消息丢弃数
