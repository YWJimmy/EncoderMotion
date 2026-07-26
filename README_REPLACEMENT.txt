EncoderMotion replacement package v3 - UART telemetry
Base commit: cbc43a65fd6209bcaa882220162793415e70d705

1. File-name encoding
All directory names and file names in this ZIP use ASCII only. This text file is
UTF-8 with BOM so Chinese content displays correctly in Windows Notepad and CCS.

2. Replacement steps
1) Back up F:\CCS_workspace\EncoderMotion.
2) Copy every file in this package into the EncoderMotion project root.
3) Overwrite files with the same name.
4) Refresh the CCS project.
5) Open empty.syscfg once and save it, or build the project so SysConfig regenerates
   ti_msp_dl_config.c and ti_msp_dl_config.h.
6) Confirm app_tick.c and serial_log.c are visible and are not Exclude from Build.
7) Run Project -> Clean, then Build Project.

Files to overwrite:
  app_config.h
  empty.c
  empty.syscfg
  encoder.c
  encoder.h
  motion.c
  motion.h
  oled.c
  oled.h

Files to add:
  app_tick.c
  app_tick.h
  serial_log.c
  serial_log.h

3. UART connection and terminal settings
- Peripheral: UART0 TX
- MCU pin: PA10
- LaunchPad path: onboard XDS110 backchannel UART
- Baud rate: 115200
- Data format: 8 data bits, no parity, 1 stop bit (8N1)
- Flow control: none
- Line ending: CRLF

Keep the LaunchPad UART jumpers in the default XDS110 backchannel position. Open the
XDS110 Application/User UART COM port in a serial terminal. PA11 RX is not used by
this telemetry-only version.

4. Telemetry behavior
The fixed 5 ms motor-control task only creates text and places it into a 1024-byte
ring buffer. Actual UART transmission is performed outside the control tick by
serial_log_service(), which fills the hardware TX FIFO without waiting.

Periodic report interval: 250 ms. Change APP_SERIAL_REPORT_PERIOD_MS in app_config.h
to adjust it. Do not set it too low because OLED service and UART share idle time.

5. Output lines
BOOT line:
  BOOT,EncoderMotion UART telemetry,115200,8N1

Motion-state event:
  EVT,t=250,state=DIST

Motion-control data:
  MOT,t=250,st=DIST,lp=120,rp=118,tg=3594,rm=3475,bp=124,pl=122,pr=126

System and encoder data:
  SYS,t=250,lc=120,rc=118,lv=125,li=1,ld=2,rv=123,ri=0,rd=3,ol=1,oe=2,or=1,ov=0,qd=0

Field meanings:
  t   elapsed time in ms
  st  IDLE, DIST, TURN, BRAKE, DONE or TIMEOUT
  lp/rp normalized left/right progress counts for the current motion
  tg  target count
  rm  remaining average count
  bp  base PWM command
  pl/pr actual signed left/right PWM commands
  lc/rc absolute encoder counts
  lv/li/ld left valid/invalid/duplicate AB transitions
  rv/ri/rd right valid/invalid/duplicate AB transitions
  ol  OLED online flag: 1 online, 0 offline
  oe  OLED transaction error count
  or  OLED successful reconnect count
  ov  fixed-control-tick overrun count
  qd  dropped complete telemetry-message count

6. Existing v2 functions retained
- 16-entry AB quadrature transition table.
- Illegal transitions synchronize state without counting.
- Valid, invalid and duplicate transition statistics.
- OLED ACK detection, bus recovery and automatic reconnect.
- Fixed 5 ms SysTick control cadence.
- Average-progress motion stop criterion, PWM slew limiting and slow final approach.

7. First test
1) Build and flash with the USB cable connected.
2) Open Windows Device Manager and identify the XDS110 Application/User UART COM port.
3) Open that COM port at 115200 8N1.
4) Reset the board. BOOT and EVT lines should appear.
5) Start a menu motion. MOT and SYS lines should update every 250 ms.
6) Check li/ri. Persistent growth indicates illegal AB transitions or missed edges.
7) Check ov. It should normally remain zero.
8) Check qd. It should normally remain zero.
