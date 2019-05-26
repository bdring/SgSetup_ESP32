## Trinamic TMC2130 StallGaurd2 Setup on ESP32

This is a simple little program to setup sensorless stall detection with a Trinamic TMC2130 driver on a stepper motor. This program runs on a ESP32 in the Arduino IDE

You send the following characters with a serial terminal to tune the settings until you can sense a stall.

```TMC2130 StallGuard2 test program
TMC2130 StallGuard2 test program

'+' = faster
'-' = slower
'1' = run
'0' = pause
'r' = reverse
'i' = inc sg
'd' = dec sg
Send '1' character to begin
```

The output will look like the following...

```RPM:10 SGT:24 SGV:0428 TSTEP: 01407 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0428 TSTEP: 01406 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0434 TSTEP: 01406 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0434 TSTEP: 01407 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0094 TSTEP: 01406 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0094 TSTEP: 01407 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0094 TSTEP: 01406 SG: 0 I:0410 
RPM:10 SGT:24 SGV:0000 TSTEP: 01407 SG: 1 I:0410 
RPM:10 SGT:24 SGV:0000 TSTEP: 01409 SG: 1 I:0410 
RPM:10 SGT:24 SGV:0000 TSTEP: 01407 SG: 1 I:0410
```

 ### Output

 * Typical: RPM:10 SGT:24 SGV:0442 TSTEP: 01407 SG:0 I:397
 * RPM = Current Spped of the motors
 * SGT = Current StallGuard Threashold
 * SGV = Current reported StallGaurd value
 * TSTEP = Time in microseconds between steps. Use this to determine TCOOLTHRS value
 * SG = Stall detected (1 or 0)
 * I = Motor current in mA 

### Tips

* StallGuard only works in coolstep mode. It will not work in the quiet Stealthchop modes
* It only works reliably in a narrow speed band. You will want to find this speed and 
use it for your homing. Start with about 10 RPM
* Play with RPM (+ & - keys) and StallGuard Threadhold (i & d keys) until you see high SGV  
while running and a SG=1 when stalling (or nearly stalling)
* Note the the TSTEP value and plug a slightly lower value in TCOOLTHRS. This will prevent SG=1 at at slower values.
