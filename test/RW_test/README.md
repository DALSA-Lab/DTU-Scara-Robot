# Joint read/write speed test
This test is to quantify where the bottleneck for the read/write loop in the HW interface is. 

## Description
A on the controller side a test program calls the setPosition and getPosition methods for each joint 1000x and saves the execution time. 
On the joint firmware the execution time of the i2c request handler is measured.
No prints and other unneccessary are executed in the timed sections.

## I2C
[Link](https://www.ti.com/lit/an/slva704/slva704.pdf)

Read / Write:

|      |         |       |      |        |      |        |      |      |        |      |      |         |       |      |        |      |      |        |       |      |                  |
| ---- | ------- | ----- | ---- | ------ | ---- | ------ | ---- | ---- | ------ | ---- | ---- | ------- | ----- | ---- | ------ | ---- | ---- | ------ | ----- | ---- | ---------------- |
| S[1] | Addr[7] | Re[1] |      | Reg[8] |      | TX1[8] |      | ...  | TXm[8] |      | S[1] | Addr[7] | Re[1] |      |        | A[1] | ...  |        | NA[1] | P[1] | *27+n*+8*m       |
|      |         |       | A[1] |        | A[1] |        | A[1] | ...  |        | A[1] |      |         |       | A[1] | RX1[8] |      | ...  | RXn[8] |       |      | *3+8\*n*+m       |
|      |         |       |      |        |      |        |      |      |        |      |      |         |       |      |        |      |      |        |       |      | **30+9\*n+9\*m** |

setPosition:
n = 1, Flags
m = 4, target (float)
-> 75 bits

getPostition:
n = 5, value (float) + flags
m = 0
-> 75 bits

at 100 kHz 75 bits take: 75/100,000 = 750 us
at 400 kHz 75 bits take: 75/400,000 = 187.5 us

## Tests

### Baseline test, at 100 kHz
Read: 860 us on avg
Write: 1216 us on avg

### Test 1 Timing the lowest level I2C zip command (used for writing):
the zip command takes almost twice as long as it should.
It is weird since the same amount of data is transfered.
In the baseline test the read command is somewhat close to 750 us with an avg of 860.
The overhead of my code is negligbe since the average of the pure zip command is very close to the average when timed at the application level.

<details>
lgI2cZip, tx size: 4, rx size: 1, took: 1205.260<br>
lgI2cZip, tx size: 4, rx size: 1, took: 1178.963<br>
lgI2cZip, tx size: 4, rx size: 1, took: 1182.500<br>
...
lgI2cZip, tx size: 4, rx size: 1, took: 1211.500<br>
lgI2cZip, tx size: 4, rx size: 1, took: 1214.593<br>
  
</details>


### Test 2, speed at 400 kHz
Read avg: 270.7 us
write avg 624.7 us

but both show big peaks at ~1700 us

### Test 3, measure execution time on joint firmware
Big news:
average execution times in the request handler ISR (where the set and getPosition are executed) are the following:
Read t_total avg: 5.2 us
write t_total avg 353.1 us

this means:
t_total (meas) = t_transmission (calc) + t_exec (meas) + t_overhead +t_noise

For 100 kHz:
write: t_overhead + t_noise = 1216 us - 750 us - 353.1 us = 113 us
read: t_overhead + t_noise = 860 us - 750 us - 5.2 us = 105 us

For 100 kHz:
write: t_overhead + t_noise = 624.7 us - 187.5 us - 353.1 us = 84 us
read: t_overhead + t_noise = 270.7 us - 187.5 us - 5.2 us = 78 us

--> This shows the remaining overhead is consistent between the calls the biggest contribution to the slow execution is in the firmware.

It is confirmed that the call of `stepper.moveToAngle(q_set)` takes on avg 346.12 us.

### Test 4, overall time with setVelocity, getVelocity at 400 kHz
write t_total: 318.5 us avg.
read t_total: 266.8 us avg.

still peaks at 3000 us

### Test 5, overall time with setVelocity, getVelocity at 100 kHz
write t_total: 905.0 us avg.
read t_total: 853.4 us avg.

still peaks at 3000 us

## Conclusion
Can we use setVelocity, getPosition and getVelocity (thats whats needed for velocity command interface) at 100 kHz?
100 kHz is more robust regarding the long cable lengths.

| Method            | t_total (100 kHz) | t_total (400 kHz) |
| ---               |---:   |---:   |
|setVelocity        | 905  |319  |
|getVelocity        | 853  |267  |
|getPosition        | 860  | 625  |
| *subtotal*        | 2,618  |1,211  |
| 4 joint **total** | 10,472 | 4,844  |

Peaks might still occur, it would be interesting though if they might be reduced when called from higher priority thread.
For now continue with 400 kHz, 100 Hz update rate and measure i2c waveform with oscilloscope.