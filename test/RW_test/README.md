# Communication Read/Write speed test
The realtime performance tests described in the [realtime test](../RT_test/README.md) showed slow execution times for both the read and write cycle. To be precise the test measured the following average cumulative execution times for all joints:
- **Read** (*getPosition*): 3575 μs
- **Write** (*setPosition*): 5062 μs

> [!NOTE]
>
> When conducting the realtime test the position joint trajectory controller was used. In the latest iteration the velocity joint trajectory controller is used. 

This test is to identify the bottleneck that causes the long execution times and to investigate if they can be increase to enable higher controller update rates.

## Description
The test consists of two units:
### Joint Firmware
<!-- TODO: describe and link how to flash firmware  -->
A firmware variant ([profiling_test.ino](../../Arduino/profiling_test/profiling_test.ino)) is flashed on the joint controller. This firmware times the execution of the `non_blocking_handler()` which handles the incoming commands and prints the results to the serial port in CSV format. The development machine stays connected to the joint via USB to capture the serial port output and save it to a file. The [Realterm](https://sourceforge.net/projects/realterm/) software was used to cpature and save the serial output. 
### Controller
<!-- TODO: describe and link how to build and source workspace  -->
The controller executes the system_test_packages/comm_speed_test program
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
ros2 run comm_speed_test main
```

<!-- TODO: write documentation for test program, link or reference here -->
The test program calls times the read and write execution of either position or velocity (`getPosition()`/getVelocity/() and `setPosition()`/setVelocity/() respectively) for a total of 1000x cycles per joint and prints the collected data to a CSV file.

No prints and other unneccessary functions are executed in the timed sections.

## I2C Protocol
The robot controller and joints communicat via an I2C bus. [This](https://www.ti.com/lit/an/slva704/slva704.pdf) document describes the protocol in detail, below the transmitted bytes are summarized in Table 1 below. Counting the number of transmitted bits is important since the I2C bus clock runs at a speed of 100 kHz (100 kbit/s) or 400 kHz (400 kbit/s).



|Sender|      |         |       |      |        |      |        |      |      |        |      |      |         |       |      |        |      |      |        |       | | Bits                 |
| ---- | ---- | ------- | ----- | ---- | ------ | ---- | ------ | ---- | ---- | ------ | ---- | ---- | ------- | ----- | ---- | ------ | ---- | ---- | ------ | ----- | ---- | ---------------- |
|*Controller* | S[1] | Addr[7] | Re[1] |      | Reg[8] |      | TX*1*[8] |      | ...  | TX*m*[8] |      | S[1] | Addr[7] | Re[1] |      |        | A[1] | ...  |        | NA[1] | P[1] | 27+*n*+8\**m*       |
|*Target* |     |         |       | A[1] |        | A[1] |        | A[1] | ...  |        | A[1] |      |         |       | A[1] | RX*1*[8] |      | ...  | RX*n*[8] |       |      | 3+8\**n*+*m*       |
|      |     |         |       |      |        |      |        |      |      |        |      |      |         |       |      |        |      |      |        |       |      | **30+9\**n*+9\**m*** |

**Table 1:** I2C transmission frames for a combined Read/Write. The number in [n] brackets are the number *n* of transmitted bits in this frame.

> [!NOTE]
>
> The sum of bits in Table 1 is only true for combined Read/Write transmission. In this application every transmission is a Read/Write transmission since at a minimum the return flags are always read from the joint. For a single write transmission with *m* transmitted bytes, the total number of transmitted bits is slightly different (repeated start and repeated address are ommited) and given as follows: *Controller* sends: 18+8\**m* bits, *Target* sends: 2+*m* bits, in **total:** 20+9\**m* bits are transmitted.

### Transmission Speed for set/get Position/Velocity
For the transmissions (get/set Position/Velocity) in questions float values of 4 byte size are transmitted:
|      |Write Bytes *m*|Read Bytes *n*| Total Bytes | Total Bits |
| ---- | ----: | ----: | ----: |  ----: | 
|`setPosition()`/`setVelocity()`| target [float]  4   | flags [uint8_t]  1   | 5 | 75 |
|`getPosition()`/`getVelocity()`|   0   |  value [float] + flags [uint8_t] 4 + 1   | 5 | 75 |
**Table 2:** Number of bytes and bits transmitted per function.

At the '*Standard Mode*' speed of 100 kbit/s the transmission of 75 bits takes then: **$t_{trans}$ = 750 μs** <br>
At the '*Fast Mode*' speed of 400 kbit/s the transmission of 75 bits takes then: **$t_{trans}$ = 187.5 μs**

## Total Execution time
The total execution time $t_{total}$ which was measured in the realtime-test is the sum of of the following contributions:
- Transmission $t_{trans}$: The formerly calculated theoretical tranmission time via I2C.
- Firmware execution time $t_{exec}$: The duration between receiving the command on the firmware, processing it in the `non_blocking_handler()` returning the state flags.
- Overhead $t_{oh}$: Similar to $t_{exec}$, program overhead, for example memory allocation or similar.
- Noise $t_{noise}$: Delays outside the programs control, for example preemptions. Note that the comm_speed_test program is run without realtime priorities and hence can be subject to preemption of higher priority threads. This will have an impact on worst case execution times but over 1000 samples the impact will be reduced.

$$
t_{total} = t_{trans} + t_{exec} + t_{oh} +t_{noise}
$$

## Tests
A series of individual tests have been conducted to gather the relevant data to determine the contributing factors described in the previous section.
- set/getPosition at 100 kHz
    - File: *[position_100khz.csv](position_100kHz.csv)*
- set/getPosition at 400 kHz
    - File: *[position_400khz.csv](position_400kHz.csv)*
- set/getVelocity at 100 kHz
    - File: *[velocity_100khz.csv](velocity_100kHz.csv)*
- set/getVelocity at 400 kHz
    - File: *[velocity_400khz.csv](velocity_400kHz.csv)*
- set/getPosition at 400 kHz, execution time on firmware.
    - File: *[baseline_joint_400khz.csv](baseline_joint_400kHz.csv)*

## Results
### Total Time
<table>
  <tr>
    <td><img src="position_100kHz.png" ><center><strong>Figure 1(a):</strong> Histogram of total time of get/setPosition() at 100 kHz</td>
    <td><img src="position_400kHz.png" ><center><strong>Figure 1(b):</strong> Histogram of total time of get/setPosition() at 400 kHz</td>
  </tr>
   <tr>
    <td><img src="velocity_100kHz.png" ><center><strong>Figure 1(c):</strong> Histogram of total time of get/setVelocity() at 100 kHz</td>
    <td><img src="velocity_400kHz.png" ><center><strong>Figure 1(d):</strong> Histogram of total time of get/setVelocity() at 400 kHz</td>
  </tr>
</table>


### set/getPosition at 100 kHz
File: *[position_100khz.csv](position_100kHz.csv)*. <br>

![](position_100kHz.png)

**Figure 1:** Total time $t_{total}$ histogram over 1000 samples per joint for `set/getPosition`.

|      | Mean [μs] | Max [μs]    |
| ---- | ----: | ----: |
|`getPosition()`|860.34 | 970.75|
|`setPosition()`|1216.62|1471.37|
**Table 3:** Mean and Max total time $t_{total}$ for `set/getPosition`

### set/getPosition at 400 kHz
File: *[position_400khz.csv](position_400khz.csv)*. <br>

![](position_400kHz.png)

**Figure 2:** Total time $t_{total}$ histogram over 1000 samples per joint for `set/getPosition` at 400 kHz.

|      | Mean [μs] | Max [μs]    |
| ---- | ----: | ----: |
|`getPosition()`|860.34 | 970.75|
|`setPosition()`|1216.62|1471.37|
**Table 4:** Mean and Max total time $t_{total}$ for `set/getPosition` at 400 kHz.



### Test 2, speed at 400 kHz
Read avg: 270.7 μs
write avg 624.7 μs

but both show big peaks at ~1700 μs

### Test 3, measure execution time on joint firmware
Big news:
average execution times in the request handler ISR (where the set and getPosition are executed) are the following:
Read t_total avg: 5.2 μs
write t_total avg 353.1 μs

this means:<br>
t_total (meas) = t_transmission (calc) + t_exec (meas) + t_overhead +t_noise <br>
t_overhead + t_noise  = t_total (meas) - t_exec (meas) - t_transmission (calc) 

For 100 kHz:<br>
write: t_overhead + t_noise = 1216 μs - 750 μs - 353.1 μs = 113 μs <br>
read: t_overhead + t_noise = 860 μs - 750 μs - 5.2 μs = 105 μs

For 400 kHz: <br>
write: t_overhead + t_noise = 624.7 μs - 187.5 μs - 353.1 μs = 84 μs <br>
read: t_overhead + t_noise = 270.7 μs - 187.5 μs - 5.2 μs = 78 μs

--> This shows the remaining overhead is consistent between the calls the biggest contribution to the slow execution is in the firmware.

It is confirmed that the call of `stepper.moveToAngle(q_set)` takes on avg 346.12 μs.

### Test 4, overall time with setVelocity, getVelocity at 400 kHz
write t_total: 318.5 μs avg.
read t_total: 266.8 μs avg.

still peaks at 3000 μs

### Test 5, overall time with setVelocity, getVelocity at 100 kHz
write t_total: 905.0 μs avg.
read t_total: 853.4 μs avg.

still peaks at 3000 μs

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


TODO: Missing exec on firmware for setVel but can argue that because much faster it is short and not as long as setPosition