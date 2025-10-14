# Joint read/write speed test
This test is to quantify where the bottleneck for the read/write loop in the HW interface is. 

## Assumptions
In HW interface joints are in unordered map, each R/W a "find" operation needs to find the joint corresponding to the state interface that is to be read. This could be a potential slow down since is needs to compare strings. Potentially investigate how the mock component is doing it.


## Baseline test
Read: 860 us on avg
Write: 1216 us on avg


## I2C

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

at 100 kHz 75 bits take: 75/100,000 = 0.7500 ms
at 400 kHz 75 bits take: 75/400,000 = 0.1875 ms

## Timing the lowest level I2C zip command (used for writing):
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


## Test 2, speed at 400 kHz
Read avg: 270.7 us
write avg 624.7 us

but both show big peaks at ~1700 us