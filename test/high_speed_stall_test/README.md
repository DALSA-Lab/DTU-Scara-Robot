# High speed stall detection

This test is to examine the performance of the speed adaptive stall threshold.

The speeds are in motor units, not joint units!

## Setup

For an unconstrained joint motor, a ramp from 0 to 120 rad/s





Development: Spikey PID error makes problems. First remove large spikes, then fast LP filter to remove remaining spikes of smaller amplitude. Needs to be fast in particular in low speed regime difficult to detect stall. 

Two regimes with a breakpoint before the "resonance". Reason for systematic increase in PID error in speeds between 15 rad/s and 40 rad/s is unclear. Peak at higher speeds only appears when decelerating.

Tuning the breakpoint might be necessary but it is difficult since all other joints cant move freely. 

Threshold is set relatively high, making threshold linear to speed as well has little impact at high threshold values.

High speed threshold leaves plenty of room. Also when stalling in highspeed regime threshold is pulled down.



SG_RESULT did not indicate stall, further tuning would be necessary but it would also suffer from limited range.

### J3

First up ramp up to 5 rad/s

than down ramp from 5 rad/s. Higher PID error at upper end but could be caused by previous acceleration

using max accel and vel from test

### J2

