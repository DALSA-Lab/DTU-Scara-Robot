# Analyzing realtime requirements
This test is to analyze the impact of Kernel configurations on the execution times of the ros2_control main update loop. Missed deadlines could lead to poor controller performance.

## Original Approach (idea)
Cyclictest, why? common tool

Run in parallel to application. (recommended)

priority higher than ROS2_control? to measure the effect it would experience. otherwise that process would influence the latency of the measurement.

With and without Rviz

with and without realtime group


Ros control tries to use FIFO scheduling (RT scheduling class) at prio (higher is higher prio)

https://man7.org/linux/man-pages/man7/sched.7.html

https://blogs.oracle.com/linux/post/task-priority


cyclictest needs to run as sudo (so it can set any prio...)

without any adjustments ros2_control runs with priority in user space (non-realtime) 20 (0 niceness) or 120 in kernel space.

![](https://blogs.oracle.com/content/published/api/v1.1/assets/CONT7E6E157EFF004C1C9864737324E44A84/Medium?cb=_cache_6df6&format=jpg&channelToken=3189ef66cf584820b5b19e6b10792d6f)

### Tests

For all tests:

- run ros2_control in parallel with Rviz for worst case test and simulated system load. the test itself adds only minimal overhead.
- --mlockall/-m (Prevent Cyclictest pages from being paged out of memory)
- (run 30 min, the longer the higher the chance to catch max latency)
- (intervall 10ms like 100Hz controller) seems very high
- -S one measuring thread per cpu 

```bash
cyclictest -l100000000 -m -S -p<49/119> -i200 -h400 -q > output
```



1. Non-realtime priority, but higher than ros2_control (just a bit)
2. Add scara user to realtime group so that ros2_conrol can run as RT. Repeat with RT priority 51 for example, just slightly higher than Rviz runs at

Assumption:

in case (1) latency in the range of + 3ms

in case (2) no missed deadlines

## Problem

cyclictest only works with RT policies, so I cant test with non rt prio 19 for example


## Alternative Approach:
Directly collect statistics from the controller managers statistics diagnostics output. */controller_manager/statistics/*

### Update Rate

50 Hz -> 20 ms deadline

Assumption, increasing scheduler priority shows effect by reducing read/write times of the hardware interface

### System Load:

following ssh sessions (no X forwarding):

- ros2_control
- trajectory publisher
- ros2 bag recorder

Simple trajectory every 5s:

​	every 5 send trajectory:

​	j1: 0.0, j2: 0.2, j3: 0.0, j4: 0.0

​	j1: 0.7, j2: 0.2, j3: -0.7, j4: 0.7

### Recording:

```bash
ros2 bag record -e ^\/controller_manager.* -o record_<update_rate>_<prio>_<note>
```

10 min recording. The longer the recording the higher the chance to catch an uncommon latency.

After recording export to CSV:
replay topics with:
```bash
ros2 bag play record_<update_rate>_<prio>_<note> -p
``` 
listen and export topics:
```bash
ros2 topic echo /controller_manager/statistics/full --csv > record_<update_rate>_<prio>_<note>.csv
```
#### Test 1
- Position control, prio 20

#### Test 2
Add the user to the realtime group as described [here](../../Raspberry/README.md#adding-the-scara-user-to-realtime-group)

Then execute the test:
- Position control, RT prio 50, FIFO Scheduler (default ROS2 control values)


# Observation
Priority has no effect on execution time. 
Time spans much longer than what is caused through normal system latency.
Instead execution time linearly scales with the number of joints. 

I2C standard mode:
100kbit/s -> 12.5 kByte/s -> 80 us/byte


transmission of set position:
register + sizeof(float) + flags = 6 bytes


all joints:
R: avg 3575 us
W: avg 5062 us

2 joints:
R: avg 1875 us
W: avg 2615 us

3 joints:
R: avg 2697 us
W: avg 3815 us

# Evaluation
- [ ] Make Histogram of execution times
- [ ] Min/Max statistics

