# Quantifying the tracking error
This test is to quantify the tracking errors of position inputs with focus on ramp (constant velocity) reference.

## Tests


use J1
set ramp that is below max speed (max vel: pi rad/s)

ramp from -1.5707 to 1.5707 in 20s -> 0,15707 rad/s

collect data command and state interface (SP and PV)

save data of upwards ramp

repeat with different ramp gradients and and max accelerations

The error is defined as (SP-PV)/input amplitude (ramp gradient)

## Expected Result
Tracking error stays constant relative to ramp but depends linearly on maxAccel

### Recording:

```bash
ros2 bag record -e ^\/controller_manager.* -o record_<ramp_gradient>_<maxAccel>_<note>
```
recording one long ramp input is sufficient. 

After recording export to CSV:
replay topics with:
```bash
ros2 bag play record_<ramp_gradient>_<maxAccel>_<note> -p
``` 
listen and export topics:
```bash
ros2 topic echo /controller_manager/statistics/full --csv > record_<ramp_gradient>_<maxAccel>_<note>.csv
```

## Preliminary Results
|      | 0.15707 |      |
| ---- | ------- | ---- |
| 0.15707 | ~0.52   |      |
| 1.5707  |  ~0.07 |      |