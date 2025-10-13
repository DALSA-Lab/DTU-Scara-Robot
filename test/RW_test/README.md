# Joint read/write speed test
This test is to quantify where the bottleneck for the read/write loop in the HW interface is. 

## Assumptions
In HW interface joints are in unordered map, each R/W a "find" operation needs to find the joint corresponding to the state interface that is to be read. This could be a potential slow down since is needs to compare strings. Potentially investigate how the mock component is doing it.


## Baseline test
Read: 860 us on avg
Write: 1216 us on avg


