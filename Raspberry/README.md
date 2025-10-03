## Setup on the Raspberry Pi
> [!CAUTION]
>
> This list is under development and not complete

- Install Ubuntu Server 24.04 LTS
- Install openssh-server
- Run the scripts provided in [/scripts](scripts)


## Realtime Requirements
Both of the following settings were set to the best realtime performance after the installation of Ubuntu Server 24.04 LTS on the Raspberry Pi. This is how to check and change them if necessary.

This set of kernel configuration is also set when installing the "Low Latency Ubuntu" (`sudo apt install linux-lowlatency`)

### Preemption Policy 

In mainline Ubuntu the maximum preemption policy is PREEMPT. Check if set using:
Check if enabled if CONFIG_PREEMPT is set:
```bash
cat /boot/<BUILD?> | grep CONFIG_PREEMPT
```
last \<BUILD\> = config-6.8.0-1028-raspi

> PREEMPT_VOLUNTARY: This config option adds preemption points to the kernel, enabling voluntary interrupts of low-priority processes. By providing faster application responses with only slightly reduced throughput, CONFIG_PREEMPT_VOLUNTARY suits desktop environments.
>
> PREEMPT: Enabling CONFIG_PREEMPT reduces the kernel latency by making low-priority processes involuntarily preempt themselves. Preemption is disabled only at critical locations where the kernel must protect data from concurrency. Such config option fits embedded applications with latency requirements in the order of milliseconds. [source](https://ubuntu.com/blog/industrial-embedded-systems-ii)

### Interrupt Timer Resolution
> The timer interrupt handler interrupts the kernel at a rate set by the HZ constant. The frequency affects the timer resolutions as a 100 Hz value for the timer granularity will yield a max resolution of 10ms (1 Hz equating to 1000ms), 250Hz will result in 4ms, and 1000Hz in the best-case resolution of 1ms. [source](https://ubuntu.com/blog/industrial-embedded-systems-ii)

```bash
cat /boot/<BUILD?> | grep CONFIG_HZ
```
last \<BUILD\> = config-6.8.0-1028-raspi

