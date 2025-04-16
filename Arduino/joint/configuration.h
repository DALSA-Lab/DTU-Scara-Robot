#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#if defined(J1)
#define ADR 0x11
#define HOMEINGRPM 15  // > 10
#define HOMEINGTIMEOUT 30
#define HOMEINGSENSITIVITY 10
#define HOMEINGCURRENT 50

#elif defined(J2)
#define ADR 0x12
#define HOMEINGRPM 40  // > 10
#define HOMEINGTIMEOUT 30
#define HOMEINGSENSITIVITY 10
#define HOMEINGCURRENT 50

#elif defined(J3)
#define ADR 0x13
#define HOMEINGRPM 40  // > 10
#define HOMEINGTIMEOUT 30
#define HOMEINGSENSITIVITY 10
#define HOMEINGCURRENT 50

#elif defined(J4)
#define ADR 0x14
#define HOMEINGRPM 40  // > 10
#define HOMEINGTIMEOUT 30
#define HOMEINGSENSITIVITY 10
#define HOMEINGCURRENT 50
#else
#error "No Joint has been defined. Define one of 'JX' where X 1,2,3,4"
#endif

#endif
