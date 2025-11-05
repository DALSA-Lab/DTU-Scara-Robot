#ifndef UTRANSMISSION_H
#define UTRANSMISSION_H

/**
 * @brief Macro for a simple transmission from joint units to actuator units.
 *
 * The translation is based on the ros2_control transmission interface, simple transmission.
 * For position reduction and offset need to be used.\n
 * For velocity and acceleration only use reduction and NO offset \n
 * For effort/torque use 1/reduction and NO offset \n
 */
#define JOINT2ACTUATOR(in, reduction, offset) (reduction * (in - offset))

/**
 * @brief Macro for a simple transmission from actuator units to joint units.
 *
 * The translation is based on the ros2_control transmission interface, simple transmission.
 * For position reduction and offset need to be used.\n
 * For velocity and acceleration only use reduction and NO offset \n
 * For effort/torque use 1/reduction and NO offset \n
 */
#define ACTUATOR2JOINT(in, reduction, offset) (in / reduction + offset)

/**
 * @brief pi
 *
 */
#define M_PI 3.14159265358979323846

/**
 * @brief Macro to convert radians to degree
 *
 */
#define RAD2DEG(rad) (rad / M_PI * 180)

/**
 * @brief Macro to convert degree to radians
 *
 */
#define DEG2RAD(deg) (deg * M_PI / 180)
#endif //UTRANSMISSION_H