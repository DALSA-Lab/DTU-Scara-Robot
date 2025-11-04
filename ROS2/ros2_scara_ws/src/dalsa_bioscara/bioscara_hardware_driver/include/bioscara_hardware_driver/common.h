/**
 * @file common.h
 * @author Sebastian Storz
 * @brief A file containing utility macros and functions
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef COMMON_H
#define COMMON_H

/**
 * @brief Macro to dump a buffer to cout
 *
 * @param buffer pointer to a buffer to dump to the console
 * @param size number of bytes to dump
 */
#define DUMP_BUFFER(buffer, size)     \
  {                                   \
    std::cout << "Buffer dump: ";     \
    for (size_t i = 0; i < size; i++) \
    {                                 \
      printf("%#x ", buffer[i]);      \
    }                                 \
    std::cout << std::endl;           \
  }

  /**
   * @brief Macro which executes a function and returns from the calling function with the error code if the called function fails.
   * 
   * Adapted from the [ESP-IDF](https://github.com/espressif/esp-idf/blob/ff97953b32a32e44f507593320b50d728eea3f06/components/esp_common/include/esp_check.h#L19)
   */
#define RETURN_ON_ERROR(x)     \
  do                           \
  {                            \
    int err_rc_ = (x);         \
    if (unlikely(err_rc_ < 0)) \
    {                          \
      return err_rc_;          \
    }                          \
  } while (0)

#endif // COMMON_H