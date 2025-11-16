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

#endif // COMMON_H