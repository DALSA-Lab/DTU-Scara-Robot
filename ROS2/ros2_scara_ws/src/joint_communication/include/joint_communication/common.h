#ifndef COMMON_H
#define COMMON_H

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