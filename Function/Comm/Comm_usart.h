#include "main.h"

#define BUFFER_SIZE 128

void USER_IRQHandler(void);

void ScopeCommPro(void);


typedef union {
    float f; // 定义一个float类型的成员变量f
    char bytes[sizeof(float)]; // 定义一个char类型的数组bytes，大小与float相同
} FloatToBytesUnion;





