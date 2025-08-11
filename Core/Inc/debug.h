#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

#define ENABLE_SERIAL_DEBUG_PRINTOUTS 0

// converts printf to a no-op if debugging is disabled
#if !ENABLE_SERIAL_DEBUG_PRINTOUTS
    #undef printf
    #define printf(...) ((void)0)
#endif

#endif /* DEBUG_H */