#ifndef MRBUS_MACROS_H
#define MRBUS_MACROS_H

// Common macros for handling 16 bit variables
// These aren't strictly part of MRBus, but are used through the code and need to be defined
#ifndef UINT16_HIGH_BYTE
#define UINT16_HIGH_BYTE(a)  ((a)>>8)
#endif 

#ifndef UINT16_LOW_BYTE
#define UINT16_LOW_BYTE(a)  ((a) & 0xFF)
#endif 

#ifndef min
#define min(a,b)  ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a,b)  ((a)>(b)?(a):(b))
#endif


#endif 

