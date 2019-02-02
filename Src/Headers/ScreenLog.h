#pragma once
#include <stdio.h>

char * TimeStamp();

#if true
#define DBG(format, arg...) DO_DBG(format, ## arg)
#else
#define DBG(format, arg...)
#endif

#define DO_DBG(format, arg...)                              \
   {                                                        \
         fprintf(stderr, "%s %s: " format "\n" ,            \
            TimeStamp(), __FUNCTION__ , ## arg);            \
   }
