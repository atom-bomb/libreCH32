#include <stdio.h>

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H 1

extern int debug_print_verbosity;

#ifdef ENABLE_DEBUG_PRINTS
  #define TRACE_PRINTF(fmt, ...) \
     if (debug_print_verbosity > 3) \
     fprintf(stderr, "%s:%s:%d:"fmt,  __FILE__, __FUNCTION__, __LINE__ \
         ,##__VA_ARGS__)
  #define DEBUG_PRINTF(fmt, ...) \
     if (debug_print_verbosity > 2) \
     fprintf(stderr, "%s:%s:%d:"fmt,  __FILE__, __FUNCTION__, __LINE__ \
        ,##__VA_ARGS__)
  #define DEBUG_PRINT_HEX(addr, len) \
     if (debug_print_verbosity > 2) { \
       const unsigned char *dph_addr = addr; \
       int dph_remaining = len; \
       int dph_off = 0; \
       int dph_col = 0; \
       while(dph_remaining) { \
         if (dph_col == 0) \
           fprintf(stderr, "%s:%s:%d:%04x:%02x",\
             __FILE__, __FUNCTION__, __LINE__, dph_off, *dph_addr); \
         else \
           fprintf(stderr, " %02x", *dph_addr);\
         if (8 == ++dph_col) { \
           fprintf(stderr, "\n"); \
           dph_col = 0;\
         } /* if */ \
         dph_addr++; \
         dph_off++; \
         dph_remaining--; \
       } /* while */ \
       if (dph_col) \
         fprintf(stderr, "\n"); \
     } /* if */ 
  #define INFO_PRINTF(fmt, ...) \
     if (debug_print_verbosity > 1) \
     fprintf(stderr, "%s:%s:%d:"fmt,  __FILE__, __FUNCTION__, __LINE__ \
        ,##__VA_ARGS__)
  #define WARNING_PRINTF(fmt, ...) \
     if (debug_print_verbosity > 0) \
     fprintf(stderr, "%s:%s:%d:"fmt,  __FILE__, __FUNCTION__, __LINE__ \
        ,##__VA_ARGS__)
  #define ERROR_PRINTF(fmt, ...) \
     fprintf(stderr, "%s:%s:%d:"fmt,  __FILE__, __FUNCTION__, __LINE__ \
        ,##__VA_ARGS__)
#else
  #define TRACE_PRINTF(...) do {} while(false)
  #define DEBUG_PRINTF(...) do {} while(false)
  #define DEBUG_PRINT_HEX(...) do {} while(false)
  #define INFO_PRINTF(...) do {} while(false)
  #define WARNING_PRINTF(...) do {} while(false)
  #define ERROR_PRINTF(...) do {} while(false)
#endif 

#endif /* DEBUG_PRINT_H */
