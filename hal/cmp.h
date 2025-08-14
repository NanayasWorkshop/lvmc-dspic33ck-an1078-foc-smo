#ifndef __CMP_H
#define __CMP_H

    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
    extern "C" {
#endif
                
            
void CMP_Initialize(void);

void CMP1_ModuleEnable(bool);
void CMP1_ReferenceSet(uint16_t );

#ifdef __cplusplus 
    }
#endif
#endif      // end of __CMP_H
    