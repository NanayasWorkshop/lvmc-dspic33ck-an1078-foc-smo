
#ifndef __DIAGNOSTICS_H
#define __DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

void DiagnosticsInit(void);

void DiagnosticsStepIsr(void);


void DiagnosticsStepMain(void);

#ifdef __cplusplus
}
#endif

#endif 
