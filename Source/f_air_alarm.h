/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __F_AIR_ALRM_H
#define __F_AIR_ALRM_H

#include "main_ini.h"


void initializeWatchdog (uint8_t prer,uint16_t rlr);
void resetWatchdog (void);
void checkError (void);

#endif /* __F_AIR_ALRM_H */
