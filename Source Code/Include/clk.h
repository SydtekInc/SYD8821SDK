#ifndef _CLK_H_
#define _CLK_H_

#include "ARMCM0.h"
#include "config.h"

#define  SYD8821_RC         (RC_get())
#define  FCLK              (FCLK_get())
#define  PCLK              (PCLK_get())
#define  SCLK              (SCLK_get())
#define  HCLK              FCLK

uint32_t RC_get(void);
uint32_t FCLK_get(void);
uint32_t PCLK_get(void);
uint32_t SCLK_get(void);

#endif //_CLK_H_
