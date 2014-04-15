#ifndef __BITBAND_H_INCLUDED
#define __BITBAND_H_INCLUDED

#define BITBAND_PERI(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4))) 
#define BITBAND_SRAM(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4)))

#define BSRRL_REG_OFFSET		0x18
#define BSRRH_REG_OFFSET		0x1A



#endif