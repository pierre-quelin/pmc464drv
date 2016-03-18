
/**
 * Driver for pmc464 boards
 *
 * ref. www.acromag.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */


#ifndef PMC464_H_
#define PMC464_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

#define DEVICE_NAME	"pmc464" /* the name of the device */

#define PMC464_DEVS 4 /* maximum number of PMC boards */


/* Use 'p' as magic number */
#define PMC464_IOC_MAGIC  'p'

typedef union
{
   int32_t timeout;  /* used by PMC464_IOCQ_ISR_HND (in ms) */
   uint32_t addr;    /* used by PMC464_IOCG_IO_ADDR */
} PMC464_IOCTL;


/*
 * S means "Set" through a pointer
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 */
#define PMC464_IOCQ_ISR_HND   _IOW(PMC464_IOC_MAGIC, 1, PMC464_IOCTL)
#define PMC464_IOCG_IO_ADDR   _IOR(PMC464_IOC_MAGIC, 2, PMC464_IOCTL)
#define PMC464_IOC_MAXNR 2

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

/* Memory map */
struct map464
{
   uint16_t InterruptRegister;      /* Interrupt Register (Read/Write) */
   uint16_t unusedw1;               /* Unused */
   uint16_t CInterruptStatusReg;    /* Interrupt Status/Clear Counter/Timer Register (Read/Write) */
   uint16_t unusedw2;               /* Unused */
   uint16_t DInterruptStatusReg[4]; /* Interrupt Status/Clear Digital I/O (Read/Write) */
   uint16_t IOPort[4];              /* Digital Input/Output Registers (Read/Write) */
   uint16_t Direction[2];           /* Direction Control Register (Read/Write) */
   uint16_t InterruptEnableReg[4];  /* Interrupt Enable Registers (Read/Write) */
   uint16_t InterruptTypeReg[4];    /* Interrupt Type (Change Of State or H/L) Configuration Registers (Read/Write) */
   uint16_t InterruptPolarityReg[4];/* Interrupt Polarity Registers (Read/Write) */
   uint16_t CounterTrigger;         /* Counter Trigger Register (Write) */
   uint16_t CounterStop;            /* Counter Stop Register (Read/Write) */
   uint32_t CounterControl1;        /* Counter Control Registers (Read/Write) */
   uint32_t CounterControl2;        /* Counter Control Registers (Read/Write) */
   uint32_t CounterControl3;        /* Counter Control Registers (Read/Write) */
   uint32_t CounterControl4;        /* Counter Control Registers (Read/Write) */
   uint32_t CounterReadBack1;       /* Counter Readback Registers (Read Only) */
   uint32_t CounterReadBack2;       /* Counter Readback Registers (Read Only) */
   uint32_t CounterConstantA1;      /* Counter Constant A Registers (Write Only) */
   uint32_t CounterConstantA2;      /* Counter Constant A Registers (Write Only) */
   uint32_t CounterConstantB1;      /* Counter Constant B Registers (Read/Write) */
   uint32_t CounterConstantB2;      /* Counter Constant B Registers (Read/Write) */
   uint32_t DebounceDurationReg[8]; /* Debounce Duration Select and Enable Registers (Read/Write) */
};

#pragma pack(pop)   /* restore original alignment from stack */


#endif /* PMC464_H_ */
