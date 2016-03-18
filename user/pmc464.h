/**
 * Library for Linux Driver pmc464
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU Lesser General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU Lesser General Public License is included in this distribution
 * in the file called "COPYING".
 */


#ifndef PMC464LIB_H_
#define PMC464LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void* PMC464;

struct pmc464reg
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

PMC464 pmc464Open( const char* devName );
int pmc464Close( PMC464 pmc464 );
struct pmc464reg* pmc464Addr( PMC464 pmc464 );
int pmc464IsrHandler( PMC464 pmc464, int timeout );
uint8_t pmc464ReadByte(PMC464 pmc464, uint8_t* addr);
uint16_t pmc464ReadWord(PMC464 pmc464, uint16_t* addr);
uint32_t pmc464ReadLong(PMC464 pmc464, uint32_t* addr);
void pmc464WriteByte(PMC464 pmc464, uint8_t* addr, uint8_t value);
void pmc464WriteWord(PMC464 pmc464, uint16_t* addr, uint16_t value);
void pmc464WriteLong(PMC464 pmc464, uint32_t* addr, uint32_t value);

#ifdef __cplusplus
}
#endif


#endif /* PMC464LIB_H_ */
