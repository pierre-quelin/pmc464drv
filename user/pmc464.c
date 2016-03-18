
#if defined(__linux__)

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/byteorder.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h> /* int32_t, uint32_t*/
#include "pmc464_pci.h"
#else

/* Windows en premiere approche */
#include <windows.h>

#endif

#include "pmc464.h"

#define APMC_SOFTWARE_RESET   0x4000   /* Software reset */
#define APMC_INT_ENABLE       0x0001   /* Interrupt enable */

typedef struct
{
   uint32_t* addr;
   uint32_t value;
} PMC464_DEVIO_REG;


/**
 * pmc464 handle
 */
typedef struct
{
   int    fd; /**< The file descriptor */
   struct pmc464reg* addr; /**< The memory area */
} PMC464HND;


/**
 * Open a pmc464 device
 *
 * @param dev The device. (ex. "/dev/pmc464_0")
 * @return An interbus master handle. A value of -1 is returned in case of
 * failure.
 */
PMC464 pmc464Open( const char* dev )
{
#if defined(__linux__)
   PMC464HND* hdl = NULL;

   hdl = malloc(sizeof(PMC464HND));

   hdl->fd = open(dev, O_RDWR);
   if( hdl->fd == -1 )
   {
      free(hdl);
      return (PMC464HND*)-1;
   }

   PMC464_IOCTL ioctl_param;
   (void)ioctl(hdl->fd, PMC464_IOCG_IO_ADDR, &ioctl_param);
   hdl->addr = (struct pmc464reg*)ioctl_param.addr;

   /* software reset board */
   pmc464WriteWord( hdl, &hdl->addr->InterruptRegister, APMC_SOFTWARE_RESET );

   /* Enable interrupts */
   /*pmc464WriteWord( hdl, &hdl->addr->InterruptRegister, APMC_INT_ENABLE );*/

   return (PMC464)hdl;
#else
   (void)dev;

   return 0;
#endif
}

struct pmc464reg* pmc464Addr( PMC464 pmc464 )
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   return hdl->addr;
#else
   (void)pmc464;

   return 0;
#endif
}

/**
 * Close a pmc464 device.
 *
 * @param pmc464 handle.
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int pmc464Close( PMC464 pmc464 )
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   close(hdl->fd);

   free(hdl);
#else
   (void)pmc464;
#endif

   return 0;
}


uint8_t pmc464ReadByte(PMC464 pmc464, uint8_t* addr)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = (uint32_t*)addr;
   reg.value = 0;
   (void)read( hdl->fd, (void*)&reg, 1 );

   return( __le32_to_cpu( reg.value ) );
#else
   (void)pmc464;
   (void)addr;

   return 0;
#endif
}

uint16_t pmc464ReadWord(PMC464 pmc464, uint16_t* addr)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = (uint32_t*)addr;
   reg.value = 0;
   (void)read( hdl->fd, (void*)&reg, 2 );

   return( __le32_to_cpu( reg.value ) );
#else
   (void)pmc464;
   (void)addr;

   return 0;
#endif
}

uint32_t pmc464ReadLong(PMC464 pmc464, uint32_t* addr)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = addr;
   reg.value = 0;
   (void)read( hdl->fd, (void*)&reg, 4 );

   return( __le32_to_cpu( reg.value ) );
#else
   (void)pmc464;
   (void)addr;

   return 0;
#endif
}


void pmc464WriteByte(PMC464 pmc464, uint8_t* addr, uint8_t value)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = (uint32_t*)addr;
   reg.value = __cpu_to_le32(value);
   (void)write( hdl->fd, (void*)&reg, 1 );
#else
   (void)pmc464;
   (void)addr;
   (void)value;
#endif
}

void pmc464WriteWord(PMC464 pmc464, uint16_t* addr, uint16_t value)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = (uint32_t*)addr;
   reg.value = __cpu_to_le32(value);
   (void)write( hdl->fd, (void*)&reg, 2 );
#else
   (void)pmc464;
   (void)addr;
   (void)value;
#endif
}

void pmc464WriteLong(PMC464 pmc464, uint32_t* addr, uint32_t value)
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_DEVIO_REG reg;
   reg.addr = (uint32_t*)addr;
   reg.value = __cpu_to_le32(value);
   (void)write( hdl->fd, (void*)&reg, 4 );
#else
   (void)pmc464;
   (void)addr;
   (void)value;
#endif
}


/**
 * Wait one or more interrupt event.
 *
 * @param a pmc464 handle.
 * @param timeout The maximum delay to wait (ms). A value of -1 indicate
 * forever.
 * @return A value of 0 is returned in
 * case of failure or timeout.
 */
int pmc464IsrHandler( PMC464 pmc464, int timeout )
{
#if defined(__linux__)
   PMC464HND* hdl = (PMC464HND*)pmc464;

   PMC464_IOCTL ioctl_param;
   ioctl_param.timeout = timeout;

   return ioctl(hdl->fd, PMC464_IOCQ_ISR_HND, &ioctl_param);
#else
   (void)pmc464;
   (void)timeout;

   Sleep(timeout);

   return 0;
#endif
}
