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

#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h> /* copy_*_user */
#include <linux/cdev.h>

#include "pmc464_pci.h"

#define PMC464_MAJOR 0 /* dynamic major by default */

static int pmc464_major = PMC464_MAJOR;
static int pmc464_devs  = PMC464_DEVS;
static int pmc464_debug = 0;

module_param(pmc464_major, int, 0);
module_param(pmc464_devs, int, 0);
module_param(pmc464_debug, int, 0644);

#define dprintk(args...) \
   do { \
      if (pmc464_debug) printk(KERN_DEBUG args); \
   } while (0)


/* enum used by PMC464_IOCQ_ISR_HND cf. isr_flag */
enum PMC464_ISR_FLAG
{
   PMC464_NO_ISR          = 0x00,
   PMC464_ISR_OCCURRED    = 0x01,
};
/**
 * pmc464 device capabilities
 */
struct pmc464_dev
{
   unsigned long membase; /**< I/O memory base address of the device */
   unsigned long memsize; /**< I/O memory size of the device */
   void __iomem* membase_mapped; /**< The device mapped I/O memory address */

   int irq; /**< The IRQ number */
   irqreturn_t (*isr)(int irq, void* dev); /**< The interrupt service routine */

   wait_queue_head_t wait_q_isr;  /**< Wait queue interrupt */
   int isr_flag; /**< The isr flag register */

   int minor; /**< Device minor number */
   struct cdev cdev;
};

/**
 * pmc464 driver capabilities
 */
struct pmc464_drv
{
   struct semaphore devCountMutex; /**< To protect devCount */
   int devCount; /**< Number of pmc464 board. Used for the minor number */
};

/**
 * TODO
 */
static struct pmc464_drv* pmc464;
static struct class* pmc464_class;


#define APMC_INT_DISABLE      0x0000   /* Disable all interrupts */
#define APMC_INT_ENABLE       0x0001   /* Enable interrupts */
#define APMC_INT_PENDING      0x0002   /* Interrupt pending bit (pending & enable) */
#define APMC_SOFTWARE_RESET   0x4000   /* Software reset */

void isr_pmc464(void* pAddr); /* interrupt handler for Pmc464 */


static ssize_t pmc464_read(struct file *file,
                           char *buf,
                           size_t length,
                           loff_t *offset)
{
   struct pmc464_dev* dev = file->private_data;
   unsigned long membase = (unsigned long)dev->membase_mapped; /* base memory */
   size_t available = dev->memsize; /* the size of available memory */
   unsigned long addr, ldata;
   unsigned short sdata;
   unsigned char cdata;

   get_user(addr, (unsigned long*)buf); /* pickup address */
   /* Protects access */
   if ( ( addr < membase ) ||
        ( addr > membase + available ) )
   {
      return (-EINVAL);
   }

   switch (length)
   {
   case 1: /* 8 bit */
      cdata = readb((void *) addr);
      ldata = (unsigned long) cdata; /* convert to long */
      break;
   case 2: /* 16 bit */
      sdata = readw((void *) addr);
      ldata = (unsigned long) sdata; /* convert to long */
      break;
   case 4: /* 32 bit */
      ldata = readl((void *) addr);
      break;
   default:
      cdata = sdata = addr = ldata = 0;
      return (-EINVAL);
      break;
   }

   put_user(ldata, (unsigned long *) (buf + (sizeof(unsigned long)))); /* update user data */

   dprintk("BRD %d pmc464_read addr:0x%04X size:%zu\n",
           dev->minor,
           (unsigned int)addr-(unsigned int)membase,
           length);

   return (length);
}

/**
 *
 */
static ssize_t pmc464_write(struct file* file,
                            const char* buf,
                            size_t length,
                            loff_t* offset)
{
   struct pmc464_dev* dev = file->private_data;
   unsigned long membase = (unsigned long)dev->membase_mapped; /* base memory */
   size_t available = dev->memsize; /* the size of available memory */
   unsigned long addr, ldata;

   get_user(addr, (unsigned long*) buf); /* pickup address */
   get_user(ldata, (unsigned long*) (buf + (sizeof(unsigned long)))); /* pickup data */

   /* Protects access */
   if ( ( addr < membase ) ||
        ( addr > membase + available ) )
   {
      return (-EINVAL);
   }

   switch (length)
   {
   case 1: /* 8 bit */
      writeb((int) ldata, (void*)addr);
      break;
   case 2: /* 16 bit */
      writew((int) ldata, (void*)addr);
      break;
   case 4: /* 32 bit */
      writel((int) ldata, (void*)addr);
      break;
   default:
      return (-EINVAL);
      break;
   }

   dprintk("BRD %d pmc464_write addr:0x%04X data:0x%04X size:%zu\n",
           dev->minor,
           (unsigned int)addr-(unsigned int)membase,
           (unsigned int)ldata,
           length);

   return (length);
}

/**
 * interbus_ioctl - control device.
 *
 * function manipulates the underlying device parameters of special files.
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @param cmd The request code
 * @param arg Untyped pointer to parameters
 * @return On success zero is returned.
 **/
static long pmc464_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
   struct pmc464_dev* dev = file->private_data;
   int err = 0;
   int retval = 0;
   PMC464_IOCTL ioctl_param;

   /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
    */
   if (_IOC_TYPE(cmd) != PMC464_IOC_MAGIC)
   {
      return -ENOTTY;
   }
   if (_IOC_NR(cmd) > PMC464_IOC_MAGIC)
   {
      return -ENOTTY;
   }

   /*
    * the direction is a bitmask, and VERIFY_WRITE catches R/W
    * transfers. `Type' is user-oriented, while
    * access_ok is kernel-oriented, so the concept of "read" and
    * "write" is reversed
    */
   if (_IOC_DIR(cmd) & _IOC_READ)
      err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
   else if (_IOC_DIR(cmd) & _IOC_WRITE)
      err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
   if (err)
   {
      return -EFAULT;
   }

   switch (cmd)
   {
   case PMC464_IOCQ_ISR_HND:
      if (copy_from_user(&ioctl_param, (void __user *)arg, sizeof(ioctl_param)))
         return -EFAULT;

      if (ioctl_param.timeout != -1)
      {
         if ( 0 >= wait_event_interruptible_timeout(dev->wait_q_isr,
                                                    dev->isr_flag != PMC464_NO_ISR,
                                                    msecs_to_jiffies(ioctl_param.timeout)) )
         {
            retval = -ERESTARTSYS;
            break;
         }
      }
      else
      {
         if ( 0 != wait_event_interruptible(dev->wait_q_isr,
                                            dev->isr_flag != PMC464_NO_ISR) )
         {
            retval = -ERESTARTSYS;
            break;
         }
      }
      dev->isr_flag = PMC464_NO_ISR;
      break;

   case PMC464_IOCG_IO_ADDR:/* return PMC address */
      ioctl_param.addr = (uint32_t)dev->membase_mapped;
      if (copy_to_user((void __user *)arg, &ioctl_param, sizeof(ioctl_param)))
         return -EFAULT;
      break;

   default: /* redundant, as cmd was checked against */
      printk(KERN_ERR "BRD %d pmc464_ioctl PMC464_IO_... ENOTTY !\n", dev->minor);
      retval = -ENOTTY;
      break;
   }

   return retval;
}

/**
 * Open an interbus device
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @return On success zero is returned.
 **/
static int pmc464_open(struct inode* inode,
                       struct file* file)
{
   struct pmc464_dev *dev;

   /*  Obtain the device */
   dev = container_of(inode->i_cdev, struct pmc464_dev, cdev);
   if( !dev )
   {
      return -ENXIO;
   }

   /* Increment the use count. */
   if( !try_module_get(THIS_MODULE) )
   {
      return -ENODEV;
   }

   file->private_data = dev;

   printk(KERN_INFO "BRD %d pmc464_open\n", dev->minor);

   return 0;
}

/**
 * Release an interbus device
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @return On success zero is returned.
 **/
static int pmc464_release(struct inode* inode,
                          struct file* file)
{
   struct pmc464_dev *dev = file->private_data;
   unsigned long membase = (unsigned long)dev->membase_mapped; /* base memory */
   uint16_t data;

   printk(KERN_INFO "BRD %d pmc464_release\n", dev->minor);

   /* Disable all interrupts */
   data = APMC_INT_DISABLE;
   writew(data, (void*)membase); /* Interrupt Register */

   /* Decrement the use count. */
   module_put(THIS_MODULE);

   return 0;
}

static struct file_operations pmc464_fops = {
   .owner            = THIS_MODULE, /* owner of the world */
   .read             = pmc464_read,
   .write            = pmc464_write,
   .unlocked_ioctl   = pmc464_ioctl,
   .open             = pmc464_open,
   .release          = pmc464_release,
};

static irqreturn_t pmc464_isr(int irq, void* handle)
{
   irqreturn_t ret = IRQ_NONE;
   struct pmc464_dev* dev = handle;
   struct map464* p_pmc464 = NULL;
   volatile unsigned short nValue;

   volatile int i;
   volatile uint16_t istat;

   p_pmc464 = (struct map464*)dev->membase_mapped;

   /* Call interrupt handler for any pending interrupts */
   nValue = readw((unsigned short*)&p_pmc464->InterruptRegister);
   if ( ( nValue & APMC_INT_ENABLE ) &&
        ( nValue & APMC_INT_PENDING ) )
   {
      dprintk( "INTERRUPT /dev/pmc464_%d \n", dev->minor);
      dprintk( "ISR Glb 0x%01X \n", (unsigned int)nValue );

      /* Counter/Timer ISR */
      istat = readw(&p_pmc464->CInterruptStatusReg);/* interrupt status */
      if ( 0 != istat ) /* non-zero if interrupt pending */
      {
         dprintk( "ISR Cnt 0x%02X\n", (unsigned int)istat );

         /* clear all pending interrupts */
         writew(istat, &p_pmc464->CInterruptStatusReg);

         dev->isr_flag = PMC464_ISR_OCCURRED;
         wake_up_interruptible(&dev->wait_q_isr);
      }

      /* Digital I/O ISR */
      for ( i = 0; i < 4; i++ )
      {
         /*  read status of module */
         istat = readw(&p_pmc464->DInterruptStatusReg[i]); /* interrupt status */
         if ( 0 != istat ) /* non-zero if interrupt pending */
         {
            dprintk( "ISR DIO 0x%02X\n", (unsigned int)istat );
            writew(istat, &p_pmc464->DInterruptStatusReg[i]);

            dev->isr_flag = PMC464_ISR_OCCURRED;
            wake_up_interruptible(&dev->wait_q_isr);
         }
      }

      ret = IRQ_HANDLED;
   }

   return ret;
}


/**
 * Probe an pmc464 PCI device.
 *
 * @param pci_dev The PCI device
 * @param id ID the kernel will use to associate devices to this driver.
 * @return On success zero is returned.
 **/
static int pmc464_pci_probe(struct pci_dev* pci_dev,
                                      const struct pci_device_id* id)
{
   int err;
   struct pmc464_dev *dev;
   uint16_t data;

   printk(KERN_INFO "pmc464 board found\n");

   dev = kzalloc(sizeof(struct pmc464_dev), GFP_KERNEL);
   if (!dev) {
      printk(KERN_ERR "pmc464_pci_probe - kzalloc failed !\n");
      return -ENOMEM;
   }

   if (pci_enable_device(pci_dev)) {
      printk(KERN_ERR "pmc464_pci_probe - pci_enable_device failed !\n");
      goto out_free;
   }

   if (pci_request_regions(pci_dev, DEVICE_NAME)) {
      printk(KERN_ERR "pmc464_pci_probe - pci_request_regions failed !\n");
      goto out_disable;
   }

   printk(KERN_INFO "BAR0 flag:0x%08X\n", (unsigned int)pci_resource_flags(pci_dev, 0) );

   /* PCI BAR 0 (I/O mem)*/
   if (!(pci_resource_flags(pci_dev, 0) & IORESOURCE_MEM)) {
      printk(KERN_ERR "pmc464_pci_probe - pci_resource_flags PCI BAR 0 (I/O mem) failed !\n");
      goto out_regions;
   }
   dev->membase = pci_resource_start(pci_dev, 0);
   dev->memsize = pci_resource_len(pci_dev, 0);
   dev->membase_mapped = ioremap_nocache(dev->membase, dev->memsize);
   if (!dev->membase_mapped) {
      printk(KERN_ERR "pmc464_pci_probe - ioremap_nocache failed !\n");
      goto out_regions;
   }

   /* pci_write_config_word */
   pci_set_master(pci_dev);
   pci_write_config_word(pci_dev, PCI_COMMAND, PCI_COMMAND_MEMORY);

   /* obtain minor number */
   down(&pmc464->devCountMutex);
   if (pmc464->devCount >= pmc464_devs) {
      up(&pmc464->devCountMutex);
      printk(KERN_ERR "pmc464_pci_probe - too many board !\n");
      goto out_unmap;
   }
   dev->minor = pmc464->devCount;
   pmc464->devCount++;
   up(&pmc464->devCountMutex);

   /* Store the IRQ */
   dev->irq = pci_dev->irq;
   dev->isr = pmc464_isr;

   /* Char device registration */
   cdev_init(&dev->cdev, &pmc464_fops);
   dev->cdev.owner = THIS_MODULE;
   dev->cdev.ops = &pmc464_fops;
   err = cdev_add (&dev->cdev, MKDEV(pmc464_major, dev->minor), 1);
   if (err) {
      printk(KERN_ERR "pmc464_pci_probe - cdev_add failed !\n");
      goto out_unmap;
   }

   dev->isr_flag = PMC464_NO_ISR;
   init_waitqueue_head(&dev->wait_q_isr);

   /* Reset board */
   data = APMC_SOFTWARE_RESET;
   writew(data, (void*)dev->membase_mapped); /* Interrupt Register */

   /* Connect interrupt */
   if (0 != request_irq(dev->irq, dev->isr, IRQF_SHARED, DEVICE_NAME, dev))
   {
      printk(KERN_ERR "pmc464_pci_probe - can't connect interrupt\n");
      goto out_cdev;
   }

   /* Create device in /dev */
   if (IS_ERR(device_create(pmc464_class, &pci_dev->dev,
             MKDEV(pmc464_major, dev->minor), NULL,
             "pmc464_%u", dev->minor))) {
      printk(KERN_ERR "pmc464_pci_probe - can't create device\n");
      goto out_interrupt;
   }

   /* Info */
   dprintk(" Card number: %d\n",
         dev->minor );
   dprintk(" Memory at: 0x%lx[size=0x%lx] mapped at: 0x%px\n",
         dev->membase,
         dev->memsize,
         dev->membase_mapped );
   dprintk(" IRQ: 0x%x\n",
         dev->irq );

   pci_set_drvdata(pci_dev, dev);

   return 0; /* succeed */

out_interrupt:
   free_irq(dev->irq, dev);
out_cdev:
   cdev_del(&dev->cdev);
out_unmap:
   iounmap(dev->membase_mapped);
out_regions:
   pci_release_regions(pci_dev);
out_disable:
   pci_disable_device(pci_dev);
out_free:
   kfree(dev);
   return -ENODEV;
}

/**
 * Remove an pmc464 PCI device.
 *
 * @param pci_dev The PCI device
 **/
static void pmc464_pci_remove (struct pci_dev* pci_dev)
{
   struct pmc464_dev* dev = pci_get_drvdata(pci_dev);

   free_irq(dev->irq, dev);

   device_destroy(pmc464_class, MKDEV(pmc464_major, dev->minor));

   cdev_del(&dev->cdev);

   down(&pmc464->devCountMutex);
   pmc464->devCount--;
   up(&pmc464->devCountMutex);

   pci_clear_master(pci_dev);
   pci_release_regions(pci_dev);
   pci_disable_device(pci_dev);
   pci_set_drvdata(pci_dev, NULL);
   iounmap(dev->membase_mapped);
   kfree(dev);
}



#define PCI_VENDOR_ID_ACROMAG 0x16D5
#define PCI_DEVICE_ID_PMC464  0x4248

static struct pci_device_id pmc464_pci_ids[ ] = {
   { PCI_DEVICE(PCI_VENDOR_ID_ACROMAG, PCI_DEVICE_ID_PMC464) },
   { 0, },
   };
MODULE_DEVICE_TABLE(pci, pmc464_pci_ids);


static struct pci_driver pmc464_pci_driver = {
   .name       = DEVICE_NAME,
   .id_table   = pmc464_pci_ids,
   .probe      = pmc464_pci_probe,
   .remove     = pmc464_pci_remove,
   };

/**
 * This function is called when the pmc464 module is loaded
 *
 * @return On success zero is returned.
 **/
static int __init pmc464_init(void)
{
   int retval;
   dev_t dev = MKDEV(pmc464_major, 0);

   /* Register device in sysfs */
   pmc464_class = class_create(THIS_MODULE, "pmc464");
   if (IS_ERR(pmc464_class)) {
      retval = PTR_ERR(pmc464_class);
      printk(KERN_ERR "pmc464_init - can't register pmc464 class\n");
      goto err;
   }

   /*
    * Register major, and accept a dynamic number.
    */
   if (pmc464_major) {
      retval = register_chrdev_region(dev, pmc464_devs, DEVICE_NAME);
   }
   else {
      retval = alloc_chrdev_region(&dev, 0, pmc464_devs, DEVICE_NAME);
      pmc464_major = MAJOR(dev);
   }
   if (retval < 0) {
      printk(KERN_ERR "pmc464_init - can't register chrdev region\n");
      goto err_class;
   }

   pmc464 = kzalloc(sizeof(struct pmc464_drv), GFP_KERNEL);
   if (!pmc464) {
      printk(KERN_ERR "pmc464_init - can't allocate memory\n");
      retval = -ENOMEM;
      goto err_unchr;
   }

   sema_init(&pmc464->devCountMutex, 1);
   pmc464->devCount = 0;

   return pci_register_driver(&pmc464_pci_driver);

err_unchr:
   unregister_chrdev_region(dev, pmc464_devs);
err_class:
   class_destroy(pmc464_class);
err:
   return retval;
}

/**
 * This function is called when the pmc464 module is unloaded
 **/
static void __exit pmc464_exit(void)
{
   pci_unregister_driver(&pmc464_pci_driver);

   kfree(pmc464);

   unregister_chrdev_region(MKDEV (pmc464_major, 0), pmc464_devs);

   class_destroy(pmc464_class);
}

module_init(pmc464_init);
module_exit(pmc464_exit);

MODULE_DESCRIPTION("Driver for pmc464 PCI boards");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
