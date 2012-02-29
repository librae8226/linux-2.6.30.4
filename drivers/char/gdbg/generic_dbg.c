#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/slab.h>
#include <asm/io.h>

#include <mach/regs-gpio.h>

#include "gdbg.h"

#define GDBG_BUFFER_SIZE    256

struct gdbg_dev *gdbg_device = NULL;
static unsigned int open_cnt = 0;
static unsigned char gdbg_buffer[GDBG_BUFFER_SIZE] = {0};

static int gdbg_init(void);
static void gdbg_exit(void);

struct file_operations gdbg_fops = {
	.owner = THIS_MODULE,
	.open = gdbg_open,
	.release = gdbg_release,
	.read = gdbg_read,
	.write = gdbg_write,
	.ioctl = gdbg_ioctl,
	.llseek = gdbg_llseek,
};

/*
 * @brief   open this device
 * @param   inode -i- device node
 *          filp -i- file(device) pointer
 * @return  a fault number
 * @note    none
 */
int gdbg_open(struct inode *inode, struct file *filp)
{
	struct gdbg_dev *dev = NULL;

	if (open_cnt > 0)
		return -ERESTARTSYS;
	open_cnt++;

	dev = container_of(inode->i_cdev, struct gdbg_dev, cdev);
	filp->private_data = dev;

	return 0;
}

/*
 * @brief   release this device
 * @param   inode -i- device node
 *          filp -i- file(device) pointer
 * @return  0
 * @note    none
 */
int gdbg_release(struct inode *inode, struct file *filp)
{
	open_cnt--;
	return 0;
}

/*
 * @brief   read a bunch of data from module buffer (kernel space)
 * @param   filp -i- file(device) pointer
 *          buf -i/o- user space buffer (destination)
 *          count -i- number of bytes to write
 *          f_pos -i/o- offset of the base of kernel space buffer to read
 * @return  a fault number
 * @note    none
 */
ssize_t gdbg_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	loff_t pos = *f_pos;

	if (pos >= GDBG_BUFFER_SIZE)
		return count;

	if (count > (GDBG_BUFFER_SIZE - pos))
		count = GDBG_BUFFER_SIZE - pos;
	pos += count;

	if (copy_to_user(buf, gdbg_buffer + *f_pos, count)) {
		return -EFAULT;
	}

	*f_pos = pos;

	return count;
}

/*
 * @brief   write a bunch of data to module buffer (kernel space)
 * @param   filp -i- file(device) pointer
 *          buf -i/o- user space buffer (source)
 *          count -i- number of bytes to write
 *          f_pos -i/o- offset of the base of kernel space buffer to write
 * @return  number of bytes of data written
 * @note    none
 */
ssize_t gdbg_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos)
{
	loff_t pos = *f_pos;

	if (pos > GDBG_BUFFER_SIZE)
		return -ENOMEM;

	if (count > (GDBG_BUFFER_SIZE - pos))
		count = GDBG_BUFFER_SIZE - pos;
	pos += count;

	if (copy_from_user(gdbg_buffer + *f_pos, buf, count)) {
		return -EFAULT;
	}

	*f_pos = pos;

	return count;
}

/*
 * @brief   driver ioctl
 * @param   inode -i- device node
 *          filp -i- file(device) pointer
 *          cmd -i- ioctl command
 *          arg -i- ioctl argument
 * @return  a fault number
 * @note    none
 */
int gdbg_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
	case COMMAND1:
		printk(KERN_INFO "%s: command1 invoked\n", __FUNCTION__);
		break;
	case COMMAND2:
		printk(KERN_INFO "%s: command2 invoked\n", __FUNCTION__);
		break;
	case CMD_WRITE_REG:
#if 0
		memcpy((volatile unsigned int *)arg, gdbg_buffer,
					sizeof(unsigned int));
#endif
		break;
	case CMD_READ_REG:
#if 0
		memcpy(gdbg_buffer, (volatile unsigned int *)arg,
				sizeof(unsigned int));
#endif
		break;
	default:
		printk(KERN_WARNING "%s: no such command\n", __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

/*
 * @brief   seek in device file
 * @param   filp -i- file(device) pointer
 *          off -i/o- seek offset
 *          whence -i- seek type
 * @return  a fault number
 * @note    none
 */
loff_t gdbg_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t pos;
	pos = filp->f_pos;

	switch (whence) {
	case SEEK_FIXED:
		pos = off;
		break;
	case SEEK_INC:
		pos += off;
		break;
	default:
		printk(KERN_WARNING "%s: no such whence\n", __FUNCTION__);
		return -EINVAL;
	}

	if ((pos > GDBG_BUFFER_SIZE) || (pos < 0)) {
		return -EINVAL;
	}

	return filp->f_pos = pos;
}

static int gdbg_init(void)
{
	int result = 0;
	dev_t devno = 0;

	printk(KERN_INFO "enter %s\n", __FUNCTION__);

	devno = MKDEV(GDBG_MAJOR, GDBG_MINOR);

	result = register_chrdev_region(devno, 1, "gdbg");
	if (result < 0) {
		printk(KERN_WARNING "gdbg: can't get major %d\n", GDBG_MAJOR);
		return result;
	}

	gdbg_device = kmalloc(sizeof(struct gdbg_dev), GFP_KERNEL);
	if (gdbg_device == NULL) {
		gdbg_exit();
		return -ENOMEM;
	} else
		memset(gdbg_device, 0, sizeof(struct gdbg_dev));

	cdev_init(&gdbg_device->cdev, &gdbg_fops);
	gdbg_device->cdev.owner = THIS_MODULE;
	gdbg_device->cdev.ops = &gdbg_fops;

	result = cdev_add(&gdbg_device->cdev, devno, 1);
	if (result) {
		printk(KERN_NOTICE "error %d adding gdbg\n", result);
		gdbg_exit();
		return result;
	}

	printk(KERN_INFO "Chip ID: 0x%08X\n", __raw_readl(S3C2410_GSTATUS1));
	return 0;
}

static void gdbg_exit(void)
{
	dev_t devno = 0;

	devno = MKDEV(GDBG_MAJOR, GDBG_MINOR);

	if (gdbg_device != NULL) {
		cdev_del(&gdbg_device->cdev);
		kfree(gdbg_device);
	}

	unregister_chrdev_region(devno, 1);

	printk(KERN_INFO "Chip ID: 0x%08X\n", __raw_readl(S3C2410_GSTATUS1));
	printk(KERN_INFO "leave %s\n", __FUNCTION__);
}

module_init(gdbg_init);
module_exit(gdbg_exit);

MODULE_AUTHOR("yrliao");
MODULE_DESCRIPTION("Generic Debug Module");
MODULE_LICENSE("GPL");
