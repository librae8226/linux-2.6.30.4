#ifndef __GDBG_H__
#define __GDBG_H__

/* device ID */
#define GDBG_MAJOR      224
#define GDBG_MINOR      0
#define COMMAND1        0x1
#define COMMAND2        0x2
#define CMD_WRITE_REG   0x55
#define CMD_READ_REG    0xAA
#define SEEK_FIXED      0x0
#define SEEK_INC        0x1

/* device structure */
struct gdbg_dev
{
    struct cdev cdev; /* char device structure */
};

int     gdbg_open(struct inode *inode, struct file *filp);
int     gdbg_release(struct inode *inode, struct file *filp);
ssize_t gdbg_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t gdbg_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
int     gdbg_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
loff_t  gdbg_llseek(struct file *filp, loff_t off, int whence);

#endif /* __GDBG_H__ */
