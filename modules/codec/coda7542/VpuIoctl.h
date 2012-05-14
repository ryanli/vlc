/*
 * VpuIoctl.h
 */
#ifndef _VPU_IOCTL_H_
#define _VPU_IOCTL_H_
#include <linux/ioctl.h>

#define MAJOR_NUM 250

#define IOCTL_BIT_RUN_COMPLETE     _IO(MAJOR_NUM, 0)
#define IOCTL_DEC_SEQ_INIT         _IO(MAJOR_NUM, 1)
#define IOCTL_DEC_SEQ_END          _IO(MAJOR_NUM, 2)
#define IOCTL_DEC_PIC_RUN          _IO(MAJOR_NUM, 3)
#define IOCTL_DEC_SET_FRAME_BUF    _IO(MAJOR_NUM, 4)
#define IOCTL_DEC_PARA_SET         _IO(MAJOR_NUM, 5)
#define IOCTL_DEC_BUF_FLUSH        _IO(MAJOR_NUM, 6)
#define IOCTL_DEC_BUF_EMPTY        _IO(MAJOR_NUM, 7)
#define IOCTL_ENC_BUF_FULL         _IO(MAJOR_NUM, 8)
#define IOCTL_GET_VPU_MEM_BASE     _IO(MAJOR_NUM, 9)

#endif
