// File: IOFunc.h

#ifndef _IOFUNC_H_
#define _IOFUNC_H_

#include "VpuApi.h"

typedef struct {
	unsigned char * buf;
	int size;
	int point;
	int count;
	int fillendbs;
} BufInfo;

#ifdef __cplusplus
extern "C" {
#endif

	/* Decode Function */
	int FillSdramBurst(BufInfo * pBufInfo, uint32_t targetAddr, phyaddr_t bsBufStartAddr, phyaddr_t bsBufEndAddr, uint32_t size, int endian, int checkeos, int *streameos);

	void StoreYuvImageBurst(unsigned char * dst, 
			int picWidth, 
			int picHeight,
			int addrY, 
			int addrCb, 
			int addrCr, 
			int stride, 
			int interLeave
			);
	void StoreYuvImageBurstFormat(uint8_t * dst, 
			int picWidth, 
			int picHeight,
			int addrY, 
			int addrCb, 
			int addrCr, 
			int stride, 
			int interLeave, 
			int format
			);
	void ReadSdramBurst(unsigned char buf[], int addr, int byteSize, int bigEndian);
	void WriteSdramBurst(unsigned char buf[], int addr, int byteSize, int bIsBigEndian);

#ifdef __cplusplus
}
#endif

#endif
