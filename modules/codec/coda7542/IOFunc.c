// File: IOFunc.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "VpuMain.h"
#include "RegDefine.h"
#include "IOFunc.h"

//////////////////// SDRAM Read/Write Healper Function ////////////////////////////
extern volatile unsigned char* g_mem_base;

extern unsigned char* g_bitstreamBuffer;
extern unsigned char* g_frameBufferLA;
extern unsigned int vpu_mem_base;
static unsigned int total_bytes = 0;

#ifdef UNICORE_FILE_READ
	extern uint8_t			*pFileBuf ;
	extern FILE *			streamFp;
  extern uint32_t    bsSize ;
#endif

void WriteSdramBurst(unsigned char  buf[], int addr, int byteSize, int bIsBigEndian)
{
	memcpy( (void*)(g_mem_base + (addr - (vpu_mem_base+ADDR_PHY_MEM_BASE))), buf, byteSize);
}
void ReadSdramBurst(unsigned char buf[], int addr, int byteSize, int bigEndian)
{
	unsigned char* src =(unsigned char*) (g_mem_base + (addr - (vpu_mem_base+ADDR_PHY_MEM_BASE)));
	memcpy(buf, src, byteSize);
}
void WriteSdramInWord(const uint8_t * src, int dst, int size)
{
	while(size > 0){
		WriteReg(dst, *src++);
		size -= 4;
		dst += 4;
	}
}

void ReadSdramInWord(uint8_t * dst, int src, int size)
{
	while(size > 0){
		*dst++ = ReadReg(src);
		size -= 4;
		src += 4;
	}
}

void StoreYuvImageBurst(uint8_t * dst, int picWidth, int picHeight,
						int addrY, int addrCb, int addrCr, int stride, int interLeave )
{
	int y, nY, nCb, nCr, bufferWidth, bufferHeight;
	int addr;
	uint8_t * puc;

	nY = picHeight;
	nCb = nCr = picHeight/2;

	puc = dst;
	addr = addrY;

	bufferWidth = ( ( bufferWidth + 15 ) & ~15 );
	bufferHeight = ( ( bufferHeight + 31 ) & ~31 );


	if( picWidth == stride ) // for fast read
	{
		ReadSdramBurst((uint8_t *)( puc ), addr, ( picWidth * picHeight ), IMAGE_ENDIAN);

		if( interLeave == 1 )
		{
			uint8_t t0,t1,t2,t3,t4,t5,t6,t7;
			int i;
			uint8_t * pTemp;
			uint8_t * dstAddrCb;
			uint8_t * dstAddrCr;

			addr = addrCb;
			stride = stride;
			dstAddrCb = (uint8_t *)(puc + picWidth * picHeight);
			dstAddrCr = (uint8_t *)(puc + picWidth * picHeight * 5 / 4);

			pTemp = malloc(picWidth);
			if (!pTemp) {
				fprintf(stderr, "malloc() failed \n");
			}
			for (y = 0; y < nY / 2; ++y) {
				ReadSdramBurst((unsigned char *)pTemp, addr + stride * y, picWidth, IMAGE_ENDIAN);
				for (i = 0; i < picWidth; i += 8) {
					t0 = pTemp[i];
					t1 = pTemp[i + 1];
					t2 = pTemp[i + 2];
					t3 = pTemp[i + 3];
					t4 = pTemp[i + 4];
					t5 = pTemp[i + 5];
					t6 = pTemp[i + 6];
					t7 = pTemp[i + 7];
					*dstAddrCb++ = t0;
					*dstAddrCb++ = t2;
					*dstAddrCb++ = t4;
					*dstAddrCb++ = t6;
					*dstAddrCr++ = t1;
					*dstAddrCr++ = t3;
					*dstAddrCr++ = t5;
					*dstAddrCr++ = t7;
				}
			}
			free(pTemp);
		}
		else
		{			
			puc = dst + picWidth * picHeight;
			addr = addrCb;
			ReadSdramBurst((uint8_t *)puc, addr, ((picWidth * picHeight) / 4), IMAGE_ENDIAN);

			puc = dst + ((picWidth * picHeight) * 5 / 4);
			addr = addrCr;
			ReadSdramBurst((uint8_t *)puc, addr, ( ( picWidth * picHeight ) / 4), IMAGE_ENDIAN);			
		}

	}
	else
	{		
		for (y = 0; y < nY; ++y) {
			ReadSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, ((picWidth+7)/8)*8, IMAGE_ENDIAN);
		}
		
		if( interLeave == 1 )
		{
			int i;
			uint8_t * pTemp;
			uint8_t * dstAddrCb;
			uint8_t * dstAddrCr;

			addr = addrCb;
			stride = stride;
			dstAddrCb = (uint8_t *)(puc + picWidth * picHeight);
			dstAddrCr = (uint8_t *)(puc + picWidth * picHeight * 5 / 4);

			pTemp = malloc((picWidth + 7) /8 *8);
			if (!pTemp) {
				fprintf( stderr, "malloc() failed \n");
			}
			for (y = 0; y < nY / 2; y++) {
				ReadSdramBurst((unsigned char *)pTemp, addr + stride * y, (picWidth + 7) /8 *8, IMAGE_ENDIAN);
				for (i = 0; (i < picWidth); i+=2) {
					*dstAddrCb++ = pTemp[(i/8)*8 + (i%8)];
					*dstAddrCr++ = pTemp[(i/8)*8 + (i%8) + 1];
				}
			}
			free(pTemp);
		}
		else
		{
			stride /= 2;
			puc = dst + picWidth * picHeight;
			addr = addrCb;
			for (y = 0; y < nCb; ++y) {
				ReadSdramBurst((uint8_t *)(puc + y * picWidth/2), addr + stride * y, ((picWidth/2 + 7)/8)*8, IMAGE_ENDIAN); 
			}

			puc = dst + picWidth * picHeight * 5 / 4;
			addr = addrCr;
			for (y = 0; y < nCr; ++y) { 
				ReadSdramBurst((uint8_t *)(puc + y * picWidth/2), addr + stride * y, ((picWidth/2 + 7)/8)*8, IMAGE_ENDIAN);
			}
		}

	}

}

void StoreYuvImageBurstFormat(uint8_t * dst, int picWidth, int picHeight,
		int addrY, int addrCb, int addrCr, int stride, int interLeave, int format)
{
	int y, nY, nCb, nCr;
	int addr;
	int lumaSize, chromaSize, chromaStride, chromaWidth;
	uint8_t * puc;

	switch(format)
	{
		case MODE420:
			nY = picHeight;
			nCb = nCr = (picHeight+1) / 2;
			chromaSize = ((picWidth + 1) / 2) * ((picHeight+1) / 2);
			chromaStride = stride / 2;
			chromaWidth = (picWidth + 1) / 2;
			break;
		case MODE224:
			nY = picHeight;
			nCb = nCr = (picHeight+1) / 2;
			chromaSize = (picWidth) * ((picHeight+1) / 2);
			chromaStride = stride;
			chromaWidth = picWidth;
			break;
		case MODE422:
			nY = picHeight;
			nCb = nCr = picHeight;
			chromaSize = ((picWidth + 1)/2) * picHeight ;
			chromaStride = stride / 2;
			chromaWidth = (picWidth + 1) / 2;
			break;
		case MODE444:
			nY = picHeight;
			nCb = nCr = picHeight;
			chromaSize = picWidth * picHeight;
			chromaStride = stride;
			chromaWidth = picWidth;
			break;
		case MODE400:
			nY = picHeight;
			nCb = nCr = (picHeight+1) / 2;
			chromaSize = ((picWidth + 1) / 2) * ((picHeight+1) / 2);
			chromaStride = stride / 2;
			chromaWidth = (picWidth + 1) / 2;
			break;
	}

	puc = dst;
	addr = addrY;
	lumaSize = picWidth * picHeight;

	if( picWidth == stride ) // for fast read
	{
		ReadSdramBurst((uint8_t *)( puc ), addr, lumaSize, IMAGE_ENDIAN);

		if( interLeave == 1 )
		{
			uint8_t t0,t1,t2,t3,t4,t5,t6,t7;
			int i;
			uint8_t * pTemp;
			uint8_t * dstAddrCb;
			uint8_t * dstAddrCr;

			addr = addrCb;
			stride = stride;
			dstAddrCb = (uint8_t *)(puc + picWidth * picHeight);
			dstAddrCr = (uint8_t *)(puc + picWidth * picHeight * 5 / 4);

			pTemp = malloc(picWidth);
			if (!pTemp) {
				fprintf(stderr, "malloc() failed \n");
			}
			for (y = 0; y < nY / 2; ++y) {
				ReadSdramBurst((unsigned char *)pTemp, addr + stride * y, picWidth, IMAGE_ENDIAN);
				for (i = 0; i < picWidth; i += 8) {
					t0 = pTemp[i];
					t1 = pTemp[i + 1];
					t2 = pTemp[i + 2];
					t3 = pTemp[i + 3];
					t4 = pTemp[i + 4];
					t5 = pTemp[i + 5];
					t6 = pTemp[i + 6];
					t7 = pTemp[i + 7];
					*dstAddrCb++ = t0;
					*dstAddrCb++ = t2;
					*dstAddrCb++ = t4;
					*dstAddrCb++ = t6;
					*dstAddrCr++ = t1;
					*dstAddrCr++ = t3;
					*dstAddrCr++ = t5;
					*dstAddrCr++ = t7;
				}
			}
			free(pTemp);
		}
		else
		{			
			puc = dst + lumaSize;
			addr = addrCb;
			ReadSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);

			puc = dst + lumaSize + chromaSize;
			addr = addrCr;
			ReadSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);			
		}

	}
	else
	{		
		for (y = 0; y < nY; ++y) {
			ReadSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, (picWidth + 7) / 8 * 8, IMAGE_ENDIAN);
		}
		
		if( interLeave == 1 )
		{
			uint8_t t0,t1,t2,t3,t4,t5,t6,t7;
			int i;
			uint8_t * pTemp;
			uint8_t * dstAddrCb;
			uint8_t * dstAddrCr;

			addr = addrCb;
			stride = stride;
			dstAddrCb = (uint8_t *)(puc + picWidth * picHeight);
			dstAddrCr = (uint8_t *)(puc + picWidth * picHeight * 5 / 4);

			pTemp = malloc(picWidth);
			if (!pTemp) {
				fprintf( stderr, "malloc() failed \n");
			}
			for (y = 0; y < nY / 2; y++) {
				ReadSdramBurst((unsigned char *)pTemp, addr + stride * y, picWidth, IMAGE_ENDIAN);
				for (i = 0; i < picWidth; i += 8) {
					t0 = pTemp[i];
					t1 = pTemp[i + 1];
					t2 = pTemp[i + 2];
					t3 = pTemp[i + 3];
					t4 = pTemp[i + 4];
					t5 = pTemp[i + 5];
					t6 = pTemp[i + 6];
					t7 = pTemp[i + 7];
					*dstAddrCb++ = t0;
					*dstAddrCb++ = t2;
					*dstAddrCb++ = t4;
					*dstAddrCb++ = t6;
					*dstAddrCr++ = t1;
					*dstAddrCr++ = t3;
					*dstAddrCr++ = t5;
					*dstAddrCr++ = t7;
				}
			}
			free(pTemp);
		}
		else
		{
			puc = dst + lumaSize;
			addr = addrCb;
			for (y = 0; y < nCb; ++y) {
				// modify (if picWidth is not multiple of 8)
				ReadSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, (chromaWidth + 7)/8*8, IMAGE_ENDIAN); 
			}

			puc = dst + lumaSize + chromaSize;
			addr = addrCr;
			for (y = 0; y < nCr; ++y) { 
				ReadSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, (chromaWidth + 7)/8*8, IMAGE_ENDIAN);
			}
		}

	}

}

// implement ring buffer 
int FillSdramBurst(BufInfo * pBufInfo, uint32_t targetAddr, phyaddr_t bsBufStartAddr, phyaddr_t bsBufEndAddr, uint32_t size, int endian, int checkeos, int *streameos)
{
	uint8_t *uca;
	int bufPnt;
	int bufSize;
	uint32_t written = 0;
	int countby4;
	int room;
	uint8_t * pSrc;
	uint8_t * pDst;

	if(checkeos == 1 && (pBufInfo->point >= pBufInfo->size))
	{
#ifdef UNICORE_FILE_READ
	  int uni_ret = fread(pBufInfo->buf, sizeof(uint8_t), pBufInfo->size, streamFp);
    if(uni_ret<=0){
   		*streameos = 1;
	   	return written;
    }else{   
    	pBufInfo->buf = pFileBuf;
    	pBufInfo->size = uni_ret;
    	pBufInfo->point = 0;
    }
#else
		*streameos = 1;
		return written;
#endif
	}

	uca = malloc( size );
	memset( uca, 0x00, size );
	
	pDst = uca;
	pSrc = pBufInfo->buf + pBufInfo->point ;
	bufPnt = pBufInfo->point;
	bufSize = pBufInfo->size;
	written = 0;

	if((pBufInfo->size - pBufInfo->point) < (int)size)
		countby4 = (pBufInfo->size - pBufInfo->point) / 8;
	else
		countby4 = size / 8;


	while(written < size) 
	{
		if(countby4-- > 0)
		{
			*((uint32_t *)pDst) = *((uint32_t *)pSrc);
			pDst += 4;
			pSrc += 4;
			written += 4;
			bufPnt += 4;
		}
		else
		{
			*pDst++ = *pSrc++;
			written++;
			bufPnt++;
		}


		if(bufPnt == bufSize)
		{
			if(checkeos == 1)
				break;
			else
			{
				bufPnt = 0;
				pSrc = pBufInfo->buf;
			}
		}
	}

	if((targetAddr + size) > bsBufEndAddr)
	{
		room = bsBufEndAddr - targetAddr;
		WriteSdramBurst(uca, targetAddr, room, endian);
		WriteSdramBurst(uca+room, bsBufStartAddr, size - room, endian);
	}
	else
	{
		WriteSdramBurst(uca, targetAddr, size, endian);
	}

	pBufInfo->point = bufPnt;
	pBufInfo->count = written;
	free(uca);

	return written;
}
