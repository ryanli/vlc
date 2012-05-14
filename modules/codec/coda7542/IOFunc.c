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

static void flushMem()
{
	unsigned char* ptr;
	ptr = (unsigned char*)malloc(64*1024*sizeof(unsigned char));
	memset(ptr, 0, 64*1024);
	free(ptr);
}

void WriteSdramBurst(unsigned char  buf[], int addr, int byteSize, int bIsBigEndian)
{
	
	memcpy( (void*)(g_mem_base + (addr - (vpu_mem_base+ADDR_PHY_MEM_BASE))), buf, byteSize);
//	flushMem();
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

void LoadYuvImage(uint8_t * src, int picWidth, int picHeight,
				  int addrY, int addrCb, int addrCr, int stride, int interLeave )
{
	int y, nY, nCb, nCr;
	int addr;
	uint8_t * puc;

	nY = picHeight;
	nCb = nCr = picHeight/2;

	puc = src;
	addr = addrY;
	for (y = 0; y < nY; ++y) {
		WriteSdramInWord((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth);
	}
	
	if( interLeave == 1 )
	{
		puc = src + picWidth * picHeight;
		addr = addrCb;	
		for (y = 0; y < nCb; ++y) {
			WriteSdramInWord((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth);
		}
	}
	else
	{
		stride /= 2;
		puc = src + picWidth * picHeight;
		addr = addrCb;
		for (y = 0; y < nCb; ++y) {
			WriteSdramInWord((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2);
		}

		puc = src + picWidth * picHeight * 5 / 4;
		addr = addrCr;
		for (y = 0; y < nCr; ++y) {
			WriteSdramInWord((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2);
		}
	}
}

void LoadYuvImageBurst(uint8_t * src, int picWidth, int picHeight,
		int addrY, int addrCb, int addrCr, int stride, int interLeave)
{
	int y, nY, nCb, nCr;
	int addr;
	uint8_t * puc;


	nY = picHeight;
	nCb = nCr = picHeight/2;

	puc = src;
	addr = addrY;

	if( picWidth == stride ) // for fast write
	{
		WriteSdramBurst((uint8_t *)( puc ), addr, ( picWidth * picHeight ), IMAGE_ENDIAN);

		if( interLeave == 1 )
		{
			puc = src + ( picWidth * picHeight );
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, ( picWidth * picHeight ) / 2, IMAGE_ENDIAN);
		}
		else
		{			
			puc = src + picWidth * picHeight;
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, ( picWidth * picHeight ) / 4, IMAGE_ENDIAN);
			
			puc = src + ( picWidth * picHeight * 5 / 4 );
			addr = addrCr;
			WriteSdramBurst((uint8_t *)puc, addr, ( picWidth * picHeight ) / 4, IMAGE_ENDIAN);			
		}
	}
	else
	{
		for (y = 0; y < nY; ++y) {
			WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
		}
		
		if( interLeave == 1 )
		{
			puc = src + picWidth * picHeight;
			addr = addrCb;	
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
			}
		}
		else
		{
			stride /= 2;
			puc = src + picWidth * picHeight;
			addr = addrCb;
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2, IMAGE_ENDIAN);
			}

			puc = src + picWidth * picHeight * 5 / 4;
			addr = addrCr;
			for (y = 0; y < nCr; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2, IMAGE_ENDIAN);
			}	
		}
		
	}
}
#ifdef JPEG_PARTIAL
void LoadYuvImageBurstFormatPartial( uint8_t * src, int picWidth, int picHeight, int picHeightPartial,
					   int addrY, int addrCb, int addrCr, int offsetLuma, int offsetChroma, int stride, int interLeave, int format )
{
	int y, nY, nCb, nCr;
	int addr;
	int imageLumaSize, imageChromaSize;
	int lumaSize, chromaSize, chromaStride, chromaWidth;
	uint8_t * puc;

	switch (format)
	{
		case MODE420:
			nY = picHeightPartial;
			nCb = nCr = picHeightPartial / 2;
			chromaSize = picWidth * picHeightPartial / 4;
			imageChromaSize = picWidth * picHeight / 4;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
			break;
		case MODE224:
			nY = picHeightPartial;
			nCb = nCr = picHeightPartial / 2;
			chromaSize = picWidth * picHeightPartial / 2;
			imageChromaSize = picWidth * picHeight / 2;
			chromaStride = stride;
			chromaWidth = picWidth;
			break;
		case MODE422:
			nY = picHeightPartial;
			nCb = nCr = picHeightPartial;
			chromaSize = picWidth * picHeightPartial / 2;
			imageChromaSize = picWidth * picHeight / 2;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
			break;
		case MODE444:
			nY = picHeightPartial;
			nCb = nCr = picHeightPartial;
			chromaSize = picWidth * picHeightPartial;
			imageChromaSize = picWidth * picHeight;
			chromaStride = stride;
			chromaWidth = picWidth;
			break;
		case MODE400:
			nY = picHeightPartial;
			nCb = nCr = picHeightPartial / 2;
			chromaSize = picWidth * picHeightPartial / 4;
			imageChromaSize = picWidth * picHeight / 4;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
			break;
	}
	
	puc = src+offsetLuma;
	addr = addrY;
	imageLumaSize = picWidth * picHeight;
	lumaSize = picWidth * picHeightPartial;

	if( picWidth == stride ) // for fast write
	{
		WriteSdramBurst((uint8_t *)( puc ), addr, lumaSize, IMAGE_ENDIAN);

		if( format == MODE400)
			return;

		if( interLeave == 1 )
		{
			puc = src + imageLumaSize + offsetChroma;
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, ( chromaSize * 2 ), IMAGE_ENDIAN);
		}
		else
		{			
			puc = src + imageLumaSize + offsetChroma;
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);
			
			puc = src + imageLumaSize + imageChromaSize + offsetChroma;
			addr = addrCr;
			WriteSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);			
		}
	}
	else
	{
		for (y = 0; y < nY; ++y) {
			WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
		}
		

		if( format == MODE400)
			return;

		if( interLeave == 1 )
		{
			puc = src + imageLumaSize + offsetChroma;
			addr = addrCb;	
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
			}
		}
		else
		{
			puc = src + imageLumaSize + offsetChroma;
			addr = addrCb;
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, chromaWidth, IMAGE_ENDIAN);
			}
			
			puc = src + imageLumaSize + imageChromaSize + offsetChroma;
			addr = addrCr;
			for (y = 0; y < nCr; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, chromaWidth, IMAGE_ENDIAN);
			}	
		}
		
	}
}
#endif // JPEG_PARTIAL

void LoadYuvImageBurstFormat( uint8_t * src, int picWidth, int picHeight,
					   int addrY, int addrCb, int addrCr, int stride, int interLeave, int format )
{
	int y, nY, nCb, nCr;
	int addr;
	int lumaSize, chromaSize, chromaStride, chromaWidth;
	uint8_t * puc;

	switch (format)
	{
		case MODE420:
			nY = picHeight;
			nCb = nCr = picHeight / 2;
			chromaSize = picWidth * picHeight / 4;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
			break;
		case MODE224:
			nY = picHeight;
			nCb = nCr = picHeight / 2;
			chromaSize = picWidth * picHeight / 2;
			chromaStride = stride;
			chromaWidth = picWidth;
			break;
		case MODE422:
			nY = picHeight;
			nCb = nCr = picHeight;
			chromaSize = picWidth * picHeight / 2;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
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
			nCb = nCr = picHeight / 2;
			chromaSize = picWidth * picHeight / 4;
			chromaStride = stride / 2;
			chromaWidth = picWidth / 2;
			break;
	}

	puc = src;
	addr = addrY;
	lumaSize = picWidth * picHeight;

	if( picWidth == stride ) // for fast write
	{
		WriteSdramBurst((uint8_t *)( puc ), addr, lumaSize, IMAGE_ENDIAN);

		if(format == MODE400)
			return;

		if( interLeave == 1 )
		{
			puc = src + lumaSize;
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, ( chromaSize * 2 ), IMAGE_ENDIAN);
		}
		else
		{			
			puc = src + lumaSize;
			addr = addrCb;
			WriteSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);

			puc = src + lumaSize + chromaSize;
			addr = addrCr;
			WriteSdramBurst((uint8_t *)puc, addr, chromaSize, IMAGE_ENDIAN);			
		}
	}
	else
	{
		for (y = 0; y < nY; ++y) {
			WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
		}

		if( format == MODE400)
			return;

		if( interLeave == 1 )
		{
			puc = src + lumaSize;
			addr = addrCb;	
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth, IMAGE_ENDIAN);
			}
		}
		else
		{
			puc = src + lumaSize;
			addr = addrCb;
			for (y = 0; y < nCb; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, chromaWidth, IMAGE_ENDIAN);
			}

			puc = src + lumaSize + chromaSize;
			addr = addrCr;
			for (y = 0; y < nCr; ++y) {
				WriteSdramBurst((uint8_t *)(puc + y * chromaWidth), addr + chromaStride * y, chromaWidth, IMAGE_ENDIAN);
			}	
		}

	}
}

void StoreYuvImage(uint8_t * dst, int picWidth, int picHeight,
				   int addrY, int addrCb, int addrCr, int stride, int interLeave )
{
	int y, nY, nCb, nCr;
	int addr;
	uint8_t * puc;

	nY = picHeight;
	nCb = nCr = picHeight/2;
	puc = dst;
	addr = addrY;
	for (y = 0; y < nY; ++y) {
		ReadSdramInWord((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth);
	}

	if( interLeave == 1 )
	{
		puc = dst + picWidth * picHeight;
		addr = addrCb;	
		for (y = 0; y < nCb; ++y) {
			ReadSdramInWord((uint8_t *)(puc + y * picWidth), addr + stride * y, picWidth);
		}
	}
	else
	{
		stride /= 2;
		puc = dst + picWidth * picHeight;
		addr = addrCb;
		for (y = 0; y < nCb; ++y) {
			ReadSdramInWord((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2);
		}

		puc = dst + picWidth * picHeight * 5 / 4;
		addr = addrCr;
		for (y = 0; y < nCr; ++y) {
			ReadSdramInWord((uint8_t *)(puc + y * picWidth/2), addr + stride * y, picWidth/2);
		}
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


void ProcessEncodedBitstreamBurst(FILE * fp, int targetAddr, phyaddr_t bsBufStartAddr, phyaddr_t bsBufEndAddr, int size, int endian)
{
	uint8_t * val = 0;
	int room = 0, sizeby4 = 0, roomby4 = 0;

	if((targetAddr + size) > (int)bsBufEndAddr)
	{
		room = bsBufEndAddr - targetAddr;
		roomby4 = ((room + 7) / 8) * 8;
		sizeby4 = (((size-room)  + 7) / 8) * 8;
		val = (uint8_t *)malloc(roomby4 + sizeby4);

		ReadSdramBurst(val, targetAddr, roomby4, endian);
		ReadSdramBurst(val+room, bsBufStartAddr, sizeby4, endian);
	}
	else
	{
		sizeby4 = (((size + 7) / 8) * 8);
		val = (uint8_t *)malloc(sizeby4);
		ReadSdramBurst(val, targetAddr, sizeby4, endian);
	}


	fwrite(val, sizeof(uint8_t), size, fp);

	free(val);
}

void ProcessEncodedBitstream(FILE * fp, int targetAddr, int size, int endian)
{
	unsigned int val;
	unsigned int val1;

	if(endian == 0){ // little endian
		while(size >= 4){
			val = ReadReg(targetAddr);
			fwrite(&val, 4, 1, fp);
			targetAddr += 4;
			size -= 4;
		}
		if(size){
			val = ReadReg(targetAddr);
			fwrite(&val, 1, size, fp);
		}
	}
	else{
		while(size >= 4){
			val = ReadReg(targetAddr);
			val1 = val >> 24;
			val1 |= val >> 8 & 0x0000ff00;
			val1 |= val << 8 & 0x00ff0000;
			val1 |= val << 24 & 0xff000000;
			fwrite(&val1, 4, 1, fp);
			targetAddr += 4;
			size -= 4;
		}
		if(size){
			val = ReadReg(targetAddr);
			val1 = val >> 24;
			val1 |= val >> 8 & 0x0000ff00;
			val1 |= val << 8 & 0x00ff0000;
			val1 |= val << 24 & 0xff000000;
			fwrite(&val1, 1, size, fp);
		}
	}
}

int FillSdramBurstFromFile(FILE *fp, int targetAddr, int bsBufStartAddr, int bsBufEndAddr, int size, int endian, int checkeos, int *streameos)
{

	uint8_t *pBuf = NULL;
	int nreaded = 0;

	pBuf = malloc(size);
	if(!pBuf)
		return 0 ;
	memset(pBuf, 0, size);

	*streameos = 0;
	nreaded = fread(pBuf, 1, size, fp);

	if(nreaded != size)
	{
		if(checkeos)
		{
			if(nreaded == 0)
			{
				*streameos = 1;
				return 1;
			}
		}
		else
		{
			fseek(fp, 0, SEEK_SET);
			if(!fread(pBuf+nreaded, 1, size-nreaded, fp))
				return 0;
		}			
	}

	if((targetAddr + size) > bsBufEndAddr)
	{
		int room = bsBufEndAddr - targetAddr;
		WriteSdramBurst(pBuf, targetAddr, room, endian);
		WriteSdramBurst(pBuf+room, bsBufStartAddr, size - room, endian);
	}
	else
		WriteSdramBurst(pBuf, targetAddr, size, endian);

	free(pBuf);

	return 1;
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


int FillSdram(BufInfo * pBufInfo, uint32_t targetAddr, phyaddr_t bsBufStartAddr, phyaddr_t bsBufEndAddr, uint32_t size, int endian, int checkeos, int *streameos)
{
	unsigned int val;
	uint8_t * pui;
	uint8_t uca[4];
	int bufPnt;
	int bufSize;
	uint32_t written;
	uint8_t * pBuf;

	if(checkeos == 1 && (pBufInfo->point >= pBufInfo->size))
	{
		*streameos = 1;
		return 1;
	}
	pBuf = pBufInfo->buf;
	bufPnt = pBufInfo->point;
	bufSize = pBufInfo->size;
	written = 0;
	if(endian){ // big endian
		while(written < size){
			while(written < size && bufPnt + 3 < bufSize){
				val = pBuf[bufPnt] << 24 | pBuf[bufPnt+1] << 16 |
					pBuf[bufPnt+2] << 8 | pBuf[bufPnt+3];
				WriteReg(targetAddr, val);
				targetAddr += 4;
				if( targetAddr == bsBufEndAddr )
					targetAddr = bsBufStartAddr;
				written += 4;
				bufPnt += 4;
			}
			if(bufPnt == bufSize){
				if(checkeos == 1)
					break;
				else
					bufPnt = 0;
			}
			if(written < size){
				uca[0] = pBuf[bufPnt++];
				if(bufPnt == bufSize) bufPnt = 0;
				uca[1] = pBuf[bufPnt++];
				if(bufPnt == bufSize) bufPnt = 0;
				uca[2] = pBuf[bufPnt++];
				if(bufPnt == bufSize) bufPnt = 0;
				uca[3] = pBuf[bufPnt++];
				if(bufPnt == bufSize) bufPnt = 0;
				val = uca[0] << 24 | uca[1] << 16 |
					uca[2] << 8 | uca[3];
				WriteReg(targetAddr, val);
				targetAddr += 4;
				if(targetAddr == bsBufEndAddr)
					targetAddr = bsBufStartAddr;
				written += 4;
			}
		}
	}
	else{
		while(written < size){
			if(bufPnt % 4 == 0){ // 4-byte aligned
				pui = (uint8_t *)(pBuf + bufPnt);
				while(written < size && bufPnt + 3 < bufSize){
					val = *pui++;
					WriteReg(targetAddr, val);
					targetAddr += 4;
					if(targetAddr == bsBufEndAddr)
						targetAddr = bsBufStartAddr;
					written += 4;
					bufPnt += 4;
				}
				if(bufPnt == bufSize){
					if(checkeos == 1)
						break;
					else
						bufPnt = 0;
				}
			}
			else{
				while(written < size && bufPnt + 3 < bufSize){
					val = pBuf[bufPnt+3] << 24 | pBuf[bufPnt+2] << 16 |
						pBuf[bufPnt+1] << 8 | pBuf[bufPnt];
					WriteReg(targetAddr, val);
					targetAddr += 4;
					if(targetAddr == bsBufEndAddr)
						targetAddr = bsBufStartAddr;
					written += 4;
					bufPnt += 4;
				}
				if(bufPnt == bufSize){
					if(checkeos == 1)
						break;
					else
						bufPnt = 0;
				}
			}
			if(written < size){
				uca[0] = pBuf[bufPnt++];
				if (bufPnt == bufSize) bufPnt = 0;
				uca[1] = pBuf[bufPnt++];
				if (bufPnt == bufSize) bufPnt = 0;
				uca[2] = pBuf[bufPnt++];
				if (bufPnt == bufSize) bufPnt = 0;
				uca[3] = pBuf[bufPnt++];
				if (bufPnt == bufSize) bufPnt = 0;
				val = uca[3] << 24 | uca[2] << 16 |
					uca[1] << 8 | uca[0];
				WriteReg(targetAddr, val);
				targetAddr += 4;
				if(targetAddr == bsBufEndAddr)
					targetAddr = bsBufStartAddr;
				written += 4;
			}
		}
	}
	pBufInfo->count = written;
	pBufInfo->point = bufPnt;
	return 1;
}
