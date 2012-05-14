//------------------------------------------------------------------------------
// File: VpuApi.c
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------

#include <string.h>
#include <unistd.h>
#include "VpuApiFunc.h"
#include "Falcon.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "VpuIoctl.h"
#include "IOFunc.h"

static void flushMem()
{
	unsigned char* ptr;
	ptr = (unsigned char*)malloc(64*1024*sizeof(unsigned char));
	memset(ptr, 0, 64*1024);
	free(ptr);
}
/*
static phyaddr_t rdPtrRegAddr[] = {
	BIT_RD_PTR_0,
	BIT_RD_PTR_1,
	BIT_RD_PTR_2,
	BIT_RD_PTR_3
};

static phyaddr_t wrPtrRegAddr[] = {
	BIT_WR_PTR_0,
	BIT_WR_PTR_1,
	BIT_WR_PTR_2,
	BIT_WR_PTR_3
};

static phyaddr_t disFlagRegAddr[] = {
	BIT_FRM_DIS_FLG_0,
	BIT_FRM_DIS_FLG_1,
	BIT_FRM_DIS_FLG_2,
	BIT_FRM_DIS_FLG_3
};
*/
static int vpuDownLoaded ;

volatile unsigned char * vpu_bit_base;
static int vpu_fd;

// If a frame is started, pendingInst is set to the proper instance.
static CodecInst * pendingInst;

CodecInst codecInstPool[MAX_NUM_INSTANCE];
volatile unsigned char* workBuffer;
volatile unsigned char* codeBuffer;
volatile unsigned char* paraBuffer;
volatile unsigned char* g_mem_base;
//vpu phy mem base addr
unsigned int vpu_mem_base;

extern volatile  unsigned char * g_frameBuffer;

int ReadReg(unsigned int ADDR)
{
	int ret = (*(volatile int*)((volatile)vpu_bit_base + ((ADDR)-BIT_BASE)));
#ifdef REG_LOG 
	fprintf(reg_log, "read  offset: %03x, data: %08x\n", ADDR - BIT_BASE, ret);
#endif
	return  ret;
}

void WriteReg(unsigned int ADDR, int DATA)
{
 	*(volatile int *)(vpu_bit_base + (ADDR)-BIT_BASE ) = (DATA);
#ifdef REG_LOG 
	if( ADDR != BIT_CODE_DOWN )
		fprintf(reg_log, "write offset: %03x, data: %08x\n", ADDR-BIT_BASE, DATA);
#endif
}

int VPU_IsBusy()
{
	return VpuReadReg(BIT_BUSY_FLAG) != 0;
}

RetCode VPU_Init(phyaddr_t workBuf)
{
	int i;
	uint32_t data;
	CodecInst * pCodecInst;
	UINT dataH, dataL;
	UI64 data64;
	int fd;

	fd = open("/dev/mem", O_RDWR);
	if(!fd)
	{
		printf("cannot open /dev/mem\n");
		return -1;
	}

	vpu_bit_base = (volatile unsigned char *)mmap(0, 1024*1024/*1 M*/, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BIT_BASE);

	close(fd);

  //open the vpu driver
  vpu_fd = open("/dev/vpu", O_RDWR);
  if(!vpu_fd)
  {
    printf("cannot open /dev/vpu\n");
    return -1;
  }

  if(ioctl(vpu_fd, IOCTL_GET_VPU_MEM_BASE, &vpu_mem_base) != 0)
  {
    printf("vpu ioctl error!\n");
    return -1;
  }
  //vpu_mem_base = 0x03400000;

	g_mem_base = (volatile unsigned char *)mmap(0, 80*1024*1024/*80 M*/, PROT_READ | PROT_WRITE, MAP_SHARED, vpu_fd, 0);
  if(!g_mem_base)
  {
    printf("  !!! vpu map !!!");
    return RETCODE_FAILURE;
  }
	codeBuffer = g_mem_base + (workBuf - ADDR_PHY_MEM_BASE);

	//codeBuffer = workBuf;
	workBuffer = codeBuffer + CODE_BUF_SIZE;
	//paraBuffer = workBuffer + WORK_BUF_SIZE + PARA_BUF2_SIZE;
	paraBuffer = workBuffer + WORK_BUF_SIZE;

	for (i = 0; i < sizeof(bit_code) / sizeof(bit_code[0]); i += 4) { 
	   data = (bit_code[i+2] << 16) | bit_code[i + 3];
	   *((unsigned int *)(codeBuffer + i * 2)) = data;
	   data = (bit_code[i] << 16) | bit_code[i+1];
	   *((unsigned int *)(codeBuffer + i * 2 + 4)) = data;
	}

	VpuWriteReg(BIT_WORK_BUF_ADDR, vpu_mem_base+workBuf+ CODE_BUF_SIZE);
//	VpuWriteReg(BIT_PARA_BUF_ADDR, workBuf+WORK_BUF_SIZE+PARA_BUF2_SIZE);
	VpuWriteReg(BIT_PARA_BUF_ADDR, vpu_mem_base+workBuf + CODE_BUF_SIZE + WORK_BUF_SIZE);
	VpuWriteReg(BIT_CODE_BUF_ADDR, vpu_mem_base+workBuf);

	pCodecInst = &codecInstPool[0];
	for( i = 0; i < MAX_NUM_INSTANCE; ++i, ++pCodecInst ) 
	{
		pCodecInst->instIndex = i;
		pCodecInst->inUse = 0;
	}

	if (VpuReadReg(BIT_CUR_PC) != 0)
		return RETCODE_SUCCESS;

	VpuWriteReg(BIT_CODE_RUN, 0);
	
	for( i = 0; i < 2048; ++i ) 
	{
	   data = bit_code[i];
	   VpuWriteReg(BIT_CODE_DOWN, (i << 16) | data);
	}

	data = STREAM_FULL_EMPTY_CHECK_DISABLE << 2;
	data |= STREAM_ENDIAN;
	data |= 1 << 3;
	VpuWriteReg(BIT_BIT_STREAM_CTRL, data);
	VpuWriteReg(BIT_FRAME_MEM_CTRL, (CBCR_INTERLEAVE<< 2) | IMAGE_ENDIAN);
	VpuWriteReg(BIT_INT_ENABLE, 0);

	if( vpuDownLoaded == 0 )
		vpuDownLoaded = 1;


	VpuWriteReg(BIT_BUSY_FLAG, 1);	
	VpuWriteReg(BIT_CODE_RUN, 1);
 	while (VpuReadReg(BIT_BUSY_FLAG))
        fprintf(stderr,"zl--bit_busy_flag");

	return RETCODE_SUCCESS;
}

RetCode VPU_GetVersionInfo( uint32_t *versionInfo )
{
	uint32_t ver;
	
	if (VpuReadReg(BIT_CUR_PC) == 0)
		return RETCODE_NOT_INITIALIZED;

	if( pendingInst )
		return RETCODE_FRAME_NOT_COMPLETE;
	
	VpuWriteReg( RET_VER_NUM , 0 );

	BitIssueCommand( 0, 0, FIRMWARE_GET );
 	while (VpuReadReg(BIT_BUSY_FLAG));
		
	ver = VpuReadReg( RET_VER_NUM );
	
	if( ver == 0 )
		return RETCODE_FAILURE;

	*versionInfo = ver;

	return RETCODE_SUCCESS;
}

RetCode VPU_EncOpen(EncHandle * pHandle, EncOpenParam * pop)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	int instIdx;
	RetCode ret;
	uint32_t  val;
	
	if (VpuReadReg(BIT_CUR_PC) == 0)
		return RETCODE_NOT_INITIALIZED;

	ret = CheckEncOpenParam(pop);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	ret = GetCodecInstance(&pCodecInst);
	if (ret == RETCODE_FAILURE) {
		*pHandle = 0;
		return RETCODE_FAILURE;
	}

	*pHandle = pCodecInst;
	instIdx = pCodecInst->instIndex;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	pEncInfo->openParam = *pop;

	if( pop->bitstreamFormat == STD_MPEG4 || pop->bitstreamFormat == STD_H263 )
		pCodecInst->codecMode = MP4_ENC;
	else if( pop->bitstreamFormat == STD_AVC )
		pCodecInst->codecMode = AVC_ENC;
	else if( pop->bitstreamFormat == STD_MJPG )
		pCodecInst->codecMode = MJPG_ENC;

	pEncInfo->streamRdPtr = pop->bitstreamBuffer;
	pEncInfo->streamWrPtr = pop->bitstreamBuffer;
	pEncInfo->streamRdPtrRegAddr = BIT_RD_PTR;
	pEncInfo->streamWrPtrRegAddr = BIT_WR_PTR;
	pEncInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pEncInfo->streamBufSize = pop->bitstreamBufferSize;
	pEncInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;
	pEncInfo->frameBufPool = 0;

#if 0  //FIXME: ybfqing for secAxi
	pEncInfo->secAxiUse.useBitEnable = 0;
	pEncInfo->secAxiUse.useIpEnable = 0;
	pEncInfo->secAxiUse.useDbkYEnable = 0;
	pEncInfo->secAxiUse.useDbkCEnable = 0;
	pEncInfo->secAxiUse.useOvlEnable = 0;
	pEncInfo->secAxiUse.useBtpEnable = 0;

	pEncInfo->secAxiUse.useHostBitEnable = 0;
	pEncInfo->secAxiUse.useHostIpEnable = 0;
	pEncInfo->secAxiUse.useHostDbkYEnable = 0;
	pEncInfo->secAxiUse.useHostDbkCEnable = 0;
	pEncInfo->secAxiUse.useHostOvlEnable = 0;
	pEncInfo->secAxiUse.useHostBtpEnable = 0;
	pEncInfo->secAxiUse.useHostMeEnable = 0;
#endif
	pEncInfo->rotationEnable = 0;
	pEncInfo->mirrorEnable = 0;
	pEncInfo->mirrorDirection = MIRDIR_NONE;
	pEncInfo->rotationAngle = 0;

	pEncInfo->initialInfoObtained = 0;
	pEncInfo->dynamicAllocEnable = pop->dynamicAllocEnable;
	pEncInfo->ringBufferEnable = pop->ringBufferEnable;
	
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(pEncInfo->streamWrPtrRegAddr, pEncInfo->streamBufStartAddr);

	
	val = VpuReadReg( BIT_BIT_STREAM_CTRL );
	val &= 0xFFC7;
	if( pEncInfo->ringBufferEnable == 0 ) {	
		val |= ( pEncInfo->dynamicAllocEnable << 5 );		
		val |= 1 << 4;	
	} else {
		val |= 1 << 3;
	}
	VpuWriteReg(BIT_BIT_STREAM_CTRL, val);
	pEncInfo->streamEndflag = val;

	return RETCODE_SUCCESS;
}

RetCode VPU_EncClose(EncHandle handle)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	if (pEncInfo->initialInfoObtained) {
		BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SEQ_END);
		while (VpuReadReg(BIT_BUSY_FLAG))
			;
	}

	FreeCodecInstance(pCodecInst);
	return RETCODE_SUCCESS;
}

RetCode VPU_EncGetInitialInfo(EncHandle handle, EncInitialInfo * info)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	int picWidth;
	int picHeight;
	uint32_t	data;
	RetCode ret;

	uint8_t * tableBuf = 0;
	uint8_t * tableBuf2 = 0;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pEncInfo->initialInfoObtained) {
		return RETCODE_CALLED_BEFORE;
	}

	picWidth = pEncInfo->openParam.picWidth;
	picHeight = pEncInfo->openParam.picHeight;

	VpuWriteReg(CMD_ENC_SEQ_BB_START, pEncInfo->streamBufStartAddr);
	VpuWriteReg(CMD_ENC_SEQ_BB_SIZE, pEncInfo->streamBufSize / 1024); // size in KB

	data = (picWidth << 16) | picHeight;
	VpuWriteReg(CMD_ENC_SEQ_SRC_SIZE, data);
	VpuWriteReg(CMD_ENC_SEQ_FRATE_DR, pEncInfo->openParam.frameRateDr);
	VpuWriteReg(CMD_ENC_SEQ_FRATE_NR, pEncInfo->openParam.frameRateNr);

	if (pEncInfo->openParam.bitstreamFormat == STD_MPEG4) {
		VpuWriteReg(CMD_ENC_SEQ_COD_STD, 0);
		data = pEncInfo->openParam.EncStdParam.mp4Param.mp4_intraDcVlcThr << 2 |
			pEncInfo->openParam.EncStdParam.mp4Param.mp4_reversibleVlcEnable << 1 |
			pEncInfo->openParam.EncStdParam.mp4Param.mp4_dataPartitionEnable;

		data |= ((pEncInfo->openParam.EncStdParam.mp4Param.mp4_hecEnable >0)? 1:0)<<5;
		data |= ((pEncInfo->openParam.EncStdParam.mp4Param.mp4_verid == 2)? 0:1) << 6;

		VpuWriteReg(CMD_ENC_SEQ_MP4_PARA, data);
	}
	else if (pEncInfo->openParam.bitstreamFormat == STD_H263) {
		VpuWriteReg(CMD_ENC_SEQ_COD_STD, 8);
		data = pEncInfo->openParam.EncStdParam.h263Param.h263_annexJEnable << 2 |
			pEncInfo->openParam.EncStdParam.h263Param.h263_annexKEnable << 1|
			pEncInfo->openParam.EncStdParam.h263Param.h263_annexTEnable;
		VpuWriteReg(CMD_ENC_SEQ_263_PARA, data);
	}
	else if (pEncInfo->openParam.bitstreamFormat == STD_AVC) {
		VpuWriteReg(CMD_ENC_SEQ_COD_STD, 2);
		data = (pEncInfo->openParam.EncStdParam.avcParam.avc_deblkFilterOffsetBeta & 15) << 12 |
			(pEncInfo->openParam.EncStdParam.avcParam.avc_deblkFilterOffsetAlpha & 15) << 8 |
			pEncInfo->openParam.EncStdParam.avcParam.avc_disableDeblk << 6 |
			pEncInfo->openParam.EncStdParam.avcParam.avc_constrainedIntraPredFlag << 5 |
			(pEncInfo->openParam.EncStdParam.avcParam.avc_chromaQpOffset & 31);
		VpuWriteReg(CMD_ENC_SEQ_264_PARA, data);
	}
	else if (pEncInfo->openParam.bitstreamFormat == STD_MJPG) {
		VpuWriteReg(CMD_ENC_SEQ_COD_STD, 6);  // ybfqing
		VpuWriteReg(CMD_ENC_SEQ_JPG_PARA, pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_sourceFormat);
		VpuWriteReg(CMD_ENC_SEQ_JPG_RST_INTERVAL, pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_restartInterval);
		VpuWriteReg(CMD_ENC_SEQ_JPG_THUMB_EN, pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_thumbNailEnable);
		data = (pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_thumbNailWidth) << 16 |
			(pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_thumbNailHeight) ;
		VpuWriteReg(CMD_ENC_SEQ_JPG_THUMB_SIZE, data);
		VpuWriteReg(CMD_ENC_SEQ_JPG_THUMB_OFFSET, 0);

		int i;
		unsigned int temp;
		tableBuf = pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_hufTable;
		for ( i = 0 ; i < 432 ; i += 4 ) {
			temp = (tableBuf[i] | (tableBuf[i+1] << 8) | (tableBuf[i+2] << 16) | (tableBuf[i+3] << 24)) ;
			*((unsigned int *)(paraBuffer + i)) = temp;
			//VpuWriteMem(paraBuffer + i, temp);
		}
    if(1){
      int i;
      for( i=0; i<108/* 432/4 */; i+=2 )
      {
         unsigned int tmp1 = *((unsigned int *)(paraBuffer + i*4)); 
         unsigned int tmp2 = *((unsigned int *)(paraBuffer + i*4+4)); 
         *((unsigned int *)(paraBuffer + i*4)) = tmp2;
         *((unsigned int *)(paraBuffer + i*4+4)) = tmp1;
      }
    }
		tableBuf = pEncInfo->openParam.EncStdParam.mjpgParam.mjpg_qMatTable;
		for ( i = 0 ; i < 192 ; i += 4 ) {
			temp = (tableBuf[i] | (tableBuf[i+1] << 8) | (tableBuf[i+2] << 16) | (tableBuf[i+3] << 24)) ;
			*((unsigned int *)(paraBuffer + 0x200 + i)) = temp;
			//VpuWriteMem(paraBuffer + 0x200 + i, temp);
		}
	}
    if(1){
      int i;
      for( i=0; i<48/* 192/4 */; i+=2)
      {
         unsigned int tmp1 = *((unsigned int *)(paraBuffer + 0x200 + i*4)); 
         unsigned int tmp2 = *((unsigned int *)(paraBuffer + 0x200 + i*4+4)); 
         *((unsigned int *)(paraBuffer + 0x200 + i*4)) = tmp2;
         *((unsigned int *)(paraBuffer + 0x200 + i*4+4)) = tmp1;
      }

    }

	if( pEncInfo->openParam.bitstreamFormat != STD_MJPG )
	{
		data = pEncInfo->openParam.slicemode.sliceSize << 2 |
			pEncInfo->openParam.slicemode.sliceSizeMode << 1 |
			pEncInfo->openParam.slicemode.sliceMode;
		VpuWriteReg(CMD_ENC_SEQ_SLICE_MODE, data);
		VpuWriteReg(CMD_ENC_SEQ_GOP_NUM, pEncInfo->openParam.gopSize);
	}

	if (pEncInfo->openParam.bitRate) { // rate control enabled
		data = (!pEncInfo->openParam.enableAutoSkip) << 31 |
			pEncInfo->openParam.initialDelay << 16 |
			pEncInfo->openParam.bitRate << 1 | 1;
		VpuWriteReg(CMD_ENC_SEQ_RC_PARA, data);
	}
	else {
		VpuWriteReg(CMD_ENC_SEQ_RC_PARA, 0);
	}
	VpuWriteReg(CMD_ENC_SEQ_RC_BUF_SIZE, pEncInfo->openParam.vbvBufferSize);
	VpuWriteReg(CMD_ENC_SEQ_INTRA_REFRESH, pEncInfo->openParam.intraRefresh);

	if(pEncInfo->openParam.rcIntraQp>=0)	
		data = (1 << 5);
	else
		data = 0;
	VpuWriteReg(CMD_ENC_SEQ_INTRA_QP, pEncInfo->openParam.rcIntraQp);

	if (pCodecInst->codecMode == AVC_ENC) {
		data |= (pEncInfo->openParam.EncStdParam.avcParam.avc_audEnable << 2);
		data |= ( pEncInfo->openParam.EncStdParam.avcParam.avc_fmoEnable << 4 );		
	}
	if(pEncInfo->openParam.userQpMax) {
		data |= (1<<6);
		VpuWriteReg(CMD_ENC_SEQ_RC_QP_MAX, pEncInfo->openParam.userQpMax);
	} 
	if(pEncInfo->openParam.userGamma) {
		data |= (1<<7);
		VpuWriteReg(CMD_ENC_SEQ_RC_GAMMA, pEncInfo->openParam.userGamma);
	}

	if(pEncInfo->openParam.avcIntra16x16OnlyModeEnable) {
		data |= (1<<8);
	}

	VpuWriteReg(CMD_ENC_SEQ_OPTION, data);

	VpuWriteReg(CMD_ENC_SEQ_RC_INTERVAL_MODE, (pEncInfo->openParam.MbInterval<<2) | pEncInfo->openParam.RcIntervalMode);

	if (pCodecInst->codecMode == AVC_ENC) {
		data = ( pEncInfo->openParam.EncStdParam.avcParam.avc_fmoType << 4 ) | ( pEncInfo->openParam.EncStdParam.avcParam.avc_fmoSliceNum & 0x0f);
		data |= ( pEncInfo->openParam.EncStdParam.avcParam.avc_fmoSliceSaveBufSize << 7 );
	}
	
	VpuWriteReg(pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(BIT_BIT_STREAM_PARAM, pEncInfo->streamEndflag);

	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SEQ_INIT);
	while (VpuReadReg(BIT_BUSY_FLAG))
		;

	if (VpuReadReg(RET_ENC_SEQ_SUCCESS) == 0) {
		return RETCODE_FAILURE;
	}
	if (pCodecInst->codecMode == MJPG_ENC)
		info->minFrameBufferCount = 0;
	else
		info->minFrameBufferCount = 2; // reconstructed frame + reference frame

	pEncInfo->initialInfo = *info;
	pEncInfo->initialInfoObtained = 1;

	pEncInfo->streamRdPtr = VpuReadReg(pEncInfo->streamRdPtrRegAddr);
	pEncInfo->streamWrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);
	pEncInfo->streamEndflag = VpuReadReg(BIT_BIT_STREAM_PARAM);

	return RETCODE_SUCCESS;
}


RetCode VPU_EncRegisterFrameBuffer(
		EncHandle handle,
		FrameBuffer * bufArray,
		int num,
		int stride)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	int i;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pEncInfo->frameBufPool) {
		return RETCODE_CALLED_BEFORE;
	}

	if (!pEncInfo->initialInfoObtained) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (bufArray == 0) {
		return RETCODE_INVALID_FRAME_BUFFER;
	}
	if (num < pEncInfo->initialInfo.minFrameBufferCount) {
		return RETCODE_INSUFFICIENT_FRAME_BUFFERS;
	}
	if (stride % 8 != 0 || stride == 0) {
		return RETCODE_INVALID_STRIDE;
	}

	pEncInfo->frameBufPool = bufArray;
	pEncInfo->numFrameBuffers = num;
	pEncInfo->stride = stride;

	// Let the codec know the addresses of the frame buffers.
	if (pCodecInst->codecMode != MJPG_ENC)
	{
		// Let the decoder know the addresses of the frame buffers.
		for (i = 0; i < num; ++i) {
			*(unsigned int *)(paraBuffer + i * 3 *4) = bufArray[i].bufY;
			*(unsigned int *)(paraBuffer + i * 3 *4 + 4) = bufArray[i].bufCb;
			*(unsigned int *)(paraBuffer + i * 3 *4 + 8) = bufArray[i].bufCr;
#if 0
			VpuWriteReg(paraBuffer + i * 3 * 4, bufArray[i].bufY);
			VpuWriteReg(paraBuffer + i * 3 * 4 + 4, bufArray[i].bufCb);
			VpuWriteReg(paraBuffer + i * 3 * 4 + 8, bufArray[i].bufCr);
#endif
		}
		int tempNum = num * 3 + 1;
		for(i = 0; i < tempNum; i+=2){
			unsigned int temp1, temp2;
			temp1 = *(unsigned int *)(paraBuffer + i * 4); 
			temp2 = *(unsigned int *)(paraBuffer + i * 4 + 4); 
			*(unsigned int *)(paraBuffer + i*4) = temp2;
			*(unsigned int *)(paraBuffer + i*4+4) = temp1;
		}	
	}
	// Tell the codec how much frame buffers were allocated.
	VpuWriteReg(CMD_SET_FRAME_BUF_NUM, num);
	VpuWriteReg(CMD_SET_FRAME_BUF_STRIDE, stride);

	VpuWriteReg( CMD_SET_FRAME_AXI_BIT_ADDR, BIT_INTERNAL_USE_BUF);
	VpuWriteReg( CMD_SET_FRAME_AXI_IPACDC_ADDR, IPACDC_INTERNAL_USE_BUF);
	VpuWriteReg( CMD_SET_FRAME_AXI_DBKY_ADDR, DBKY_INTERNAL_USE_BUF);
	VpuWriteReg( CMD_SET_FRAME_AXI_DBKC_ADDR, DBKC_INTERNAL_USE_BUF);
	VpuWriteReg( CMD_SET_FRAME_AXI_OVL_ADDR, OVL_INTERNAL_USE_BUF);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SET_FRAME_BUF);
	while (VpuReadReg(BIT_BUSY_FLAG))
		;
	
	return RETCODE_SUCCESS;
}

RetCode VPU_EncGetBitstreamBuffer( EncHandle handle,
		phyaddr_t * prdPrt,
		phyaddr_t * pwrPtr,
		uint32_t * size)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	phyaddr_t rdPtr;
	phyaddr_t wrPtr;
	uint32_t room;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if ( prdPrt == 0 || pwrPtr == 0 || size == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	rdPtr = pEncInfo->streamRdPtr;
	wrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);

	if( pEncInfo->ringBufferEnable == 1 ) {
		if ( wrPtr >= rdPtr ) {
			room = wrPtr - rdPtr;
		}
		else {
			room = ( pEncInfo->streamBufEndAddr - rdPtr ) + ( wrPtr - pEncInfo->streamBufStartAddr );
		}
	}
	else {
		if( rdPtr == pEncInfo->streamBufStartAddr && wrPtr >= rdPtr )
			room = wrPtr - rdPtr;	
		else
			return RETCODE_INVALID_PARAM;
	}

	*prdPrt = rdPtr;
	*pwrPtr = wrPtr;
	*size = room;

	return RETCODE_SUCCESS;
}

RetCode VPU_EncUpdateBitstreamBuffer(
		EncHandle handle,
		uint32_t size)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	phyaddr_t wrPtr;
	phyaddr_t rdPtr;
	RetCode ret;
	int		i = 0;
	int		room = 0;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	rdPtr = pEncInfo->streamRdPtr;

	wrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);
	if ( rdPtr < wrPtr ) {
		if ( rdPtr + size  > wrPtr )
			return RETCODE_INVALID_PARAM;
	}

	if( pEncInfo->ringBufferEnable == 1 ) {
		rdPtr += size;
		if (rdPtr > pEncInfo->streamBufEndAddr) 
		{
			room = rdPtr - pEncInfo->streamBufEndAddr;
			rdPtr = pEncInfo->streamBufStartAddr;
			rdPtr += room;
		}
		if (rdPtr == pEncInfo->streamBufEndAddr) {
			rdPtr = pEncInfo->streamBufStartAddr;
		}
	}
	else {
		rdPtr = pEncInfo->streamBufStartAddr;
	}

	pEncInfo->streamRdPtr = rdPtr;
	pEncInfo->streamWrPtr = wrPtr;
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, rdPtr);
	return RETCODE_SUCCESS;
}

RetCode VPU_EncStartOneFrame(
		EncHandle handle,
		EncParam * param )
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	FrameBuffer * pSrcFrame;
	uint32_t data = 0;
	uint32_t rotMirEnable;
	uint32_t rotMirMode;
	uint32_t val;
	RetCode ret;
	// When doing pre-rotation, mirroring is applied first and rotation later,
	// vice versa when doing post-rotation.
	// For consistency, pre-rotation is converted to post-rotation orientation.
	static uint32_t rotatorModeConversion[] = {
		0, 1, 2, 3, 4, 7, 6, 5,
		6, 5, 4, 7, 2, 3, 0, 1
	};

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pEncInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	ret = CheckEncParam(pCodecInst, param);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	pSrcFrame = param->sourceFrame;
	rotMirEnable = 0;
	rotMirMode = 0;
	if (pEncInfo->rotationEnable) {
		rotMirEnable = 0x10; // Enable rotator
		switch (pEncInfo->rotationAngle) {
			case 0:
				rotMirMode |= 0x0;
				break;

			case 90:
				rotMirMode |= 0x1;
				break;

			case 180:
				rotMirMode |= 0x2;
				break;

			case 270:
				rotMirMode |= 0x3;
				break;
		}
	}
	if (pEncInfo->mirrorEnable) {
		rotMirEnable = 0x10; // Enable rotator

		switch (pEncInfo->mirrorDirection) {
			case MIRDIR_NONE :
				rotMirMode |= 0x0;
				break;

			case MIRDIR_VER :
				rotMirMode |= 0x4;
				break;

			case MIRDIR_HOR :
				rotMirMode |= 0x8;
				break;

			case MIRDIR_HOR_VER :
				rotMirMode |= 0xc;
				break;

		}

	}

	rotMirMode = rotatorModeConversion[rotMirMode];
	rotMirMode |= rotMirEnable;
	VpuWriteReg(CMD_ENC_PIC_ROT_MODE, rotMirMode);

	VpuWriteReg(CMD_ENC_PIC_QS, param->quantParam);

	if (param->skipPicture) {
#ifdef API_CR
		VpuWriteReg(CMD_ENC_PIC_OPTION,
			(param->enReportSliceInfo<<5) | (param->enReportMVInfo<<4) |(param->enReportMBInfo<<3) | 1);
#else
 		VpuWriteReg(CMD_ENC_PIC_OPTION, 1);
#endif
	}
	else {
		int val = 
#ifdef JPEG_PARTIAL
			((pEncInfo->jpgPartialEncode&1)<<6) |
#endif // JPEG_PARTIAL
			((param->enReportSliceInfo	&1)<<5) | 
			((param->enReportMVInfo		&1)<<4) |
			((param->enReportMBInfo		&1)<<3) |
			((param->forceIPicture		&1)<<1);

		VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_Y, pSrcFrame->bufY);
		VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_CB, pSrcFrame->bufCb);
		VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_CR, pSrcFrame->bufCr);

#ifdef API_CR
		VpuWriteReg(CMD_ENC_PIC_OPTION, val);
#else
		VpuWriteReg(CMD_ENC_PIC_OPTION, (param->forceIPicture << 1 & 0x2) );
#endif
	}

	if( pEncInfo->dynamicAllocEnable == 1 ) {
		VpuWriteReg( CMD_ENC_PIC_BB_START, param->picStreamBufferAddr );
		VpuWriteReg( CMD_ENC_PIC_BB_SIZE, param->picStreamBufferSize / 1024 ); // size in KB
	}

#ifdef API_CR
	if(param->enReportMBInfo || param->enReportMVInfo || param->enReportSliceInfo) {
		VpuWriteReg(CMD_ENC_PIC_PARA_BASE_ADDR, param->picParaBaseAddr);
		
		if(param->enReportMBInfo)
			Vpu_EncSetHostParaAddr(param->picParaBaseAddr 		, param->picMbInfoAddr);
		if(param->enReportMVInfo)
			Vpu_EncSetHostParaAddr(param->picParaBaseAddr + 8	, param->picMvInfoAddr);
		if(param->enReportSliceInfo)
			Vpu_EncSetHostParaAddr(param->picParaBaseAddr + 16	, param->picSliceInfoAddr);
	}	
#endif
#ifdef JPEG_PARTIAL
	VpuWriteReg( CMD_ENC_PIC_JPG_PART_MB_SIZE, param->jpgPartialMbNum );
#endif // JPEG_PARTIAL
	
	val = ( pEncInfo->secAxiUse.useBitEnable | pEncInfo->secAxiUse.useIpEnable << 1 | pEncInfo->secAxiUse.useDbkYEnable << 2 
		| pEncInfo->secAxiUse.useDbkCEnable << 3 | pEncInfo->secAxiUse.useOvlEnable << 4 | pEncInfo->secAxiUse.useMeEnable << 5 
		| pEncInfo->secAxiUse.useHostBitEnable << 7 | pEncInfo->secAxiUse.useHostIpEnable << 8 | pEncInfo->secAxiUse.useHostDbkYEnable << 9 
		| pEncInfo->secAxiUse.useHostDbkCEnable << 10 | pEncInfo->secAxiUse.useHostOvlEnable << 11 | pEncInfo->secAxiUse.useHostMeEnable << 12);
	
	VpuWriteReg( BIT_AXI_SRAM_USE, val);
	VpuWriteReg(pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(BIT_BIT_STREAM_PARAM, pEncInfo->streamEndflag);
	VpuWriteReg(BIT_FRAME_MEM_CTRL, (CBCR_INTERLEAVE<< 2) | IMAGE_ENDIAN);		// BIT_FRAME_MEM_CTRL(CbCrInterleave) should be set every PIC_RUN command.
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, PIC_RUN);

	pendingInst = pCodecInst;

	return RETCODE_SUCCESS;
}

RetCode VPU_EncGetOutputInfo(
		EncHandle handle,
		EncOutputInfo * info
		)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;
	phyaddr_t rdPtr;
	phyaddr_t wrPtr;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pendingInst == 0) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pCodecInst != pendingInst) {
		return RETCODE_INVALID_HANDLE;
	}

	info->picType = VpuReadReg(RET_ENC_PIC_TYPE);

	if( pEncInfo->ringBufferEnable == 0 ) {		
		if( pEncInfo->dynamicAllocEnable == 1 ) {
			rdPtr = VpuReadReg( CMD_ENC_PIC_BB_START );
			wrPtr = VpuReadReg( pEncInfo->streamWrPtrRegAddr);
			info->bitstreamBuffer = rdPtr;
			info->bitstreamSize = wrPtr - rdPtr;
		}
		else {
			rdPtr = pEncInfo->streamBufStartAddr;
			wrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);
			info->bitstreamBuffer = rdPtr;
			info->bitstreamSize = wrPtr - rdPtr;
		}	
	}

	info->numOfSlices = VpuReadReg(RET_ENC_PIC_SLICE_NUM);
	info->bitstreamWrapAround = VpuReadReg(RET_ENC_PIC_FLAG);

#ifdef API_CR
	if(pEncInfo->enReportMBInfo) {
		int val;
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);

		ReadSdramBurst(tempBuf, pEncInfo->picParaBaseAddr, 8, 1);

		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		info->MbInfo.addr =	((tempBuf[4]<<24) & 0xFF000000) |
							((tempBuf[5]<<16) & 0x00FF0000) |
							((tempBuf[6]<< 8) & 0x0000FF00) |
							((tempBuf[7]<< 0) & 0x000000FF);

		info->MbInfo.sz = val & 0xFFFF;
		info->MbInfo.enable = (val >> 24) & 0xFF;
	}

	if(pEncInfo->enReportMVInfo) {
		int val;
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);

		ReadSdramBurst(tempBuf, pEncInfo->picParaBaseAddr+8, 8, 1);

		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		info->MvInfo.addr =	((tempBuf[4]<<24) & 0xFF000000) |
							((tempBuf[5]<<16) & 0x00FF0000) |
							((tempBuf[6]<< 8) & 0x0000FF00) |
							((tempBuf[7]<< 0) & 0x000000FF);

		info->MvInfo.sz = val & 0xFFFF;
		info->MvInfo.enable = (val >> 24) & 0xFF;
		info->MvInfo.type = (val >> 16) & 0xFF;				
	}
	if(pEncInfo->enReportSliceInfo) {
		int val;
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);

		ReadSdramBurst(tempBuf, pEncInfo->picParaBaseAddr+16, 8, 1);

		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		info->SliceInfo.addr =	((tempBuf[4]<<24) & 0xFF000000) |
								((tempBuf[5]<<16) & 0x00FF0000) |
								((tempBuf[6]<< 8) & 0x0000FF00) |
								((tempBuf[7]<< 0) & 0x000000FF);

		info->SliceInfo.sz = val & 0xFFFF;
		info->SliceInfo.enable = (val >> 24) & 0xFF;
		info->SliceInfo.type = (val >> 16) & 0xFF;
	}
#endif
	
	pendingInst = 0;

	pEncInfo->streamWrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);
	pEncInfo->streamRdPtr = VpuReadReg(pEncInfo->streamRdPtrRegAddr);

	return RETCODE_SUCCESS;
}


RetCode VPU_EncGiveCommand(
		EncHandle handle,
		CodecCommand cmd,
		void * param)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	switch (cmd) {
#ifdef JPEG_PARTIAL
		case ENABLE_JPG_PARTIAL :
			{
				pEncInfo->jpgPartialEncode = 1;
				break;
			}

		case DISABLE_JPG_PARTIAL :
			{
				pEncInfo->jpgPartialEncode = 0;
				break;
			}
#endif // JPEG_PARTIAL

		case ENABLE_ROTATION :
			{
				pEncInfo->rotationEnable = 1;
				break;
			}

		case DISABLE_ROTATION :
			{
				pEncInfo->rotationEnable = 0;
				break;
			}

		case ENABLE_MIRRORING :
			{
				pEncInfo->mirrorEnable = 1;
				break;
			}

		case DISABLE_MIRRORING :
			{
				pEncInfo->mirrorEnable = 0;
				break;
			}

		case SET_MIRROR_DIRECTION :
			{

				MirrorDirection mirDir;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				mirDir = *(MirrorDirection *)param;
				if (!(MIRDIR_NONE <= mirDir && mirDir <= MIRDIR_HOR_VER)) {
					return RETCODE_INVALID_PARAM;
				}
				pEncInfo->mirrorDirection = mirDir;

				break;
			}

		case SET_ROTATION_ANGLE :
			{
				int angle;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				angle = *(int *)param;
				if (angle != 0 && angle != 90 &&
						angle != 180 && angle != 270) {
					return RETCODE_INVALID_PARAM;
				}
				if (pEncInfo->initialInfoObtained && (angle == 90 || angle ==270)) {
					return RETCODE_INVALID_PARAM;
				}
				pEncInfo->rotationAngle = angle;
				break;
			}
		case ENC_GET_SPS_RBSP:
			{
				if (pCodecInst->codecMode != AVC_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				GetParaSet(handle, 0, param);
				break;
			}

		case ENC_GET_PPS_RBSP:
			{
				if (pCodecInst->codecMode != AVC_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				GetParaSet(handle, 1, param);
				break;
			}

		case ENC_PUT_MP4_HEADER:
			{
				EncHeaderParam * encHeaderParam;

				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}				
				encHeaderParam = (EncHeaderParam *)param;
				if (!( VOL_HEADER<=encHeaderParam->headerType && encHeaderParam->headerType <= VIS_HEADER)) {
					return RETCODE_INVALID_PARAM;
				}
				EncodeHeader(handle, encHeaderParam);
				break;
			}

		case ENC_PUT_AVC_HEADER:					
			{
				EncHeaderParam * encHeaderParam;
				
				if (pCodecInst->codecMode != AVC_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}				
				encHeaderParam = (EncHeaderParam *)param;
				if (!( SPS_RBSP<=encHeaderParam->headerType && encHeaderParam->headerType <= PPS_RBSP)) {
					return RETCODE_INVALID_PARAM;
				}
				EncodeHeader(handle, encHeaderParam);
				break;
			}
		case ENC_SET_SEARCHRAM_PARAM:
			{
				SearchRamParam *scRamParam = 0;
				int EncPicX;
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}

				EncPicX = pCodecInst->CodecInfo.encInfo.openParam.picWidth;

				scRamParam = (SearchRamParam *)param;
				if( scRamParam->SearchRamSize != ( ( ( ( EncPicX + 15 ) & ~15 ) * 36 ) + 2048 ) ){
					return RETCODE_INVALID_PARAM;
				}
				VpuWriteReg( CMD_ENC_SEARCH_BASE, scRamParam->searchRamAddr );
				VpuWriteReg( CMD_ENC_SEARCH_SIZE, scRamParam->SearchRamSize  );

				break;
			}
		case ENC_GET_VOS_HEADER:
			{
				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				GetParaSet(handle, 1, param); 
				break;
			}
		case ENC_GET_VO_HEADER:
			{
				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				GetParaSet(handle, 2, param); 
				break;
			}
		case ENC_GET_VOL_HEADER:
			{
				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				GetParaSet(handle, 0, param); 
				break;
			}
		case ENC_SET_GOP_NUMBER:
			{
				int *pGopNumber =(int *)param;
				if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
					return RETCODE_INVALID_COMMAND;
				}
				if (*pGopNumber < 0 ||  *pGopNumber > 60) {
					return RETCODE_INVALID_PARAM;
				}
				SetGopNumber(handle, (uint32_t *)pGopNumber); 
				break;
			}
		case ENC_SET_INTRA_QP:
			{
				int *pIntraQp =(int *)param;
				if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
					return RETCODE_INVALID_COMMAND;
				}
				if (pCodecInst->codecMode == MP4_ENC)
				{	
					if(*pIntraQp<1 || *pIntraQp>31)
						return RETCODE_INVALID_PARAM;
				}
				if (pCodecInst->codecMode == AVC_ENC)
				{	
					if(*pIntraQp<0 || *pIntraQp>51)
						return RETCODE_INVALID_PARAM;
				}
				SetIntraQp(handle, (uint32_t *)pIntraQp); 
				break;
			}
		case ENC_SET_BITRATE:
			{
				int *pBitrate = (int *)param;
				if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
					return RETCODE_INVALID_COMMAND;
				}
				if (*pBitrate < 0 || *pBitrate> 32767) {
					return RETCODE_INVALID_PARAM;
				}
				SetBitrate(handle, (uint32_t *)pBitrate); 
				break;
			}
		case ENC_SET_FRAME_RATE:
			{
				int *pFramerate = (int *)param;

				if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
					return RETCODE_INVALID_COMMAND;
				}
				if (*pFramerate <= 0) {
					return RETCODE_INVALID_PARAM;
				}
				SetFramerate(handle, (uint32_t *)pFramerate); 
				break;
			}
		case ENC_SET_INTRA_MB_REFRESH_NUMBER:
			{
				int *pIntraRefreshNum =(int *)param;
				SetIntraRefreshNum(handle, (uint32_t *)pIntraRefreshNum); 
			}
			break;
		case ENC_SET_SLICE_INFO:
			{
				EncSliceMode *pSliceMode = (EncSliceMode *)param;
				if(pSliceMode->sliceMode<0 || pSliceMode->sliceMode>1)
				{
					return RETCODE_INVALID_PARAM;
				}
				if(pSliceMode->sliceSizeMode<0 || pSliceMode->sliceSizeMode>1)
				{
					return RETCODE_INVALID_PARAM;
				}
				SetSliceMode(handle, (EncSliceMode *)pSliceMode);
			}
			break;
		case ENC_ENABLE_HEC:
			{
				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				SetHecMode(handle, 1);
			}
			break;
		case ENC_DISABLE_HEC:
			{
				if (pCodecInst->codecMode != MP4_ENC) {
					return RETCODE_INVALID_COMMAND;
				}
				SetHecMode(handle, 0);
			}
			break;
		case ENC_SET_REPORT_MBINFO: {
			EncParam *pEncParam = (EncParam*)param;
			pEncInfo->enReportMBInfo = pEncParam->enReportMBInfo;
			pEncInfo->picMbInfoAddr  = pEncParam->picMbInfoAddr;
			}
			break;
		case ENC_SET_REPORT_MVINFO: {
			EncParam *pEncParam = (EncParam*)param;
			pEncInfo->enReportMVInfo = pEncParam->enReportMVInfo;
			pEncInfo->picMvInfoAddr  = pEncParam->picMvInfoAddr;
			}
			break;
		case ENC_SET_REPORT_SLICEINFO: {
			EncParam *pEncParam = (EncParam*)param;
			pEncInfo->enReportSliceInfo = pEncParam->enReportSliceInfo;
			pEncInfo->picSliceInfoAddr  = pEncParam->picSliceInfoAddr;
			}
			break;
		case ENC_SET_PIC_PARA_ADDR: {
			EncParam *pEncParam = (EncParam*)param;
			pEncInfo->picParaBaseAddr   = pEncParam->picParaBaseAddr;
			}
			break;

		case SET_SEC_AXI:{
				SecAxiUse secAxiUse;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				secAxiUse = *(SecAxiUse *)param;

				pEncInfo->secAxiUse.useBitEnable = (secAxiUse.useBitEnable & 0x1);
				pEncInfo->secAxiUse.useIpEnable = (secAxiUse.useIpEnable & 0x1);
				pEncInfo->secAxiUse.useDbkYEnable = (secAxiUse.useDbkYEnable & 0x1);
				pEncInfo->secAxiUse.useDbkCEnable = (secAxiUse.useDbkCEnable & 0x1);
				pEncInfo->secAxiUse.useOvlEnable = (secAxiUse.useOvlEnable & 0x1);
				pEncInfo->secAxiUse.useBtpEnable = (secAxiUse.useBtpEnable & 0x1);
				pEncInfo->secAxiUse.useMeEnable = (secAxiUse.useMeEnable & 0x1);
				pEncInfo->secAxiUse.useHostBitEnable = (secAxiUse.useHostBitEnable & 0x1);
				pEncInfo->secAxiUse.useHostIpEnable = (secAxiUse.useHostIpEnable & 0x1);
				pEncInfo->secAxiUse.useHostDbkYEnable = (secAxiUse.useHostDbkYEnable & 0x1);
				pEncInfo->secAxiUse.useHostDbkCEnable = (secAxiUse.useHostDbkCEnable & 0x1);
				pEncInfo->secAxiUse.useHostOvlEnable = (secAxiUse.useHostOvlEnable & 0x1);
				pEncInfo->secAxiUse.useHostBtpEnable = (secAxiUse.useHostBtpEnable & 0x1);
				pEncInfo->secAxiUse.useHostMeEnable = (secAxiUse.useHostMeEnable & 0x1);


				pEncInfo->secAxiUse.bufBitUse = secAxiUse.bufBitUse;
				pEncInfo->secAxiUse.bufIpAcDcUse = secAxiUse.bufIpAcDcUse;
				pEncInfo->secAxiUse.bufDbkYUse = secAxiUse.bufDbkYUse;
				pEncInfo->secAxiUse.bufDbkCUse = secAxiUse.bufDbkCUse;
				pEncInfo->secAxiUse.bufOvlUse = secAxiUse.bufOvlUse;

				break;
			}



		default:
			return RETCODE_INVALID_COMMAND;
	}
	return RETCODE_SUCCESS;
}


RetCode VPU_DecOpen(DecHandle * pHandle, DecOpenParam * pop)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	int instIdx, value;
	RetCode ret;

	if (VpuReadReg(BIT_CUR_PC) == 0){
		return RETCODE_NOT_INITIALIZED;
	}

	ret = CheckDecOpenParam(pop);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	ret = GetCodecInstance(&pCodecInst);
	if (ret == RETCODE_FAILURE) {
		*pHandle = 0;
		return RETCODE_FAILURE;
	}

	*pHandle = pCodecInst;
	instIdx = pCodecInst->instIndex;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	pDecInfo->openParam = *pop;

	if (pop->bitstreamFormat == STD_MPEG4) {
		pCodecInst->codecMode = MP4_DEC;
	}
	else if (pop->bitstreamFormat == STD_AVC) {
		pCodecInst->codecMode = AVC_DEC;
	} 
	else if (pop->bitstreamFormat == STD_VC1) {
		pCodecInst->codecMode = VC1_DEC;
	}
	else if (pop->bitstreamFormat == STD_MPEG2) {
		pCodecInst->codecMode = MP2_DEC;
	}
	else if (pop->bitstreamFormat == STD_DIV3) {
		pCodecInst->codecMode = DV3_DEC;
	}	
	else if (pop->bitstreamFormat == STD_RV) {
		pCodecInst->codecMode = RV_DEC;
	}
    else if (pop->bitstreamFormat == STD_AVS) {
        pCodecInst->codecMode = AVS_DEC;
	}
	else if (pop->bitstreamFormat == STD_MJPG) {
		pCodecInst->codecMode = MJPG_DEC;
	} 

	if (pop->bitstreamFormat == STD_DIV3)
		pDecInfo->picSrcSize = (pop->picWidth << 16) | pop->picHeight;

	pDecInfo->streamWrPtr = pop->bitstreamBuffer;
	pDecInfo->streamRdPtr = pop->bitstreamBuffer;
	pDecInfo->streamRdPtrRegAddr = BIT_RD_PTR;
	pDecInfo->streamWrPtrRegAddr = BIT_WR_PTR;
	pDecInfo->frameDisplayFlagRegAddr = BIT_FRM_DIS_FLG;
	pDecInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pDecInfo->streamBufSize = pop->bitstreamBufferSize;
	pDecInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;
	pDecInfo->frameBufPool = 0;

	pDecInfo->rotationEnable = 0;
	pDecInfo->mirrorEnable = 0;
	pDecInfo->mirrorDirection = MIRDIR_NONE;
	pDecInfo->rotationAngle = 0;
	pDecInfo->rotatorOutputValid = 0;
	pDecInfo->rotatorStride = 0;
	pDecInfo->deringEnable	= 0;
	pDecInfo->secAxiUse.useBitEnable = 0;
	pDecInfo->secAxiUse.useIpEnable = 0;
	pDecInfo->secAxiUse.useDbkYEnable = 0;
	pDecInfo->secAxiUse.useDbkCEnable = 0;
	pDecInfo->secAxiUse.useOvlEnable = 0;
	pDecInfo->secAxiUse.useBtpEnable = 0;

	pDecInfo->secAxiUse.useHostBitEnable = 0;
	pDecInfo->secAxiUse.useHostIpEnable = 0;
	pDecInfo->secAxiUse.useHostDbkYEnable = 0;
	pDecInfo->secAxiUse.useHostDbkCEnable = 0;
	pDecInfo->secAxiUse.useHostOvlEnable = 0;
	pDecInfo->secAxiUse.useHostBtpEnable = 0;
//	pDecInfo->secAxiUse.useHostBitEnable = 0;

	pDecInfo->secAxiUse.bufBitUse = 0;
	pDecInfo->secAxiUse.bufIpAcDcUse = 0;
	pDecInfo->secAxiUse.bufDbkYUse = 0;
	pDecInfo->secAxiUse.bufDbkCUse = 0;
	pDecInfo->secAxiUse.bufOvlUse = 0;

	pDecInfo->filePlayEnable = pop->filePlayEnable;
	if( pop->filePlayEnable == 1 )
	{
		pDecInfo->picSrcSize = (pop->picWidth << 16) | pop->picHeight;
		pDecInfo->dynamicAllocEnable = pop->dynamicAllocEnable;
	}

	pDecInfo->initialInfoObtained = 0;
	pDecInfo->vc1BframeDisplayValid = 0;

#ifdef JPEG_PARTIAL
	pDecInfo->jpgPartialDecode = 0;
#endif // JPEG_PARTIAL
	pDecInfo->picParaBaseAddr = 0;
	pDecInfo->userDataBufAddr = 0;
	pDecInfo->frameBufStatEnable = 0;
	pDecInfo->mbParamEnable = 0;
	pDecInfo->mvReportEnable = 0;
	pDecInfo->userDataEnable = 0;
	pDecInfo->userDataBufSize = 0;

	VpuWriteReg(pDecInfo->streamRdPtrRegAddr, pDecInfo->streamBufStartAddr);
	VpuWriteReg(pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pDecInfo->frameDisplayFlagRegAddr, 0);
	pDecInfo->frameDisplayFlag = 0;

	value = VpuReadReg( BIT_BIT_STREAM_PARAM );
	if (value & (1 << ( 2)))
		value -= 1 << ( 2);
	VpuWriteReg(BIT_BIT_STREAM_PARAM, value);
	
	pDecInfo->streamEndflag = value;
	
	return RETCODE_SUCCESS;
}

RetCode VPU_DecClose(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->initialInfoObtained) {
		if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
			VpuWriteReg(BIT_RUN_AUX_STD, 1);
		else
			VpuWriteReg(BIT_RUN_AUX_STD, 0);
		BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SEQ_END);
		while (VpuReadReg(BIT_BUSY_FLAG))
			;
	}
	FreeCodecInstance(pCodecInst);
  if(vpu_fd)
    close(vpu_fd);
	return RETCODE_SUCCESS;
}

RetCode VPU_DecSetEscSeqInit( DecHandle handle, int escape )
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	int val;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	
	val = VpuReadReg(CMD_DEC_SEQ_INIT_ESCAPE);

	val = val | (escape & 0x01);
	VpuWriteReg(CMD_DEC_SEQ_INIT_ESCAPE, val );	

	return RETCODE_SUCCESS;
}


RetCode VPU_DecGetInitialInfo(
		DecHandle handle,
		DecInitialInfo * info)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	uint32_t val, val2;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
    fprintf(stderr,"zl--%s,%s,%d\n",__func__,__FILE__,__LINE__);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}
	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->initialInfoObtained) {
		return RETCODE_CALLED_BEFORE;
	}


	if (DecBitstreamBufEmpty(pDecInfo)) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	VpuWriteReg(CMD_DEC_SEQ_BB_START, pDecInfo->streamBufStartAddr);
	VpuWriteReg(CMD_DEC_SEQ_BB_SIZE, pDecInfo->streamBufSize / 1024); // size in KBytes

	if( pDecInfo->filePlayEnable == 1 )
		VpuWriteReg(CMD_DEC_SEQ_START_BYTE, pDecInfo->openParam.streamStartByteOffset);

	val  = 0;
    fprintf(stderr,"zl--%s,%s,%d\n",__func__,__FILE__,__LINE__);
#ifdef API_CR
	if(pCodecInst->codecMode == MJPG_DEC) {
		val |= (pDecInfo->userDataReportMode		<<10 );
		val |= (pDecInfo->userDataEnable			<< 5 );
	}
#endif
	val |= (pDecInfo->dynamicAllocEnable << 3) & 0x8;
	val |= (pDecInfo->filePlayEnable << 2) & 0x4;
	val |= (pDecInfo->openParam.reorderEnable << 1) & 0x2;
	val |= (pDecInfo->openParam.mp4DeblkEnable & 0x1);	
	VpuWriteReg(CMD_DEC_SEQ_OPTION, val);					

	if( pCodecInst->codecMode == VC1_DEC ) {
		VpuWriteReg(CMD_DEC_SEQ_VC1_STREAM_FMT, 0);
	}
	if( pCodecInst->codecMode == MP4_DEC ) {
		VpuWriteReg(CMD_DEC_SEQ_MP4_ASP_CLASS, pDecInfo->openParam.mp4Class);
	}

	if( pCodecInst->codecMode == AVC_DEC ) {
		VpuWriteReg( CMD_DEC_SEQ_PS_BB_START, pDecInfo->openParam.psSaveBuffer );
		VpuWriteReg( CMD_DEC_SEQ_PS_BB_SIZE, (pDecInfo->openParam.psSaveBufferSize / 1024) );
	}
	if( pCodecInst->codecMode == MJPG_DEC ) {
		VpuWriteReg( CMD_DEC_SEQ_JPG_THUMB_EN, pDecInfo->openParam.mjpg_thumbNailDecEnable );
	}

	VpuWriteReg( CMD_DEC_SEQ_SRC_SIZE, pDecInfo->picSrcSize );

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);
		
	VpuWriteReg(pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pDecInfo->streamRdPtrRegAddr, pDecInfo->streamRdPtr);
	val = VpuReadReg(BIT_BIT_STREAM_PARAM);
	val |= pDecInfo->streamEndflag;
	VpuWriteReg(BIT_BIT_STREAM_PARAM, val);

#ifdef _CDB
	while ( VPU_IsBusy() )
        fprintf(stderr,"zl--%s,%s,%d\n",__func__,__FILE__,__LINE__);
#endif	// _CDB
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SEQ_INIT);
	while (VpuReadReg(BIT_BUSY_FLAG))
        fprintf(stderr,"zl--%s,%s,%d\n",__func__,__FILE__,__LINE__);

	if (VpuReadReg(RET_DEC_SEQ_SUCCESS) == 0)
		return RETCODE_FAILURE;
	
	val = VpuReadReg(RET_DEC_SEQ_SRC_SIZE);
	info->picWidth = ( (val >> 16) & 0xffff );
	info->picHeight = ( val & 0xffff );

	info->fRateNumerator	= VpuReadReg(RET_DEC_SEQ_FRATE_NR);
	info->fRateDenominator	= VpuReadReg(RET_DEC_SEQ_FRATE_DR);

	if (pCodecInst->codecMode  == MP4_DEC) 
	{
		val = VpuReadReg(RET_DEC_SEQ_INFO);
		info->mp4_shortVideoHeader = (val >> 2) & 1;
		info->mp4_dataPartitionEnable = val & 1;
		info->mp4_reversibleVlcEnable =
			info->mp4_dataPartitionEnable ?
			((val >> 1) & 1) : 0;
		info->h263_annexJEnable = (val >> 3) & 1;
	}

	info->minFrameBufferCount = (VpuReadReg(RET_DEC_SEQ_FRAME_NEED)&0xff) + 2;
	info->frameBufDelay = VpuReadReg(RET_DEC_SEQ_FRAME_DELAY);

	if (pCodecInst->codecMode == AVC_DEC)
	{
		val = VpuReadReg(RET_DEC_SEQ_CROP_LEFT_RIGHT);	
		val2 = VpuReadReg(RET_DEC_SEQ_CROP_TOP_BOTTOM);
		if( val == 0 && val2 == 0 )
		{
			info->picCropRect.left = 0;
			info->picCropRect.right = 0;
			info->picCropRect.top = 0;
			info->picCropRect.bottom = 0;
		}
		else
		{
			info->picCropRect.left = ( (val>>16) & 0xFFFF );
			info->picCropRect.right = info->picWidth - ( ( val & 0xFFFF ) );
			info->picCropRect.top = ( (val2>>16) & 0xFFFF );
			info->picCropRect.bottom = info->picHeight - ( ( val2 & 0xFFFF ) );
		}
		
		val = (info->picWidth * info->picHeight * 3 / 2) / 1024;
		info->normalSliceSize = val / 4;
		info->worstSliceSize = val / 2;
	}
	else
	{
		info->picCropRect.left = 0;
		info->picCropRect.right = 0;
		info->picCropRect.top = 0;
		info->picCropRect.bottom = 0;
	}
	if (pCodecInst->codecMode == MJPG_DEC)
	{
		info->mjpg_thumbNailEnable = ( VpuReadReg(RET_DEC_SEQ_JPG_THUMB_IND) & 0x01 );
		info->mjpg_sourceFormat = ( VpuReadReg(RET_DEC_SEQ_JPG_PARA) & 0x07 );
		if (pDecInfo->openParam.mjpg_thumbNailDecEnable == 1)
			if (info->mjpg_thumbNailEnable == 0)
					return RETCODE_FAILURE;
	}

	pDecInfo->initialInfo = *info;
	pDecInfo->initialInfoObtained = 1;

#ifdef API_CR
	val = VpuReadReg(RET_DEC_SEQ_HEADER_REPORT);
	info->profile =			(val >> 0) & 0xFF;
	info->level =			(val >> 8) & 0xFF;
	info->interlace =		(val >> 16) & 0x01;
	info->direct8x8Flag =		(val >> 17) & 0x01;
	info->vc1_psf =			(val >> 18) & 0x01;
	info->constraint_set_flag[0] = 	(val >> 19) & 0x01;
	info->constraint_set_flag[1] = 	(val >> 20) & 0x01;
	info->constraint_set_flag[2] = 	(val >> 21) & 0x01;
	info->constraint_set_flag[3] = 	(val >> 22) & 0x01;

	val = VpuReadReg(RET_DEC_SEQ_ASPECT);

	info->aspectRateInfo = val;
#endif

	pDecInfo->streamRdPtr = VpuReadReg(pDecInfo->streamRdPtrRegAddr);
	pDecInfo->streamWrPtr = VpuReadReg(pDecInfo->streamWrPtrRegAddr);
	pDecInfo->streamEndflag = VpuReadReg(BIT_BIT_STREAM_PARAM);
	return RETCODE_SUCCESS;
}

RetCode VPU_DecRegisterFrameBuffer(
		DecHandle handle,
		FrameBuffer * bufArray,
		int num,
		int stride,
		DecBufInfo * pBufInfo)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	int i;
	RetCode ret;


	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->frameBufPool) {
		return RETCODE_CALLED_BEFORE;
	}

	if (!pDecInfo->initialInfoObtained) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (bufArray == 0) {
		return RETCODE_INVALID_FRAME_BUFFER;
	}
	if (num < pDecInfo->initialInfo.minFrameBufferCount) {
		return RETCODE_INSUFFICIENT_FRAME_BUFFERS;
	}
	if (stride < pDecInfo->initialInfo.picWidth ||
			stride % 8 != 0
			) {
		return RETCODE_INVALID_STRIDE;
	}

	pDecInfo->frameBufPool = bufArray;
	pDecInfo->numFrameBuffers = num;
	pDecInfo->stride = stride;

	if (pDecInfo->openParam.bitstreamFormat == STD_MJPG)
		return RETCODE_SUCCESS;

	// Let the decoder know the addresses of the frame buffers.
	for (i = 0; i < num; i++) {
		*(unsigned int *)(paraBuffer + i * 3 *4) = bufArray[i].bufY;
		*(unsigned int *)(paraBuffer + i * 3 *4 + 4) = bufArray[i].bufCb;
		*(unsigned int *)(paraBuffer + i * 3 *4 + 8) = bufArray[i].bufCr;

		if( pDecInfo->openParam.bitstreamFormat == STD_AVC )
			*(unsigned int *)(paraBuffer + (i + 96) *4) = bufArray[i].bufMvCol;
		
	}

	//flushMem();

	int tempNum = num * 3 + 1;
	for(i = 0; i < tempNum; i+=2){
		unsigned int temp1, temp2;
		temp1 = *(unsigned int *)(paraBuffer + i * 4); 
		temp2 = *(unsigned int *)(paraBuffer + i * 4 + 4); 
		*(unsigned int *)(paraBuffer + i*4) = temp2;
		*(unsigned int *)(paraBuffer + i*4+4) = temp1;
	}

	if( pDecInfo->openParam.bitstreamFormat == STD_AVC )
	{
		tempNum = num + 1;
		for(i = 0; i < tempNum; i+=2){
			unsigned int temp1, temp2;
			temp1 = *(unsigned int *)(paraBuffer + (384 + i * 4)); 
			temp2 = *(unsigned int *)(paraBuffer + (384 + i * 4 + 4)); 
			*(unsigned int *)(paraBuffer + (384 + i * 4)) = temp2;
			*(unsigned int *)(paraBuffer + (384 + i * 4 + 4)) = temp1;
		}
	}
	if( pDecInfo->openParam.bitstreamFormat == STD_VC1 
                  || pDecInfo->openParam.bitstreamFormat == STD_MPEG4
                  || pDecInfo->openParam.bitstreamFormat == STD_AVS
                  || pDecInfo->openParam.bitstreamFormat == STD_RV)
	{
		*(unsigned int *)(paraBuffer + 384 + 4) = bufArray[0].bufMvCol;
	}

 	//flushMem();

	// Tell the decoder how much frame buffers were allocated.
	VpuWriteReg(CMD_SET_FRAME_BUF_NUM, num);
	VpuWriteReg(CMD_SET_FRAME_BUF_STRIDE, stride);
	
#if 0
	VpuWriteReg( CMD_SET_FRAME_AXI_BIT_ADDR, pDecInfo->secAxiUse.bufBitUse);
	VpuWriteReg( CMD_SET_FRAME_AXI_IPACDC_ADDR, pDecInfo->secAxiUse.bufIpAcDcUse);
	VpuWriteReg( CMD_SET_FRAME_AXI_DBKY_ADDR, pDecInfo->secAxiUse.bufDbkYUse);
	VpuWriteReg( CMD_SET_FRAME_AXI_DBKC_ADDR, pDecInfo->secAxiUse.bufDbkCUse);
	VpuWriteReg( CMD_SET_FRAME_AXI_OVL_ADDR, pDecInfo->secAxiUse.bufOvlUse);
#endif

	if( pCodecInst->codecMode == AVC_DEC ) {
		VpuWriteReg( CMD_SET_FRAME_SLICE_BB_START, pBufInfo->avcSliceBufInfo.sliceSaveBuffer );
		VpuWriteReg( CMD_SET_FRAME_SLICE_BB_SIZE, (pBufInfo->avcSliceBufInfo.sliceSaveBufferSize/1024) );
	}

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);

	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, SET_FRAME_BUF);
	while (VpuReadReg(BIT_BUSY_FLAG))
		;

	return RETCODE_SUCCESS;
}

RetCode VPU_DecGetBitstreamBuffer( DecHandle handle,
		phyaddr_t * prdPrt,
		phyaddr_t * pwrPtr,
		uint32_t * size)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	phyaddr_t rdPtr;
	phyaddr_t wrPtr;
	uint32_t room;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;


	if ( prdPrt == 0 || pwrPtr == 0 || size == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;


	rdPtr = VpuReadReg(pDecInfo->streamRdPtrRegAddr);
	wrPtr = pDecInfo->streamWrPtr;
	
	if (wrPtr < rdPtr) {
		room = rdPtr - wrPtr - 1;
	}
	else {
		room = ( pDecInfo->streamBufEndAddr - wrPtr ) + ( rdPtr - pDecInfo->streamBufStartAddr ) - 1;	
	}

	*prdPrt = rdPtr;
	*pwrPtr = wrPtr;
	*size = room;

	return RETCODE_SUCCESS;
}


RetCode VPU_DecUpdateBitstreamBuffer(
		DecHandle handle,
		uint32_t size)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	phyaddr_t wrPtr;
	phyaddr_t rdPtr;
	RetCode ret;
	int		val = 0;
	int		room = 0;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	wrPtr = pDecInfo->streamWrPtr;
	
	if (size == 0) 
	{
		val = VpuReadReg( BIT_BIT_STREAM_PARAM );
		val |= 1 << 2;
		VpuWriteReg(BIT_BIT_STREAM_PARAM, val);
		pDecInfo->streamEndflag = val;
		return RETCODE_SUCCESS;
	}
	
	rdPtr = VpuReadReg(pDecInfo->streamRdPtrRegAddr);
	if (wrPtr < rdPtr) {
		if (rdPtr <= wrPtr + size)
			return RETCODE_INVALID_PARAM;
	}
	wrPtr = VpuReadReg(pDecInfo->streamWrPtrRegAddr);
	wrPtr += size;

	if( pDecInfo->filePlayEnable != 1 )  {
		if (wrPtr > pDecInfo->streamBufEndAddr) {
			room = wrPtr - pDecInfo->streamBufEndAddr;
			wrPtr = pDecInfo->streamBufStartAddr;
			wrPtr += room;
		}
		if (wrPtr == pDecInfo->streamBufEndAddr) {
			wrPtr = pDecInfo->streamBufStartAddr;
		}
	}

	pDecInfo->streamWrPtr = wrPtr;
	pDecInfo->streamRdPtr = rdPtr;
	VpuWriteReg(pDecInfo->streamWrPtrRegAddr, wrPtr);

	return RETCODE_SUCCESS;
}

void Vpu_DecSetHostParaAddr(phyaddr_t baseAddr, phyaddr_t paraAddr)
{
	BYTE tempBuf[8];					// 64bit bus & endian
	uint32_t val;

	val = paraAddr;
 	tempBuf[0] = 0;
 	tempBuf[1] = 0;
 	tempBuf[2] = 0;
 	tempBuf[3] = 0;
	tempBuf[4] = (val >> 24) & 0xff;
	tempBuf[5] = (val >> 16) & 0xff;
	tempBuf[6] = (val >> 8) & 0xff;
	tempBuf[7] = (val >> 0) & 0xff;						
	WriteSdramBurst((unsigned char *)tempBuf, baseAddr,  8, 1);				
}

//---- VPU_SLEEP/WAKE
RetCode VPU_SleepWake(int iSleepWake)
{
	static unsigned int regBk[64];
	int i=0;

	if(iSleepWake==1) {
		for ( i = 0 ; i < 64 ; i++)
			regBk[i] = VpuReadReg(BIT_BASE + 0x100 + (i * 4));
		BitIssueCommand(0, 0, VPU_SLEEP);
	}
	else {
		if (VpuReadReg(BIT_CUR_PC) != 0) {
			BitIssueCommand(0, 0, VPU_WAKE);
			return RETCODE_SUCCESS;
		}
		for ( i = 0 ; i < 64 ; i++)
			VpuWriteReg(BIT_BASE + 0x100 + (i * 4), regBk[i]);
//		VpuWriteReg(BIT_RESET_CTRL,	   1);
		VpuWriteReg(BIT_CODE_RUN,	   0);
		//---- LOAD BOOT CODE
		{	uint32_t data;
			for( i = 0; i <2048; i++ ) {
			   data = bit_code[i];
			   VpuWriteReg(BIT_CODE_DOWN, (i << 16) | data);
			}
		}
		VpuWriteReg(BIT_BUSY_FLAG, 1);
		VpuWriteReg(BIT_CODE_RUN, 1);
 		while (VpuReadReg(BIT_BUSY_FLAG))
 			sleep( 1 );
		BitIssueCommand(0, 0, VPU_WAKE);
	}

	while( VPU_IsBusy() ) {
		sleep( 1 );
	}
	return RETCODE_SUCCESS;
}
void Vpu_EncSetHostParaAddr(phyaddr_t baseAddr, phyaddr_t paraAddr)
{
	BYTE tempBuf[8];					// 64bit bus & endian
	uint32_t val;

	val = paraAddr;
 	tempBuf[0] = 0;
 	tempBuf[1] = 0;
 	tempBuf[2] = 0;
 	tempBuf[3] = 0;
	tempBuf[4] = (val >> 24) & 0xff;
	tempBuf[5] = (val >> 16) & 0xff;
	tempBuf[6] = (val >> 8)  & 0xff;
	tempBuf[7] = (val >> 0)  & 0xff;
	WriteSdramBurst((unsigned char *)tempBuf, baseAddr, 8, 1);
}
RetCode VPU_DecStartOneFrame(DecHandle handle, DecParam *param)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	uint32_t rotMir;
	uint32_t val;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
//    fprintf(stderr,"zl--%s,%s,%d,pDecInfo->filePlayEnable=%d\n",__FILE__,__func__,__LINE__,pDecInfo->filePlayEnable);

	if (pDecInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	rotMir = 0;
	if (pDecInfo->rotationEnable) {
		rotMir |= 0x10; // Enable rotator
		switch (pDecInfo->rotationAngle) {
			case 0:
				rotMir |= 0x0;
				break;

			case 90:
				rotMir |= 0x1;
				break;

			case 180:
				rotMir |= 0x2;
				break;

			case 270:
				rotMir |= 0x3;
				break;
		}
	}
	if (pDecInfo->mirrorEnable) {
		rotMir |= 0x10; // Enable rotator
		switch (pDecInfo->mirrorDirection) {
			case MIRDIR_NONE :
				rotMir |= 0x0;
				break;

			case MIRDIR_VER :
				rotMir |= 0x4;
				break;

			case MIRDIR_HOR :
				rotMir |= 0x8;
				break;

			case MIRDIR_HOR_VER :
				rotMir |= 0xc;
				break;

		}
	}
	if (pDecInfo->deringEnable) {
		rotMir |= 0x20; // Enable Dering Filter
	}

	if( pCodecInst->codecMode == MJPG_DEC ) {
		VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_Y, pDecInfo->rotatorOutput.bufY);
		VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CB, pDecInfo->rotatorOutput.bufCb);
		VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CR, pDecInfo->rotatorOutput.bufCr);
		VpuWriteReg(CMD_DEC_PIC_ROT_STRIDE, pDecInfo->rotatorStride);
	} else {
		if (rotMir & 0x30) // rotator or dering enabled
		{
			VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_Y, pDecInfo->rotatorOutput.bufY);
			VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CB, pDecInfo->rotatorOutput.bufCb);
			VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CR, pDecInfo->rotatorOutput.bufCr);
			VpuWriteReg(CMD_DEC_PIC_ROT_STRIDE, pDecInfo->rotatorStride);
		}
	}

	VpuWriteReg(CMD_DEC_PIC_ROT_MODE, rotMir);

#ifdef API_CR
	if (pDecInfo->mbParamEnable || pDecInfo->mvReportEnable || pDecInfo->frameBufStatEnable) {
		VpuWriteReg(CMD_DEC_PIC_PARA_BASE_ADDR, pDecInfo->picParaBaseAddr);
	
	Vpu_DecSetHostParaAddr(pDecInfo->picParaBaseAddr		, pDecInfo->frameBufStateAddr);
	Vpu_DecSetHostParaAddr(pDecInfo->picParaBaseAddr + 8	, pDecInfo->mbParamDataAddr);
	Vpu_DecSetHostParaAddr(pDecInfo->picParaBaseAddr + 16	, pDecInfo->mvDataAddr);
	}
	
	
	
	if(pDecInfo->userDataEnable) {
		VpuWriteReg(CMD_DEC_PIC_USER_DATA_BASE_ADDR, pDecInfo->userDataBufAddr);
		VpuWriteReg(CMD_DEC_PIC_USER_DATA_BUF_SIZE, pDecInfo->userDataBufSize);
	}
#endif


	val = 0;
	if( param->iframeSearchEnable == 1 ) { // if iframeSearch is Enable, other bit is ignore;
		
		val |= (pDecInfo->userDataReportMode		<<10 );
		val |= (pDecInfo->frameBufStatEnable		<< 8 );
		val |= (pDecInfo->mbParamEnable				<< 7 );
		val |= (pDecInfo->mvReportEnable			<< 6 );
		val |= (( param->iframeSearchEnable &0x1)   << 2 );

	} else {
#ifdef API_CR	
		if (param->skipframeMode) {
			val |= (pDecInfo->userDataReportMode		<<10 );
			val |= (pDecInfo->frameBufStatEnable		<< 8 );
			val |= (pDecInfo->mbParamEnable				<< 7 );
			val |= (pDecInfo->mvReportEnable			<< 6 );
			val |= (param->skipframeMode				<< 3 );
			val |= (param->prescanMode					<< 1 );
			val |= (param->prescanEnable					 );
		} else {
#ifdef JPEG_PARTIAL
			if( pCodecInst->codecMode == MJPG_DEC )
				val |= (pDecInfo->jpgPartialDecode		<<11 );
#endif // JPEG_PARTIAL
			val |= (pDecInfo->userDataReportMode		<<10 );
			val |= (pDecInfo->frameBufStatEnable		<< 8 );
			val |= (pDecInfo->mbParamEnable				<< 7 );
			val |= (pDecInfo->mvReportEnable			<< 6 );
			val |= (pDecInfo->userDataEnable			<< 5 );
			val	|= (param->prescanMode					<< 1 );
			val	|= (param->prescanEnable					 );
		}
#else
		val |= (pDecInfo->userDataReportMode		<<10 );
		val |= (param->skipframeMode				<< 3 );
		val |= (param->prescanMode					<< 1 );
		val |= (param->prescanEnable					 );
#endif

	}

	VpuWriteReg( CMD_DEC_PIC_OPTION, val );
	
#ifdef JPEG_PARTIAL
	VpuWriteReg( CMD_DEC_PIC_JPG_PART_MB_SIZE, param->jpgPartialMbNum );
#else
	VpuWriteReg( CMD_DEC_PIC_SKIP_NUM, param->skipframeNum );
#endif // JPEG_PARTIAL

	
	if( pDecInfo->filePlayEnable == 1 )
	{
		VpuWriteReg( CMD_DEC_PIC_CHUNK_SIZE, param->chunkSize );
		if( pDecInfo->dynamicAllocEnable == 1 )
			VpuWriteReg( CMD_DEC_PIC_BB_START, param->picStreamBufferAddr );					
		
		VpuWriteReg(CMD_DEC_PIC_START_BYTE, param->picStartByteOffset);
	}

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);

	// Secondary channel
	val = ( 
		pDecInfo->secAxiUse.useBitEnable	<< 0 | 
		pDecInfo->secAxiUse.useIpEnable		<< 1 |
		pDecInfo->secAxiUse.useDbkYEnable	<< 2 |
		pDecInfo->secAxiUse.useDbkCEnable	<< 3 |
		pDecInfo->secAxiUse.useOvlEnable	<< 4 | 
		pDecInfo->secAxiUse.useBtpEnable	<< 5 | 
		pDecInfo->secAxiUse.useMeEnable		<< 6 | 
		pDecInfo->secAxiUse.useHostBitEnable    << 8 |
		pDecInfo->secAxiUse.useHostIpEnable     << 9 |
		pDecInfo->secAxiUse.useHostDbkYEnable   << 10 | 
		pDecInfo->secAxiUse.useHostDbkCEnable   << 11 | 
		pDecInfo->secAxiUse.useHostOvlEnable    << 12 |
		pDecInfo->secAxiUse.useHostBtpEnable    << 13 |
		pDecInfo->secAxiUse.useHostMeEnable     << 14);	

	VpuWriteReg( BIT_AXI_SRAM_USE, val);


	VpuWriteReg(pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pDecInfo->streamRdPtrRegAddr, pDecInfo->streamRdPtr);
	VpuWriteReg(pDecInfo->frameDisplayFlagRegAddr, pDecInfo->frameDisplayFlag);
	VpuWriteReg(BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);
	VpuWriteReg(BIT_FRAME_MEM_CTRL, (CBCR_INTERLEAVE<< 2) | IMAGE_ENDIAN);		// BIT_FRAME_MEM_CTRL(CbCrInterleave) should be set every PIC_RUN command.
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, PIC_RUN);

	pendingInst = pCodecInst;

	return RETCODE_SUCCESS;
}

#ifdef	JPEG_PARTIAL
RetCode VPU_DecSetBasesJpgPartial(DecHandle handle, int jpg_partial_size, int jpg_partial_cnt, int jpg_mb_size)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	uint32_t offsetLuma = jpg_partial_cnt * jpg_partial_size;
	uint32_t offsetChroma = jpg_partial_cnt * jpg_partial_size / (jpg_mb_size>>6);
	
	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_Y, pDecInfo->rotatorOutput.bufY-offsetLuma);
	VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CB, pDecInfo->rotatorOutput.bufCb-offsetChroma);
	VpuWriteReg(CMD_DEC_PIC_ROT_ADDR_CR, pDecInfo->rotatorOutput.bufCr-offsetChroma);

	return RETCODE_SUCCESS;
}

RetCode VPU_EncSetBasesJpgPartial(EncHandle handle, EncParam * param, int jpg_partial_size, int jpg_partial_cnt, int jpg_mb_size)
{
	CodecInst * pCodecInst;
	FrameBuffer * pSrcFrame;
	RetCode ret;

	uint32_t offsetLuma = jpg_partial_cnt * jpg_partial_size;
	uint32_t offsetChroma = jpg_partial_cnt * jpg_partial_size / (jpg_mb_size>>6);
	
	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;
	
	pCodecInst = handle;
	pSrcFrame = param->sourceFrame;

	VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_Y, pSrcFrame->bufY-offsetLuma);
	VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_CB, pSrcFrame->bufCb-offsetChroma);
	VpuWriteReg(CMD_ENC_PIC_SRC_ADDR_CR, pSrcFrame->bufCr-offsetChroma);
	
	return RETCODE_SUCCESS;
}
#endif // JPEG_PARTIAL

RetCode VPU_DecGetOutputInfo(
		DecHandle handle,
		DecOutputInfo * info)
{
	CodecInst * pCodecInst;
	DecInfo	  *	pDecInfo;
	RetCode		ret;
	uint32_t		val = 0;
	uint32_t		val2 = 0;
	int			i = 0;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pendingInst == 0) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pCodecInst != pendingInst) {

		return RETCODE_INVALID_HANDLE;
	}

	val	= VpuReadReg(RET_DEC_PIC_SUCCESS);
	info->decodingSuccess = (val & 0x01);

	if( pCodecInst->codecMode == AVC_DEC )
	{
		info->notSufficientPsBuffer = (val >> 3) & 0x1;
		info->notSufficientSliceBuffer = (val >> 2) & 0x1;
	}
	if( pCodecInst->codecMode == MP4_DEC )
		info->mp4PackedPBframe = ((val >> 16) & 0x01);
	
	val = VpuReadReg(RET_DEC_PIC_SIZE);				// decoding picture size
	info->decPicHeight	= val & 0xFFFF;
	info->decPicWidth	= (val>>16) & 0xFFFF;

#ifdef PIC_ASPECT_F_RATE
	info->picAspectRatio	= VpuReadReg(RET_DEC_PIC_ASPECT);	
	info->picFrateNr		= VpuReadReg(RET_DEC_PIC_FRATE_NR);
	info->picFrateDr		= VpuReadReg(RET_DEC_PIC_FRATE_DR);
#endif
	if( pCodecInst->codecMode == AVC_DEC )
	{
		val = VpuReadReg(RET_DEC_PIC_CROP_LEFT_RIGHT);				// frame crop information(left, right)
		val2 = VpuReadReg(RET_DEC_PIC_CROP_TOP_BOTTOM);				// frame crop information(top, bottom)
		if( val == 0 && val2 == 0 )
		{
			info->decPicCrop.left = 0;
			info->decPicCrop.right = 0;
			info->decPicCrop.top = 0;
			info->decPicCrop.bottom	= 0;
		}
		else
		{
			info->decPicCrop.left = ( (val>>16) & 0xFFFF );
			info->decPicCrop.right = info->decPicWidth - ( val & 0xFFFF );
			info->decPicCrop.top = ( (val2>>16) & 0xFFFF );
			info->decPicCrop.bottom	= info->decPicHeight - (val2 & 0xFFFF);
		}
	}

	val = VpuReadReg(RET_DEC_PIC_TYPE);
	info->picType = val & 7;
	info->picTypeFirst = (val & 0x38) >> 3;

#ifndef API_CR	
	info->interlacedFrame = (val >> 16) & 0x1;	
#else

	info->h264PicStruct		= (val >>  8) & 0xF;
	info->interlacedFrame	= (val >> 18) & 0x1;
	info->pictureStructure  = (val >> 19) & 0x0003;	// MbAffFlag[17], FieldPicFlag[16]
	info->topFieldFirst     = (val >> 21) & 0x0001;	// TopFieldFirst[18]
	info->repeatFirstField  = (val >> 22) & 0x0001;
	info->progressiveFrame  = (val >> 23) & 0x0003;
	info->fieldSequence     = (val >> 25) & 0x0007;
	info->refFlag			= (val >> 30) & 0x1;
	// Frame Buffer Status
	if (pDecInfo->frameBufStatEnable) {
		int size, paraInfo, address;
		
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);
		
		ReadSdramBurst(tempBuf, pDecInfo->picParaBaseAddr, 8, 1);
		
		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		address = ((tempBuf[4]<<24) & 0xFF000000) |
				((tempBuf[5]<<16) & 0x00FF0000) |
				((tempBuf[6]<< 8) & 0x0000FF00) |
				((tempBuf[7]<< 0) & 0x000000FF);

		paraInfo = (val >> 24) & 0xFF;
		size = (val >>  0) & 0xFFFFFF;
		
		if (paraInfo == PARA_TYPE_FRM_BUF_STATUS) {
			info->decOutputExtData.frameBufDataSize = size;
			info->decOutputExtData.frameBufStatDataAddr = pDecInfo->frameBufStateAddr;
		} else {
			// VPU does not write data
			info->decOutputExtData.frameBufDataSize = 0;
			info->decOutputExtData.frameBufStatDataAddr = 0;
		}
	}

	// Mb Param
	if (pDecInfo->mbParamEnable) {
		int size, paraInfo, address;
		
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);
		
		ReadSdramBurst(tempBuf, pDecInfo->picParaBaseAddr + 8, 8, 1);

		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		address = ((tempBuf[4]<<24) & 0xFF000000) |
				((tempBuf[5]<<16) & 0x00FF0000) |
				((tempBuf[6]<< 8) & 0x0000FF00) |
				((tempBuf[7]<< 0) & 0x000000FF);

		paraInfo = (val >> 24) & 0xFF;
		size = (val >>  0) & 0x00FFFF;
		
		if (paraInfo == PARA_TYPE_MB_PARA) {
			info->decOutputExtData.mbParamDataSize = size;
			info->decOutputExtData.mbParamDataAddr = pDecInfo->mbParamDataAddr;
		} else {
			// VPU does not write data
			info->decOutputExtData.mbParamDataSize = 0;
			info->decOutputExtData.mbParamDataAddr = 0;
		}
	}

	// Motion Vector
	if (pDecInfo->mvReportEnable) {
		int size, paraInfo, address, mvNumPerMb;
		
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);
		
		ReadSdramBurst(tempBuf, pDecInfo->picParaBaseAddr + 16, 8, 1);
		
		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		address = ((tempBuf[4]<<24) & 0xFF000000) |
				((tempBuf[5]<<16) & 0x00FF0000) |
				((tempBuf[6]<< 8) & 0x0000FF00) |
				((tempBuf[7]<< 0) & 0x000000FF);

		paraInfo	= (val >> 24) & 0xFF;
		mvNumPerMb	= (val >> 16) & 0xFF;
		size		= (val >>  0) & 0xFFFF;
		
		if (paraInfo == PARA_TYPE_MV) {
			info->decOutputExtData.mvDataSize = size;
			info->decOutputExtData.mvNumPerMb = mvNumPerMb;
			info->decOutputExtData.mvDataAddr = pDecInfo->mvDataAddr;
		} else {
			// VPU does not write data
			info->decOutputExtData.mvNumPerMb = 0;
			info->decOutputExtData.mvDataAddr = 0;
		}
	}
	
	// User Data
	if (pDecInfo->userDataEnable) {
		int userDataNum;
		int userDataSize;
		BYTE tempBuf[8];
		memset(tempBuf, 0, 8);
		
		ReadSdramBurst(tempBuf, pDecInfo->userDataBufAddr + 0, 8, 1);
		
		val =	((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);
		
		userDataNum = (val >> 16) & 0xFFFF;
		userDataSize = (val >> 0) & 0xFFFF;
		if (userDataNum == 0)
			userDataSize = 0;
		
		info->decOutputExtData.userDataNum = userDataNum;
		info->decOutputExtData.userDataSize = userDataSize;
		
		val =	((tempBuf[4]<<24) & 0xFF000000) |
				((tempBuf[5]<<16) & 0x00FF0000) |
				((tempBuf[6]<< 8) & 0x0000FF00) |
				((tempBuf[7]<< 0) & 0x000000FF);

		if (userDataNum == 0)
			info->decOutputExtData.userDataBufFull = 0;
		else
			info->decOutputExtData.userDataBufFull = (val >> 16) & 0xFFFF;
	}
#endif

	info->indexFrameDecoded		= VpuReadReg(RET_DEC_PIC_CUR_IDX);
	info->indexFrameDisplay		= VpuReadReg(RET_DEC_PIC_FRAME_IDX);
	info->numOfErrMBs		= VpuReadReg(RET_DEC_PIC_ERR_MB);
	info->prescanresult		= VpuReadReg(RET_DEC_PIC_OPTION );
	info->decodingSuccess	= (VpuReadReg(RET_DEC_PIC_SUCCESS) & 0x01);

	if( pCodecInst->codecMode == VC1_DEC )
	{
		val = ReadReg(RET_DEC_PIC_POST);
		info->hScaleFlag = val >> 1 & 1 ;
		info->vScaleFlag = val >> 2 & 1 ;
	}	

	if (pCodecInst->codecMode == VC1_DEC && info->indexFrameDisplay != -3) {
		if (pDecInfo->vc1BframeDisplayValid == 0) {
			if ( info->picType == 2) {
				info->indexFrameDisplay = -3;
			} else {
				pDecInfo->vc1BframeDisplayValid = 1;
			}
		}
	}

	pendingInst = 0;

	pDecInfo->streamWrPtr = VpuReadReg(pDecInfo->streamWrPtrRegAddr);
	pDecInfo->streamRdPtr = VpuReadReg(pDecInfo->streamRdPtrRegAddr);
	pDecInfo->frameDisplayFlag = VpuReadReg(pDecInfo->frameDisplayFlagRegAddr);	
	return RETCODE_SUCCESS;
}

RetCode VPU_DecBitBufferFlush(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);

	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, DEC_BUF_FLUSH);
	while (VpuReadReg(BIT_BUSY_FLAG));

	pDecInfo->streamWrPtr = pDecInfo->streamBufStartAddr;
	VpuWriteReg(pDecInfo->streamWrPtrRegAddr, pDecInfo->streamBufStartAddr);	

	return RETCODE_SUCCESS;
}

RetCode VPU_DecResetRdPtr(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	unsigned int new_rdptr;
	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);

	VpuWriteReg(CMD_DEC_BUF_FLUSH_TYPE, 1);
	printf("\nSet New RDPTR : ");
	scanf("%x", &new_rdptr);
	VpuWriteReg(CMD_DEC_BUF_FLUSH_RDPTR, new_rdptr);

	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, DEC_BUF_FLUSH);

	while (VpuReadReg(BIT_BUSY_FLAG))
		;

	return RETCODE_SUCCESS;
}
RetCode VPU_DecClrDispFlag(DecHandle handle, int index)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	int	val;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if ((index < 0) || (index > (pDecInfo->numFrameBuffers - 1)))
		return	RETCODE_INVALID_PARAM;

 	val = 1 << index;
 	val = ~val;
 	val = (val & VpuReadReg(pDecInfo->frameDisplayFlagRegAddr));
	pDecInfo->frameDisplayFlag = val;
	return RETCODE_SUCCESS;
}


RetCode VPU_DecGiveCommand(
		DecHandle handle,
		CodecCommand cmd,
		void * param)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (pendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	switch (cmd) 
	{
		case ENABLE_ROTATION :
			{
				if (!pDecInfo->rotatorOutputValid) {
					return RETCODE_ROTATOR_OUTPUT_NOT_SET;
				}
				if (pDecInfo->rotatorStride == 0) {
					return RETCODE_ROTATOR_STRIDE_NOT_SET;
				}
				pDecInfo->rotationEnable = 1;
				break;
			}

		case DISABLE_ROTATION :
			{
				pDecInfo->rotationEnable = 0;
				break;
			}

		case ENABLE_MIRRORING :
			{
				if (!pDecInfo->rotatorOutputValid) {
					return RETCODE_ROTATOR_OUTPUT_NOT_SET;
				}
				if (pDecInfo->rotatorStride == 0) {
					return RETCODE_ROTATOR_STRIDE_NOT_SET;
				}
				pDecInfo->mirrorEnable = 1;
				break;
			}
		case DISABLE_MIRRORING :
			{
				pDecInfo->mirrorEnable = 0;
				break;
			}
		case SET_MIRROR_DIRECTION :
			{

				MirrorDirection mirDir;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				mirDir = *(MirrorDirection *)param;
				if (!(MIRDIR_NONE <= mirDir && mirDir <= MIRDIR_HOR_VER)) {
					return RETCODE_INVALID_PARAM;
				}
				pDecInfo->mirrorDirection = mirDir;

				break;
			}
		case SET_ROTATION_ANGLE :
			{
				int angle;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				angle = *(int *)param;
				if (angle != 0 && angle != 90 &&
						angle != 180 && angle != 270) {
					return RETCODE_INVALID_PARAM;
				}
				if (pDecInfo->rotatorStride != 0) {				
					if (angle == 90 || angle ==270) {
						if (pDecInfo->initialInfo.picHeight > pDecInfo->rotatorStride) {
							return RETCODE_INVALID_PARAM;
						}
					} else {
						if (pDecInfo->initialInfo.picWidth > pDecInfo->rotatorStride) {
							return RETCODE_INVALID_PARAM;
						}
					}
				}
							
				pDecInfo->rotationAngle = angle;
				break;
			}

		case SET_ROTATOR_OUTPUT :
			{
				FrameBuffer * frame;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				frame = (FrameBuffer *)param;
				pDecInfo->rotatorOutput = *frame;
				pDecInfo->rotatorOutputValid = 1;
				break;
			}

		case SET_ROTATOR_STRIDE :
			{
				int stride;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				stride = *(int *)param;
				if (stride % 8 != 0 || stride == 0) {
					return RETCODE_INVALID_STRIDE;
				}
				if (pDecInfo->rotationAngle == 90 || pDecInfo->rotationAngle == 270) {
					if (pDecInfo->initialInfo.picHeight > stride) {
						return RETCODE_INVALID_STRIDE;
					}
				} else {
					if (pDecInfo->initialInfo.picWidth > stride) {
						return RETCODE_INVALID_STRIDE;
					}					
				}

				pDecInfo->rotatorStride = stride;
				break;
			}
		case DEC_SET_SPS_RBSP:
			{
				if (pCodecInst->codecMode != AVC_DEC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				SetParaSet(handle, 0, param);
				break;
			}

		case DEC_SET_PPS_RBSP:
			{
				if (pCodecInst->codecMode != AVC_DEC) {
					return RETCODE_INVALID_COMMAND;
				}
				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				SetParaSet(handle, 1, param);
				break;
			}
		case ENABLE_DERING :
			{
				if (!pDecInfo->rotatorOutputValid) {
					return RETCODE_ROTATOR_OUTPUT_NOT_SET;
				}
				if (pDecInfo->rotatorStride == 0) {
					return RETCODE_ROTATOR_STRIDE_NOT_SET;
				}
				pDecInfo->deringEnable = 1;
				break;
			}

		case DISABLE_DERING :
			{
				pDecInfo->deringEnable = 0;
				break;
			}
		case SET_SEC_AXI:
			{
				SecAxiUse secAxiUse;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				secAxiUse = *(SecAxiUse *)param;

				pDecInfo->secAxiUse.useBitEnable = (secAxiUse.useBitEnable & 0x1);
				pDecInfo->secAxiUse.useIpEnable = (secAxiUse.useIpEnable & 0x1);
				pDecInfo->secAxiUse.useDbkYEnable = (secAxiUse.useDbkYEnable & 0x1);
				pDecInfo->secAxiUse.useDbkCEnable = (secAxiUse.useDbkCEnable & 0x1);
				pDecInfo->secAxiUse.useOvlEnable = (secAxiUse.useOvlEnable & 0x1);
				pDecInfo->secAxiUse.useBtpEnable = (secAxiUse.useBtpEnable & 0x1);
//				pDecInfo->secAxiUse.useMeEnable = (secAxiUse.useMeEnable & 0x1);

				pDecInfo->secAxiUse.useHostBitEnable = (secAxiUse.useHostBitEnable & 0x1);
				pDecInfo->secAxiUse.useHostIpEnable = (secAxiUse.useHostIpEnable & 0x1);
				pDecInfo->secAxiUse.useHostDbkYEnable = (secAxiUse.useHostDbkYEnable & 0x1);
				pDecInfo->secAxiUse.useHostDbkCEnable = (secAxiUse.useHostDbkCEnable & 0x1);
				pDecInfo->secAxiUse.useHostOvlEnable = (secAxiUse.useHostOvlEnable & 0x1);
				pDecInfo->secAxiUse.useHostBtpEnable = (secAxiUse.useHostOvlEnable & 0x1);
//				pDecInfo->secAxiUse.useHostMeEnable = (secAxiUse.useHostMeEnable & 0x1);

				pDecInfo->secAxiUse.bufBitUse = secAxiUse.bufBitUse;
				pDecInfo->secAxiUse.bufIpAcDcUse = secAxiUse.bufIpAcDcUse;
				pDecInfo->secAxiUse.bufDbkYUse = secAxiUse.bufDbkYUse;
				pDecInfo->secAxiUse.bufDbkCUse = secAxiUse.bufDbkCUse;
				pDecInfo->secAxiUse.bufOvlUse = secAxiUse.bufOvlUse;
				pDecInfo->secAxiUse.bufBtpUse = secAxiUse.bufBtpUse;

				break;
			}
		case ENABLE_REP_BUFSTAT:
			{
				pDecInfo->frameBufStatEnable = 1;
				break;
			}
		case DISABLE_REP_BUFSTAT:
			{
				pDecInfo->frameBufStatEnable = 0;
				break;
			}
		case ENABLE_REP_MBPARAM:
			{
				pDecInfo->mbParamEnable = 1;
				break;
			}
		case DISABLE_REP_MBPARAM:
			{
				pDecInfo->mbParamEnable = 0;
				break;
			}
		case ENABLE_REP_MV:
			{
				pDecInfo->mvReportEnable = 1;
				break;
			}
		case DISABLE_REP_MV:
			{
				pDecInfo->mvReportEnable = 0;
				break;
			}

#ifdef JPEG_PARTIAL
		case ENABLE_JPG_PARTIAL:
			{
				pDecInfo->jpgPartialDecode = 1;
			}
			break;
		case DISABLE_JPG_PARTIAL:
			{
				pDecInfo->jpgPartialDecode = 0;
			}
			break;
#endif // JPEG_PARTIAL
		case ENABLE_REP_USERDATA:
			{
				if (!pDecInfo->userDataBufAddr) {
					return RETCODE_USERDATA_BUF_NOT_SET;
				}
				if (pDecInfo->userDataBufSize == 0) {
					return RETCODE_USERDATA_BUF_NOT_SET;
				}
				pDecInfo->userDataEnable = 1;
				break;
			}
		case DISABLE_REP_USERDATA:
			{
				pDecInfo->userDataEnable = 0;
				break;
			}
		case SET_ADDR_REP_PICPARAM:
			{
				phyaddr_t picParaBaseAddr;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				picParaBaseAddr = *(phyaddr_t *)param;
				if (picParaBaseAddr % 8 != 0 || picParaBaseAddr == 0) {
					return RETCODE_INVALID_PARAM;
				}

				pDecInfo->picParaBaseAddr = picParaBaseAddr;
				break;
			}
		case SET_ADDR_REP_USERDATA:
			{
				phyaddr_t userDataBufAddr;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				userDataBufAddr = *(phyaddr_t *)param;
				if (userDataBufAddr % 8 != 0 || userDataBufAddr == 0) {
					return RETCODE_INVALID_PARAM;
				}

				pDecInfo->userDataBufAddr = userDataBufAddr;
				break;
			}
		case SET_SIZE_REP_USERDATA:
			{
				phyaddr_t userDataBufSize;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				userDataBufSize = *(phyaddr_t *)param;

				pDecInfo->userDataBufSize = userDataBufSize;
				break;
			}
			
		case SET_USERDATA_REPORT_MODE:
			{
				int userDataMode;

				userDataMode = *(int *)param;
				if (userDataMode != 1 && userDataMode != 0) {
					return RETCODE_INVALID_PARAM;
				}
				pDecInfo->userDataReportMode = userDataMode;
				break;
			}
		case SET_ADDR_REP_MB_PARAM:
			{
				phyaddr_t mbParamDataAddr;

				if (param == 0) {
					return RETCODE_INVALID_PARAM;
				}
				mbParamDataAddr = *(phyaddr_t *)param;
				if (mbParamDataAddr % 8 != 0 || mbParamDataAddr == 0) {
					return RETCODE_INVALID_PARAM;
				}

				pDecInfo->mbParamDataAddr = mbParamDataAddr;
				break;

			}
		case SET_ADDR_REP_MV_DATA:
			{
				phyaddr_t mvDataAddr = *(phyaddr_t *)param;
				if (param == 0)
					return RETCODE_INVALID_PARAM;
				if (mvDataAddr % 8 != 0 || mvDataAddr == 0)
					return RETCODE_INVALID_PARAM;
				
				pDecInfo->mvDataAddr = mvDataAddr;
				break;

			}
		case SET_ADDR_REP_BUF_STATE:
			{
				phyaddr_t frameBufStateAddr = *(phyaddr_t *)param;
				if (param == 0)
					return RETCODE_INVALID_PARAM;
				if (frameBufStateAddr % 8 != 0 || frameBufStateAddr == 0)
					return RETCODE_INVALID_PARAM;
				
				pDecInfo->frameBufStateAddr = frameBufStateAddr;
				break;
			}
		default:
			return RETCODE_INVALID_COMMAND;
	}
	return RETCODE_SUCCESS;
}
