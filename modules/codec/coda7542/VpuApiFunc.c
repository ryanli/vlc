//------------------------------------------------------------------------------
// File: VpuApiFunc.c
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------

#include <stdio.h>
#include "VpuApiFunc.h"
#include "RegDefine.h"
 

extern CodecInst codecInstPool[MAX_NUM_INSTANCE];
extern phyaddr_t workBuffer;
extern phyaddr_t codeBuffer;
extern phyaddr_t paraBuffer;

void BitIssueCommand(int instIdx, int cdcMode, int cmd)
{
	VpuWriteReg(BIT_BUSY_FLAG, 1);
	VpuWriteReg(BIT_RUN_INDEX, instIdx);
	VpuWriteReg(BIT_RUN_COD_STD, cdcMode);
	VpuWriteReg(BIT_RUN_COMMAND, cmd);
}

/*
 * GetCodecInstance() obtains a instance.
 * It stores a pointer to the allocated instance in *ppInst
 * and returns RETCODE_SUCCESS on success.
 * Failure results in 0(null pointer) in *ppInst and RETCODE_FAILURE.
 */

RetCode GetCodecInstance(CodecInst ** ppInst)
{
	int i;
	CodecInst * pCodecInst;

	pCodecInst = &codecInstPool[0];
	for (i = 0; i < MAX_NUM_INSTANCE; ++i, ++pCodecInst) {
		if (!pCodecInst->inUse)
			break;
	}

	if (i == MAX_NUM_INSTANCE) {
		*ppInst = 0;
		return RETCODE_FAILURE;
	}

	pCodecInst->inUse = 1;
	*ppInst = pCodecInst;
	return RETCODE_SUCCESS;
}

void FreeCodecInstance(CodecInst * pCodecInst)
{
	pCodecInst->inUse = 0;
}

RetCode CheckInstanceValidity(CodecInst * pci)
{
	CodecInst * pCodecInst;
	int i;

	pCodecInst = &codecInstPool[0];
	for (i = 0; i < MAX_NUM_INSTANCE; ++i, ++pCodecInst) {
		if (pCodecInst == pci)
			return RETCODE_SUCCESS;
	}
	return RETCODE_INVALID_HANDLE;
}

RetCode CheckEncInstanceValidity(EncHandle handle)
{
	CodecInst * pCodecInst;
	RetCode ret;

	pCodecInst = handle;
	ret = CheckInstanceValidity(pCodecInst);
	if (ret != RETCODE_SUCCESS) {
		return RETCODE_INVALID_HANDLE;
	}
	if (!pCodecInst->inUse) {
		return RETCODE_INVALID_HANDLE;
	}

	if (pCodecInst->codecMode != MP4_ENC && 
		pCodecInst->codecMode != AVC_ENC &&
		pCodecInst->codecMode != MJPG_ENC ) {
		return RETCODE_INVALID_HANDLE;
	}
	return RETCODE_SUCCESS;
}

RetCode CheckEncOpenParam(EncOpenParam * pop)
{
	int picWidth;
	int picHeight;

	if (pop == 0) {
		return RETCODE_INVALID_PARAM;
	}
	picWidth = pop->picWidth;
	picHeight = pop->picHeight;

	if (pop->bitstreamBuffer % 4) { // not 4-bit aligned
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitstreamBufferSize % 1024 ||
			pop->bitstreamBufferSize < 1024 ||
			pop->bitstreamBufferSize > 16383 * 1024) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitstreamFormat != STD_MPEG4 &&
			pop->bitstreamFormat != STD_H263 &&
			pop->bitstreamFormat != STD_AVC &&
			pop->bitstreamFormat != STD_MJPG ) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitRate > 32767 || pop->bitRate < 0) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitRate !=0 && pop->initialDelay > 32767) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitRate !=0 && pop->initialDelay != 0 && pop->vbvBufferSize < 0) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->enableAutoSkip != 0 && pop->enableAutoSkip != 1) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->gopSize > 60) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->slicemode.sliceMode != 0 && pop->slicemode.sliceMode != 1) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->slicemode.sliceMode == 1) {
		if (pop->slicemode.sliceSizeMode != 0 && pop->slicemode.sliceSizeMode != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (pop->slicemode.sliceSize == 0) {
			return RETCODE_INVALID_PARAM;
		}
	}
	if (pop->intraRefresh < 0 || pop->intraRefresh >= (picWidth * picHeight /256)) {
		return RETCODE_INVALID_PARAM;
	}

	if (pop->bitstreamFormat == STD_MPEG4) {
		EncMp4Param * param = &pop->EncStdParam.mp4Param;
		if (param->mp4_dataPartitionEnable != 0 && param->mp4_dataPartitionEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->mp4_dataPartitionEnable == 1) {
			if (param->mp4_reversibleVlcEnable != 0 && param->mp4_reversibleVlcEnable != 1) {
				return RETCODE_INVALID_PARAM;
			}
		}
		if (param->mp4_intraDcVlcThr < 0 || 7 < param->mp4_intraDcVlcThr) {
			return RETCODE_INVALID_PARAM;
		}
		if (picWidth < 48 || picWidth > MAX_ENC_PIC_WIDTH ) {
			return RETCODE_INVALID_PARAM;
		}
		if (picHeight < 16 || picHeight > MAX_ENC_PIC_HEIGHT ) {
			return RETCODE_INVALID_PARAM;
		}
	}
	else if (pop->bitstreamFormat == STD_H263) {
		EncH263Param * param = &pop->EncStdParam.h263Param;
		uint32_t frameRateInc, frameRateRes;
		if (param->h263_annexJEnable != 0 && param->h263_annexJEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->h263_annexKEnable != 0 && param->h263_annexKEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->h263_annexTEnable != 0 && param->h263_annexTEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->h263_annexJEnable == 0 &&
				param->h263_annexKEnable == 0 &&
				param->h263_annexTEnable == 0
		   ) {
			if (!(picWidth == 128 && picHeight == 96) &&
					!(picWidth == 176 && picHeight == 144) &&
					!(picWidth == 352 && picHeight == 288) &&
					!(picWidth == 704 && picHeight == 576)) {
				return RETCODE_INVALID_PARAM;
			}
		}
		if (picWidth < 64 || picWidth > MAX_ENC_PIC_WIDTH ) {
			return RETCODE_INVALID_PARAM;
		}
		if (picHeight < 16 || picHeight > MAX_ENC_PIC_HEIGHT ) {
			return RETCODE_INVALID_PARAM;
		}
		frameRateInc = pop->frameRateDr;
		frameRateRes = pop->frameRateNr;
		if( (frameRateRes/frameRateInc) <15 )
			return RETCODE_INVALID_PARAM;

	}
	else if (pop->bitstreamFormat == STD_AVC) {
		EncAvcParam * param = &pop->EncStdParam.avcParam;
		if (param->avc_constrainedIntraPredFlag != 0 && param->avc_constrainedIntraPredFlag != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_disableDeblk != 0 && param->avc_disableDeblk != 1 && param->avc_disableDeblk != 2) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_deblkFilterOffsetAlpha < -6 || 6 < param->avc_deblkFilterOffsetAlpha) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_deblkFilterOffsetBeta < -6 || 6 < param->avc_deblkFilterOffsetBeta) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_chromaQpOffset < -12 || 12 < param->avc_chromaQpOffset) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_audEnable != 0 && param->avc_audEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_fmoEnable != 0 && param->avc_fmoEnable != 1) {
			return RETCODE_INVALID_PARAM;
		}
		if (param->avc_fmoEnable == 1 ) {
			if (param->avc_fmoType != 0 && param->avc_fmoType != 1) {
				return RETCODE_INVALID_PARAM;
			}
			if (param->avc_fmoSliceNum < 2 || 8 < param->avc_fmoSliceNum ) {
				return RETCODE_INVALID_PARAM;
			}
		}
		if (picWidth < 64 || picWidth > MAX_ENC_PIC_WIDTH ) {
			return RETCODE_INVALID_PARAM;
		}
		if (picHeight < 16 || picHeight > MAX_ENC_PIC_HEIGHT ) {
			return RETCODE_INVALID_PARAM;
		}
		
	}
	else if (pop->bitstreamFormat == STD_MJPG) {
		if (picWidth < 16 || picWidth > MAX_ENC_MJPG_PIC_WIDTH ) {
			return RETCODE_INVALID_PARAM;
		}
		if (picHeight < 16 || picHeight > MAX_ENC_MJPG_PIC_HEIGHT ) {
			return RETCODE_INVALID_PARAM;
		}		
	}

	return RETCODE_SUCCESS;
}

RetCode CheckEncParam(CodecInst * pCodecInst, EncParam * param)
{
	if (param == 0) {
		return RETCODE_INVALID_PARAM;
	}
	if (param->skipPicture != 0 && param->skipPicture != 1) {
		return RETCODE_INVALID_PARAM;
	}
	if (param->skipPicture == 0) {
		if (param->sourceFrame == 0) {
			return RETCODE_INVALID_FRAME_BUFFER;
		}
		if (param->forceIPicture != 0 && param->forceIPicture != 1) {
			return RETCODE_INVALID_PARAM;
		}
	}
	if (pCodecInst->CodecInfo.encInfo.openParam.bitRate == 0) { // no rate control
		if (pCodecInst->codecMode == MP4_ENC) {
			if (param->quantParam < 1 || param->quantParam > 31) {
				return RETCODE_INVALID_PARAM;
			}
		}
		else { // AVC_ENC
			if (param->quantParam < 0 || param->quantParam > 51) {
				return RETCODE_INVALID_PARAM;
			}
		}
	}
	return RETCODE_SUCCESS;
}

void EncodeHeader(EncHandle handle, EncHeaderParam * encHeaderParam)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	phyaddr_t rdPtr;
	phyaddr_t wrPtr;		

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if( pEncInfo->dynamicAllocEnable == 1 ) {
		VpuWriteReg( CMD_ENC_HEADER_BB_START, encHeaderParam->buf );
		VpuWriteReg( CMD_ENC_HEADER_BB_SIZE, encHeaderParam->size );
	}
	
	VpuWriteReg(CMD_ENC_HEADER_CODE, encHeaderParam->headerType); // 0: SPS, 1: PPS
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, ENCODE_HEADER);
	while (VpuReadReg(BIT_BUSY_FLAG))
		;

	if( pEncInfo->dynamicAllocEnable == 1 ) {
		rdPtr = VpuReadReg( CMD_ENC_HEADER_BB_START );
		wrPtr = VpuReadReg( pEncInfo->streamWrPtrRegAddr);
		encHeaderParam->buf = rdPtr;
		encHeaderParam->size = wrPtr - rdPtr;
	}
	else {
		rdPtr = VpuReadReg(pEncInfo->streamRdPtrRegAddr);
		wrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);	
		encHeaderParam->buf = rdPtr;
		encHeaderParam->size = wrPtr - rdPtr;
	}	
	pEncInfo->streamWrPtr = wrPtr;
	pEncInfo->streamRdPtr = rdPtr;

}

void GetParaSet(EncHandle handle, int paraSetType, EncParamSet * para)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	
	VpuWriteReg(pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	VpuWriteReg(pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(BIT_BIT_STREAM_PARAM, pEncInfo->streamEndflag);

	VpuWriteReg(CMD_ENC_PARA_SET_TYPE, paraSetType); // SPS: 0, PPS: 1, VOS: 1, VO: 2, VOL: 0
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, ENC_PARA_SET);
	while (VpuReadReg(BIT_BUSY_FLAG)) ;

	para->paraSet = paraBuffer;
	para->size = VpuReadReg(RET_ENC_PARA_SET_SIZE);

	pEncInfo->streamRdPtr = VpuReadReg(pEncInfo->streamRdPtrRegAddr);
	pEncInfo->streamWrPtr = VpuReadReg(pEncInfo->streamWrPtrRegAddr);
	pEncInfo->streamEndflag = VpuReadReg(BIT_BIT_STREAM_PARAM);

}

RetCode SetGopNumber(EncHandle handle, uint32_t *pGopNumber)
{
	CodecInst * pCodecInst;
	int data =0;
	uint32_t gopNumber = *pGopNumber;

	pCodecInst = handle;
	data = 1;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_RC_GOP, gopNumber);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));
	return RETCODE_SUCCESS;
}

RetCode SetIntraQp(EncHandle handle, uint32_t *pIntraQp)
{
	CodecInst * pCodecInst;
	int data =0;
	uint32_t intraQp = *pIntraQp;

	pCodecInst = handle;
	data = 1<<1;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_RC_INTRA_QP, intraQp);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));
	return RETCODE_SUCCESS;
}

RetCode SetBitrate(EncHandle handle, uint32_t *pBitrate)
{
	CodecInst * pCodecInst;
	int data =0;
	uint32_t bitrate = *pBitrate;

	pCodecInst = handle;
	data = 1<<2;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_RC_BITRATE, bitrate);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));
	return RETCODE_SUCCESS;
}

RetCode SetFramerate(EncHandle handle, uint32_t *pFramerate)
{
	CodecInst * pCodecInst;
	int data =0;
	uint32_t framerate = *pFramerate;

	pCodecInst = handle;
	data = 1<<3;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_RC_FRAME_RATE, framerate);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));

	return RETCODE_SUCCESS;
}

RetCode SetIntraRefreshNum(EncHandle handle, uint32_t *pIntraRefreshNum)
{
	CodecInst * pCodecInst;
	uint32_t intraRefreshNum = *pIntraRefreshNum;
	int data = 0;
	pCodecInst = handle;
	data = 1<<4;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_INTRA_MB_NUM, intraRefreshNum);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));

	return RETCODE_SUCCESS;
}

RetCode SetSliceMode(EncHandle handle, EncSliceMode *pSliceMode)
{
	CodecInst * pCodecInst;
	uint32_t data = 0;
	int data2 = 0;
	data = pSliceMode->sliceSize<<2 | pSliceMode->sliceSizeMode<<1 | pSliceMode->sliceMode;
	pCodecInst = handle;

	data2 = 1<<5;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data2);
	VpuWriteReg(CMD_ENC_SEQ_PARA_SLICE_MODE, data);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));

	return RETCODE_SUCCESS;
}

RetCode SetHecMode(EncHandle handle, int mode)
{
	CodecInst * pCodecInst;
	uint32_t HecMode = mode;
	int data = 0;
	pCodecInst = handle;
	data = 1 << 6;
	VpuWriteReg(CMD_ENC_SEQ_PARA_CHANGE_ENABLE, data);
	VpuWriteReg(CMD_ENC_SEQ_PARA_HEC_MODE, HecMode);
	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, RC_CHANGE_PARAMETER);
	while (VpuReadReg(BIT_BUSY_FLAG));

	return RETCODE_SUCCESS;
}


RetCode CheckDecInstanceValidity(DecHandle handle)
{
	CodecInst * pCodecInst;
	RetCode ret;

	pCodecInst = handle;
	ret = CheckInstanceValidity(pCodecInst);
	if (ret != RETCODE_SUCCESS) {
		return RETCODE_INVALID_HANDLE;
	}
	if (!pCodecInst->inUse) {
		return RETCODE_INVALID_HANDLE;
	}
	if (pCodecInst->codecMode != AVC_DEC && 
		pCodecInst->codecMode != VC1_DEC &&
		pCodecInst->codecMode != MP2_DEC &&
		pCodecInst->codecMode != MP4_DEC &&
		pCodecInst->codecMode != DV3_DEC &&
		pCodecInst->codecMode != RV_DEC &&
		pCodecInst->codecMode != AVS_DEC &&
        pCodecInst->codecMode != MJPG_DEC) {
		return RETCODE_INVALID_HANDLE;
	}

	return RETCODE_SUCCESS;
}

RetCode CheckDecOpenParam(DecOpenParam * pop)
{
	if (pop == 0) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitstreamBuffer % 4) { // not 4-bit aligned
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitstreamBufferSize % 1024 ||
			pop->bitstreamBufferSize < 1024 ||
			pop->bitstreamBufferSize > (16383 * 1024) ) {
		return RETCODE_INVALID_PARAM;
	}
	if (pop->bitstreamFormat != STD_AVC
			&& pop->bitstreamFormat != STD_VC1
			&& pop->bitstreamFormat != STD_MPEG2
			&& pop->bitstreamFormat != STD_MPEG4
			&& pop->bitstreamFormat != STD_DIV3
			&& pop->bitstreamFormat != STD_RV
            && pop->bitstreamFormat != STD_AVS
            && pop->bitstreamFormat != STD_MJPG
	) {
		return RETCODE_INVALID_PARAM;
	}
	
	if( pop->mp4DeblkEnable == 1 && !(pop->bitstreamFormat == STD_MPEG4 || pop->bitstreamFormat == STD_MPEG2 || pop->bitstreamFormat == STD_DIV3)) {
		return RETCODE_INVALID_PARAM;
	}
	
	return RETCODE_SUCCESS;
}


int DecBitstreamBufEmpty(DecInfo * pDecInfo)
{
	return ( VpuReadReg(pDecInfo->streamRdPtrRegAddr) == VpuReadReg(pDecInfo->streamWrPtrRegAddr) );
}

void SetParaSet(DecHandle handle, int paraSetType, DecParamSet * para)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	int i;
	uint32_t * src;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	src = para->paraSet;
	for (i = 0; i < para->size; i += 4) {
		VpuWriteReg(paraBuffer + i, *src++);
	}
	VpuWriteReg(CMD_DEC_PARA_SET_TYPE, paraSetType); // 0: SPS, 1: PPS
	VpuWriteReg(CMD_DEC_PARA_SET_SIZE, para->size);

	if (pDecInfo->openParam.bitstreamFormat == STD_DIV3)
		VpuWriteReg(BIT_RUN_AUX_STD, 1);
	else
		VpuWriteReg(BIT_RUN_AUX_STD, 0);

	BitIssueCommand(pCodecInst->instIndex, pCodecInst->codecMode, DEC_PARA_SET);
	while (VpuReadReg(BIT_BUSY_FLAG));
}
