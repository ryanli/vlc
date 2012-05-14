//------------------------------------------------------------------------------
// File: VpuApiFunc.h
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------


#ifndef VPUAPI_UTIL_H_INCLUDED
#define VPUAPI_UTIL_H_INCLUDED

#include "VpuApi.h"
#include "RegDefine.h"



#define MAX_NUM_INSTANCE 16

enum {
	AVC_DEC = 0,
	VC1_DEC = 1,
	MP2_DEC = 2,
	MP4_DEC = 3,
	DV3_DEC = 3,
	RV_DEC = 4,
    AVS_DEC = 5,
    MJPG_DEC = 6,
	AVC_ENC = 8,
	MP4_ENC = 11,
	MJPG_ENC = 14
};

enum {
	SEQ_INIT = 1,
	SEQ_END = 2,
	PIC_RUN = 3,
	SET_FRAME_BUF = 4,
	ENCODE_HEADER = 5,
	ENC_PARA_SET = 6,
	DEC_PARA_SET = 7,
	DEC_BUF_FLUSH = 8,
	RC_CHANGE_PARAMETER	= 9,
	VPU_SLEEP	= 10,
	VPU_WAKE	= 11,
	FIRMWARE_GET = 0xf
};

#define MAX_ENC_PIC_WIDTH	1280 //720
#define MAX_ENC_PIC_HEIGHT	720  //576
#define MAX_ENC_MJPG_PIC_WIDTH		8192
#define MAX_ENC_MJPG_PIC_HEIGHT	8192

typedef struct {
	EncOpenParam openParam;
	EncInitialInfo initialInfo;
	phyaddr_t streamRdPtr;
	phyaddr_t streamWrPtr;
	phyaddr_t streamRdPtrRegAddr;
	phyaddr_t streamWrPtrRegAddr;
	phyaddr_t streamBufStartAddr;
	phyaddr_t streamBufEndAddr;
	int	streamEndflag;
	int streamBufSize;
	FrameBuffer * frameBufPool;
	int numFrameBuffers;
	int stride;
	int rotationEnable;
	int mirrorEnable;
	MirrorDirection mirrorDirection;
	int rotationAngle;
	int initialInfoObtained;
	int dynamicAllocEnable;
	int ringBufferEnable;
	SecAxiUse secAxiUse;
#ifdef API_CR
	int enReportMBInfo;
	int enReportMVInfo;
	int enReportSliceInfo;
	phyaddr_t picParaBaseAddr;
	phyaddr_t picMbInfoAddr;
	phyaddr_t picMvInfoAddr;
	phyaddr_t picSliceInfoAddr;
#endif
#ifdef JPEG_PARTIAL
	int jpgPartialEncode;
#endif // JPEG_PARTIAL
} EncInfo;

typedef struct {
	DecOpenParam openParam;
	DecInitialInfo initialInfo;
	
	phyaddr_t streamWrPtr;
	phyaddr_t streamRdPtr;
	phyaddr_t streamRdPtrRegAddr;
	phyaddr_t streamWrPtrRegAddr;
	phyaddr_t streamBufStartAddr;
	phyaddr_t streamBufEndAddr;
	phyaddr_t	frameDisplayFlagRegAddr;
	phyaddr_t	frameDisplayFlag;
	int	streamEndflag;

	int streamBufSize;
	FrameBuffer * frameBufPool;
	int numFrameBuffers;
	FrameBuffer * recFrame;
	int stride;
	int rotationEnable;
	int mirrorEnable;
	int	deringEnable;
	MirrorDirection mirrorDirection;
	int rotationAngle;
	FrameBuffer rotatorOutput;
	int rotatorStride;
	int rotatorOutputValid;	
	int initialInfoObtained;
	int filePlayEnable;
	int picSrcSize;
	int	dynamicAllocEnable;
	int	vc1BframeDisplayValid;
	SecAxiUse secAxiUse;
#ifdef	API_CR
	phyaddr_t picParaBaseAddr;
	phyaddr_t userDataBufAddr;

	phyaddr_t frameBufStateAddr;
	phyaddr_t mbParamDataAddr;
	phyaddr_t mvDataAddr;

#ifdef JPEG_PARTIAL
	int jpgPartialDecode;
#endif // JPEG_PARTIAL
	int frameBufStatEnable;				// Frame Buffer Status
	int mbParamEnable;					// Mb Param for Error Concealment
	int mvReportEnable;					// Motion vector
	int userDataEnable;					// User Data
	int userDataBufSize;
	int userDataReportMode;				// User Data report mode (0 : interrupt mode, 1 interrupt disable mode)
#endif
} DecInfo;


typedef struct CodecInst {
	int instIndex;
	int inUse;
	int codecMode;
	union {
		EncInfo encInfo;
		DecInfo decInfo;
	} CodecInfo;
} CodecInst;


#ifdef __cplusplus
extern "C" {
#endif	
	void	BitIssueCommand(int instIdx, int cdcMode, int cmd);

	RetCode GetCodecInstance(CodecInst ** ppInst);
	void	FreeCodecInstance(CodecInst * pCodecInst);
	RetCode CheckInstanceValidity(CodecInst * pci);

	RetCode CheckEncInstanceValidity(EncHandle handle);
	RetCode CheckEncOpenParam(EncOpenParam * pop);
	RetCode CheckEncParam(CodecInst * pCodecInst, EncParam * param);
	void	EncodeHeader(EncHandle handle, EncHeaderParam * encHeaderParam);
	void	GetParaSet(EncHandle handle, int paraSetType, EncParamSet * para);
	RetCode SetGopNumber(EncHandle handle, uint32_t *gopNumber);
	RetCode SetIntraQp(EncHandle handle, uint32_t *intraQp);
	RetCode SetBitrate(EncHandle handle, uint32_t *bitrate);
	RetCode SetFramerate(EncHandle handle, uint32_t *framerate);
	RetCode SetIntraRefreshNum(EncHandle handle, uint32_t *pIntraRefreshNum);
	RetCode SetSliceMode(EncHandle handle, EncSliceMode *pSliceMode);
	RetCode SetHecMode(EncHandle handle, int mode);

	RetCode CheckDecInstanceValidity(DecHandle handle);
	RetCode CheckDecOpenParam(DecOpenParam * pop);
	int		DecBitstreamBufEmpty(DecInfo * pDecInfo);
	void	SetParaSet(DecHandle handle, int paraSetType, DecParamSet * para);	
	
	
#ifdef __cplusplus
}
#endif

#endif // endif VPUAPI_UTIL_H_INCLUDED
