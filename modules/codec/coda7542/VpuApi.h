//------------------------------------------------------------------------------
// File: VpuApi.h
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------


#ifndef VPUAPI_H_INCLUDED
#define VPUAPI_H_INCLUDED


#include <stdio.h>
#include <stdint.h>

extern volatile unsigned char * vpu_bit_base;
#ifdef REG_LOG
extern FILE* reg_log;
#endif


int ReadReg(unsigned int ADDR);
void WriteReg(unsigned int ADDR, int DATA);

#define VpuWriteReg( ADDR, DATA )	WriteReg( ADDR, DATA ) // system register write
#define VpuReadReg( ADDR )			ReadReg( ADDR )		   // system register write

//#define JPEG_PARTIAL
#define API_CR
#define	PIC_ASPECT_F_RATE

typedef uint32_t phyaddr_t;

// FIXME: ybfqing for g_mem_base
// FIXME: modified all physical address to offset.hefang 20101022
#define ADDR_PHY_MEM_BASE 	0x00000000

#define	ADDR_BIT_STREAM			0x00000000
#define	LARGE_STREAM_BUF_SIZE	0x00100000
#define	SMALL_STREAM_BUF_SIZE	0x00100000
	
#define	ADDR_BIT_STREAM_1		0x00000000
#define	ADDR_BIT_STREAM_2		0x00010000
#define	ADDR_BIT_STREAM_3		0x00020000
#define	ADDR_BIT_STREAM_4		0x00030000
#define	ADDR_BIT_STREAM_5		0x00040000
#define	ADDR_BIT_STREAM_6		0x00050000
#define	ADDR_BIT_STREAM_7		0x00060000
#define	ADDR_BIT_STREAM_8		0x00070000
#define	ADDR_BIT_STREAM_9		0x00080000
#define	ADDR_BIT_STREAM_10		0x00090000
#define	ADDR_BIT_STREAM_11		0x000A0000
#define	ADDR_BIT_STREAM_12		0x000B0000
#define	ADDR_BIT_STREAM_13		0x000C0000
#define	ADDR_BIT_STREAM_14		0x000D0000
#define	ADDR_BIT_STREAM_15		0x000E0000
#define	ADDR_BIT_STREAM_16		0x000F0000

#define	STREAM_BUF_SIZE_1		0x00010000
#define	STREAM_BUF_SIZE_2		0x00010000
#define	STREAM_BUF_SIZE_3		0x00010000
#define	STREAM_BUF_SIZE_4		0x00010000
#define	STREAM_BUF_SIZE_5		0x00010000
#define	STREAM_BUF_SIZE_6		0x00010000
#define	STREAM_BUF_SIZE_7		0x00010000
#define	STREAM_BUF_SIZE_8		0x00010000
#define	STREAM_BUF_SIZE_9		0x00010000
#define	STREAM_BUF_SIZE_10	0x00010000
#define	STREAM_BUF_SIZE_11	0x00010000
#define	STREAM_BUF_SIZE_12	0x00010000
#define	STREAM_BUF_SIZE_13	0x00010000
#define	STREAM_BUF_SIZE_14	0x00010000
#define	STREAM_BUF_SIZE_15	0x00010000
#define	STREAM_BUF_SIZE_16	0x00010000

#define	ADDR_SLICE_SAVE_BUFFER	0x00100000
#define	SLICE_SAVE_SIZE			0x00080000

#define	ADDR_PS_SAVE_BUFFER		0x00180000
#define PS_SAVE_SIZE			  0x000C0000

#define	ADDR_PS_SAVE_BUFFER_1	0x00180000
#define	ADDR_PS_SAVE_BUFFER_2	0x001A0000
#define	ADDR_PS_SAVE_BUFFER_3	0x001C0000
#define	ADDR_PS_SAVE_BUFFER_4	0x001E0000
#define PS_SAVE_SIZE_1			0x00001400
#define PS_SAVE_SIZE_2			0x00001400
#define PS_SAVE_SIZE_3			0x00001400
#define PS_SAVE_SIZE_4			0x00001400
	
// SRAM Total Size = 0x20000
#define	DBKY_INTERNAL_USE_BUF		0x000000	// DBKY_SIZE = MAX_WIDTH*16
#define	DBKC_INTERNAL_USE_BUF		0x008000	// DBKC_SIZE = MAX_WIDTH*16
#define	BIT_INTERNAL_USE_BUF		0x010000	// BIT_SIZE  = MAX_WIDTH/16*128
#define IPACDC_INTERNAL_USE_BUF		0x016000	// IP_SIZE   = MAX_WIDTH/16*128
#define	OVL_INTERNAL_USE_BUF		0x01A000	// OVL_SIZE  = MAX_WIDTH*5
#define	BTP_INTERNAL_USE_BUF		0x01F000	// BTP_SIZE  = {(MAX_WIDTH/256)*(MAX_HEIGHT/16) + 1}*6

#define	ADDR_BIT_WORK			0x00240000
#define ADDR_FRAME_BASE		0x00380000

#define MAX_FRAME_BASE	  (0x00380000 + 0x2000000)   // 32 MB
#ifdef JPEG_PARTIAL
#define FRAME_BUF_SIZE			MAX_FRAME_BASE - ADDR_FRAME_BASE
#endif // JPEG_PARTIAL

#ifdef API_CR
	#define SIZE_PARA_BUF					0x100
	#define	SIZE_USER_BUF					0x10000
	#define	SIZE_MV_DATA					0x20000
	#define	SIZE_MB_DATA					0x4000
	#define	SIZE_FRAME_BUF_STAT				0x100
	#define	USER_DATA_INFO_OFFSET			8*17
	#define	ADDR_PIC_PARA_BASE_ADDR			ADDR_FRAME_BASE + 1920*1088*3/2*7
	#define	ADDR_USER_BASE_ADDR				ADDR_PIC_PARA_BASE_ADDR 		+ SIZE_PARA_BUF
	#define	ADDR_MV_BASE_ADDR				ADDR_USER_BASE_ADDR				+ SIZE_USER_BUF
	#define	ADDR_MB_BASE_ADDR				ADDR_MV_BASE_ADDR				+ SIZE_MV_DATA
	#define	ADDR_FRAME_BUF_STAT_BASE_ADDR	ADDR_MB_BASE_ADDR				+ SIZE_MB_DATA
	#define ADDR_END_OF_RPT_BUF				ADDR_FRAME_BUF_STAT_BASE_ADDR 	+ SIZE_FRAME_BUF_STAT
	//#define	SIZE_USER_BUF				USER_DATA_INFO_OFFSET+32
#endif

#define CODE_BUF_SIZE			( 160 * 1024 )
#define FMO_SLICE_SAVE_BUF_SIZE	( 64 )
#define WORK_BUF_SIZE			( 160 * 1024 ) + (MAX_NUM_INSTANCE*41*1024) // 160K(tempbuff) + index*41K(comom+static)

#define PARA_BUF2_SIZE			( 2 * 1024 )
#define PARA_BUF_SIZE			( 10 * 1024 )

#define IMAGE_ENDIAN			0		// 0 (64 bit little endian), 1 (64 bit big endian), 2 (32 bit swaped little endian), 3 (32 bit swaped big endian)
#define STREAM_ENDIAN			0       // 0 (64 bit little endian), 1 (64 bit big endian), 2 (32 bit swaped little endian), 3 (32 bit swaped big endian)
#define	CBCR_INTERLEAVE			0		// 0 (chroma seperate mode), 1 (chroma interleave mode)

#define	USE_BIT_INTERNAL_BUF	0
#define USE_IP_INTERNAL_BUF		0
#define	USE_DBKY_INTERNAL_BUF	0
#define	USE_DBKC_INTERNAL_BUF	0
#define	USE_OVL_INTERNAL_BUF	0
#define	USE_BTP_INTERNAL_BUF	0
#define	USE_ME_INTERNAL_BUF		0

#define USE_HOST_BIT_INTERNAL_BUF	0
#define USE_HOST_IP_INTERNAL_BUF	0
#define USE_HOST_DBKY_INTERNAL_BUF	0
#define USE_HOST_DBKC_INTERNAL_BUF	0
#define	USE_HOST_OVL_INTERNAL_BUF	0
#define	USE_HOST_BTP_INTERNAL_BUF	0
#define USE_HOST_ME_INTERNAL_BUF	0

//typedef unsigned _int64 UI64;
typedef unsigned long long UI64;
typedef unsigned int    UINT;
typedef unsigned char   BYTE;

#define DEFAULT_SEARCHRAM_ADDR  0x002C0000
#define STREAM_FULL_EMPTY_CHECK_DISABLE 0

//#define REPORT_HEADER_INFO
//------------------------------------------------------------------------------
// common struct and definition
//------------------------------------------------------------------------------
typedef enum {
	INT_INIT			= 0,
	INT_SEQ_INIT		= 1,
	INT_SEQ_END			= 2,
	INT_PIC_RUN			= 3,
	INT_SET_FRAME		= 4,
#ifdef API_CR
	INT_BIT_USERDATA	= 9,
#endif // API_CR
#ifdef JPEG_PARTIAL
	INT_BIT_JPG_PARTIAL = 11,
#endif // JPEG_PARTIAL
} IntBitAlign;

typedef enum {
	STD_AVC,
	STD_VC1,
	STD_MPEG2,
	STD_MPEG4,
	STD_H263,
	STD_DIV3,
	STD_RV,
  STD_AVS,
  STD_MJPG
} CodStd;

typedef enum {
	RETCODE_SUCCESS,
	RETCODE_FAILURE,
	RETCODE_INVALID_HANDLE,
	RETCODE_INVALID_PARAM,
	RETCODE_INVALID_COMMAND,
	RETCODE_ROTATOR_OUTPUT_NOT_SET,
	RETCODE_ROTATOR_STRIDE_NOT_SET,
	RETCODE_FRAME_NOT_COMPLETE,
	RETCODE_INVALID_FRAME_BUFFER,
	RETCODE_INSUFFICIENT_FRAME_BUFFERS,
	RETCODE_INVALID_STRIDE,
	RETCODE_WRONG_CALL_SEQUENCE,
	RETCODE_CALLED_BEFORE,
	RETCODE_NOT_INITIALIZED,
	RETCODE_USERDATA_BUF_NOT_SET
} RetCode;

	phyaddr_t picParaBaseAddr;
	phyaddr_t userDataBufAddr;
	int userDataBufSize;

typedef enum {
	ENABLE_ROTATION,
	DISABLE_ROTATION,
	ENABLE_MIRRORING,
	DISABLE_MIRRORING,
	SET_MIRROR_DIRECTION,
	SET_ROTATION_ANGLE,
	SET_ROTATOR_OUTPUT,
	SET_ROTATOR_STRIDE,
	DEC_SET_SPS_RBSP,
	DEC_SET_PPS_RBSP,
	ENABLE_DERING,
	DISABLE_DERING,
	SET_SEC_AXI,
	ENABLE_REP_BUFSTAT,
	DISABLE_REP_BUFSTAT,
	ENABLE_REP_MBPARAM,
	DISABLE_REP_MBPARAM,
	ENABLE_REP_MV,
	DISABLE_REP_MV,
#ifdef JPEG_PARTIAL
	ENABLE_JPG_PARTIAL,
	DISABLE_JPG_PARTIAL,
#endif // JPEG_PARTIAL
	ENABLE_REP_USERDATA,
	DISABLE_REP_USERDATA,
	SET_ADDR_REP_PICPARAM,
	SET_ADDR_REP_USERDATA,
	SET_SIZE_REP_USERDATA,
	SET_USERDATA_REPORT_MODE,
	SET_ADDR_REP_BUF_STATE,
	SET_ADDR_REP_MB_PARAM,
	SET_ADDR_REP_MV_DATA,
	ENC_GET_SPS_RBSP,
	ENC_GET_PPS_RBSP,
	ENC_PUT_MP4_HEADER,
	ENC_PUT_AVC_HEADER,
	ENC_SET_SEARCHRAM_PARAM,
	ENC_GET_VOS_HEADER,
	ENC_GET_VO_HEADER,
	ENC_GET_VOL_HEADER,
	ENC_SET_INTRA_MB_REFRESH_NUMBER,
	ENC_ENABLE_HEC,
	ENC_DISABLE_HEC,
	ENC_SET_SLICE_INFO,
	ENC_SET_GOP_NUMBER,
	ENC_SET_INTRA_QP,
	ENC_SET_BITRATE,
	ENC_SET_FRAME_RATE,
	ENC_SET_REPORT_MBINFO,
	ENC_SET_REPORT_MVINFO,
	ENC_SET_REPORT_SLICEINFO,
	ENC_SET_PIC_PARA_ADDR
} CodecCommand;

typedef struct {
	phyaddr_t bufY;
	phyaddr_t bufCb;
	phyaddr_t bufCr;
	phyaddr_t bufMvCol;
} FrameBuffer;

typedef struct
{
    uint32_t    left;
    uint32_t    top;
    uint32_t    right;
    uint32_t    bottom;
} Rect;

typedef enum {
	MIRDIR_NONE,
	MIRDIR_VER,
	MIRDIR_HOR,
	MIRDIR_HOR_VER
} MirrorDirection;

typedef	struct
{
	int useBitEnable;
	int useIpEnable;
	int useDbkYEnable;
	int useDbkCEnable;
	int useOvlEnable;
	int useBtpEnable;
	int useMeEnable;

	int useHostBitEnable;
	int useHostIpEnable;
	int useHostDbkYEnable;
	int useHostDbkCEnable;
	int useHostOvlEnable;
	int useHostBtpEnable;
	int useHostMeEnable;
	
	phyaddr_t bufBitUse;
	phyaddr_t bufIpAcDcUse;
	phyaddr_t bufDbkYUse;
	phyaddr_t bufDbkCUse;
	phyaddr_t bufOvlUse;
	phyaddr_t bufBtpUse;
} SecAxiUse;

struct CodecInst;

//------------------------------------------------------------------------------
// decode struct and definition
//------------------------------------------------------------------------------

typedef struct CodecInst DecInst;
typedef DecInst * DecHandle;

typedef struct {
	CodStd bitstreamFormat;
	phyaddr_t bitstreamBuffer;
	int bitstreamBufferSize;
	int mp4DeblkEnable;
	int reorderEnable;
	int filePlayEnable;
	int picWidth;
	int picHeight;
	int	dynamicAllocEnable;
	int	streamStartByteOffset;
	int mjpg_thumbNailDecEnable;
	phyaddr_t psSaveBuffer;
	int  psSaveBufferSize;
	int mp4Class;
} DecOpenParam;

typedef struct {
	int picWidth;			// {(PicX+15)/16} * 16
	int picHeight;			// {(PicY+15)/16} * 16
	Rect picCropRect;
	int mp4_dataPartitionEnable;
	int mp4_reversibleVlcEnable;
	int mp4_shortVideoHeader;
	int h263_annexJEnable;
	int minFrameBufferCount;
	int frameBufDelay;
	int	nextDecodedIdxNum;
	int	normalSliceSize;
	int	worstSliceSize;
	int	mjpg_thumbNailEnable;
	int	mjpg_sourceFormat;
#ifdef API_CR
    int profile;
    int level;
    int interlace;
	int constraint_set_flag[4];
	int direct8x8Flag;
    int vc1_psf;
	int aspectRateInfo;
#endif
#ifdef JPEG_PARTIAL
	int jpg_mb_size;
	int jpg_partial_framebuffer;
#endif // JPEG_PARTIAL
	int fRateNumerator; 
	int fRateDenominator;
} DecInitialInfo;

typedef	struct{
	phyaddr_t sliceSaveBuffer;
	int  sliceSaveBufferSize;
} DecAvcSliceBufInfo;

typedef	struct{
	DecAvcSliceBufInfo avcSliceBufInfo;
} DecBufInfo;

#ifdef	API_CR

typedef enum {
		PARA_TYPE_FRM_BUF_STATUS	= 1,
		PARA_TYPE_MB_PARA		= 2,
		PARA_TYPE_MV			= 4,
} ExtParaType;

#endif

typedef struct {
	int prescanEnable;
	int prescanMode;	
	int dispReorderBuf;
	int iframeSearchEnable;
	int skipframeMode;
	int skipframeNum;
	int chunkSize;
	int	picStartByteOffset;
#ifdef JPEG_PARTIAL
	int jpgPartialMbNum;
#endif // JPEG_PARTIAL
	phyaddr_t picStreamBufferAddr;
} DecParam;

#ifdef	API_CR
typedef struct {
	int frameBufDataSize;			// Frame Buffer Status
	phyaddr_t frameBufStatDataAddr;		
	
	int mbParamDataSize;			// Mb Param for Error Concealment
	phyaddr_t mbParamDataAddr;
	
	int mvDataSize;				// Motion vector
	int mvNumPerMb;
	phyaddr_t mvDataAddr;
		
	int userDataNum;			// User Data
	int userDataSize;
	int userDataBufFull;

} DecOutputExtData;
#endif

typedef struct {
	int indexFrameDisplay;
	int indexFrameDecoded;
	int picType;
	int picTypeFirst;
	int numOfErrMBs;
	int hScaleFlag;			
	int vScaleFlag;
	int prescanresult;
	int	notSufficientPsBuffer;
	int	notSufficientSliceBuffer;
	int	decodingSuccess;
	int interlacedFrame;
	int mp4PackedPBframe;
	int h264PicStruct;
	

#ifdef API_CR
    int pictureStructure;
    int topFieldFirst;
    int repeatFirstField;
    int progressiveFrame;
    int fieldSequence;
	DecOutputExtData decOutputExtData;
#endif
	int packedMode;

	int decPicHeight;
	int decPicWidth;

	Rect decPicCrop; 
	int	refFlag;
#ifdef PIC_ASPECT_F_RATE
	int picAspectRatio;
	int picFrateNr;
	int	picFrateDr;
#endif
} DecOutputInfo;

typedef struct {
	uint32_t * paraSet;
	int size;
} DecParamSet;

//------------------------------------------------------------------------------
// encode struct and definition
//------------------------------------------------------------------------------

typedef struct CodecInst EncInst;
typedef EncInst * EncHandle;

typedef struct {
	int mp4_dataPartitionEnable;
	int mp4_reversibleVlcEnable;
	int mp4_intraDcVlcThr;
	int	mp4_hecEnable;
	int mp4_verid;
} EncMp4Param;

typedef struct {
	int h263_annexJEnable;
	int h263_annexKEnable;
	int h263_annexTEnable;
} EncH263Param;

typedef struct {
	int avc_constrainedIntraPredFlag;
	int avc_disableDeblk;
	int avc_deblkFilterOffsetAlpha;
	int avc_deblkFilterOffsetBeta;
	int avc_chromaQpOffset;
	int avc_audEnable;
	int avc_fmoEnable;
	int avc_fmoSliceNum;
	int avc_fmoType;
	int	avc_fmoSliceSaveBufSize;
} EncAvcParam;

typedef struct {
	int mjpg_sourceFormat;
	int mjpg_restartInterval;
	int mjpg_thumbNailEnable;
	int mjpg_thumbNailWidth;
	int mjpg_thumbNailHeight;
	uint8_t * mjpg_hufTable;
	uint8_t * mjpg_qMatTable;
} EncMjpgParam;

typedef struct{
	int sliceMode;
	int sliceSizeMode;
	int sliceSize;
} EncSliceMode;

typedef struct {
	phyaddr_t bitstreamBuffer;
	uint32_t bitstreamBufferSize;
	CodStd bitstreamFormat;
	
	int picWidth;
	int picHeight;
	int bitRate;
	int initialDelay;
	int vbvBufferSize;
	int enableAutoSkip;
	int gopSize;

	EncSliceMode slicemode;
	
	int intraRefresh;
	
	int rcIntraQp;	
	int dynamicAllocEnable;
	int ringBufferEnable;
	union {
		EncMp4Param mp4Param;
		EncH263Param h263Param;
		EncAvcParam avcParam;
		EncMjpgParam mjpgParam;		
	} EncStdParam;
	int userQpMax;
	uint32_t userGamma;
	int avcIntra16x16OnlyModeEnable;
	int RcIntervalMode;		// 0:normal, 1:frame_level, 2:slice_level, 3: user defined Mb_level
	int MbInterval;			// use when RcintervalMode is 3
	int frameRateDr;
	int frameRateNr;
} EncOpenParam;

typedef struct {
	int minFrameBufferCount;
#ifdef JPEG_PARTIAL
	int jpg_mb_size;
	int jpg_partial_framebuffer;
#endif // JPEG_PARTIAL
} EncInitialInfo;

typedef struct {
	FrameBuffer * sourceFrame;
	int forceIPicture;
	int skipPicture;
	int quantParam;
	phyaddr_t picStreamBufferAddr;
	int	picStreamBufferSize;
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
	int jpgPartialMbNum;
	int jpgPartialVertSize;
#endif // JPEG_PARTIAL
} EncParam;

#ifdef	API_CR
typedef	struct {
	int	enable;
	int	type;
	int	sz;
	phyaddr_t	addr;
} EncReportInfo;
#endif

typedef struct {
	phyaddr_t bitstreamBuffer;
	uint32_t bitstreamSize;
	int bitstreamWrapAround; 
	int picType;
	int numOfSlices;

#ifdef API_CR
	EncReportInfo MbInfo;
	EncReportInfo MvInfo;
	EncReportInfo SliceInfo;
#endif
} EncOutputInfo;


typedef struct {
	phyaddr_t paraSet;
	int size;
} EncParamSet;

typedef struct {
	phyaddr_t searchRamAddr;
	int SearchRamSize;
} SearchRamParam;

typedef struct {
	phyaddr_t buf;
	int size;
	int headerType;
} EncHeaderParam;

typedef enum {
	VOL_HEADER,
	VOS_HEADER,
	VIS_HEADER
} Mp4HeaderType;

typedef enum {
	SPS_RBSP,
	PPS_RBSP
} AvcHeaderType;	

typedef struct {
	uint32_t gopNumber;
	uint32_t intraQp;
	uint32_t bitrate;
	uint32_t framerate;
} stChangeRcPara;

#ifdef __cplusplus
extern "C" {
#endif

	int		VPU_IsBusy(void);
	RetCode VPU_Init(phyaddr_t workBuf);
	RetCode VPU_GetVersionInfo( uint32_t *versionInfo );

	// function for encode
	RetCode VPU_EncOpen(EncHandle *, EncOpenParam *);
	RetCode VPU_EncClose(EncHandle);
	RetCode VPU_EncGetInitialInfo(EncHandle, EncInitialInfo *);
	RetCode VPU_EncRegisterFrameBuffer(
		EncHandle handle,
		FrameBuffer * bufArray,
		int num,
		int stride);
	RetCode VPU_EncGetBitstreamBuffer( 
		EncHandle handle,
		phyaddr_t * prdPrt,
		phyaddr_t * pwrPtr,
		uint32_t * size);
	RetCode VPU_EncUpdateBitstreamBuffer(
		EncHandle handle,
		uint32_t size);
	RetCode VPU_EncStartOneFrame(
		EncHandle handle,
		EncParam * param );
	RetCode VPU_EncGetOutputInfo(
		EncHandle handle,
		EncOutputInfo * info);
	RetCode VPU_EncGiveCommand(
		EncHandle handle,
		CodecCommand cmd,
		void * parameter);
	void Vpu_EncSetHostParaAddr(
		phyaddr_t baseAddr,
		phyaddr_t paraAddr);

	// function for decode
	RetCode VPU_SleepWake(int iSleepWake);
	RetCode VPU_DecOpen(DecHandle *, DecOpenParam *);
	RetCode VPU_DecClose(DecHandle);
	RetCode VPU_DecSetEscSeqInit( 
		DecHandle handle, 
		int escape );
	RetCode VPU_DecGetInitialInfo(
		DecHandle handle,
		DecInitialInfo * info);
	RetCode VPU_DecRegisterFrameBuffer(
		DecHandle handle,
		FrameBuffer * bufArray,
		int num,
		int stride,
		DecBufInfo * pBufInfo);
	RetCode VPU_DecGetBitstreamBuffer(
		DecHandle handle,
		phyaddr_t * prdPrt,
		phyaddr_t * pwrPtr,
		uint32_t * size );
	RetCode VPU_DecUpdateBitstreamBuffer(
		DecHandle handle,
		uint32_t size);
	RetCode VPU_DecStartOneFrame( 
		DecHandle handle, 
		DecParam *param ); 
#ifdef	JPEG_PARTIAL
	RetCode VPU_DecSetBasesJpgPartial(
		DecHandle handle,
		int jpg_partial_size, 
		int jpg_partial_cnt,
		int jpg_mb_size );
	RetCode VPU_EncSetBasesJpgPartial(
		DecHandle handle,
		EncParam * param,
		int jpg_partial_size, 
		int jpg_partial_cnt,
		int jpg_mb_size );
#endif // JPEG_PARTIAL
	RetCode VPU_DecGetOutputInfo(
		DecHandle handle,
		DecOutputInfo * info);
	RetCode VPU_DecBitBufferFlush(
		DecHandle handle);
	RetCode VPU_DecClrDispFlag(
		DecHandle handle, int index);
	RetCode VPU_DecGiveCommand(
		DecHandle handle,
		CodecCommand cmd,
		void * parameter);	
	RetCode VPU_DecResetRdPtr(
		DecHandle handle);
	
#ifdef __cplusplus
}
#endif

#endif
