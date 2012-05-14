// File: VpuMain.h
#ifndef _VPUMAIN_H_
#define _VPUMAIN_H_

#include "VpuApi.h"

//------------------------------------------------------------------------------
// DEBUG DEFINE
//------------------------------------------------------------------------------
//#define	PRE_SCAN

// IF656 definition
#define IF656_BASE              0x01002000
#define IF656_CTRL              (IF656_BASE + 0x000)
#define IF656_X_SIZE            (IF656_BASE + 0x004)
#define IF656_Y_OFFSET          (IF656_BASE + 0x008)
#define IF656_INT_STS           (IF656_BASE + 0x00C)
#define IF656_RAS_Y             (IF656_BASE + 0x020)
#define IF656_RAS_C             (IF656_BASE + 0x024)

// MIXER definition
#define MIX_BASE                0x11000000
#define DISP_MIX                0x12000000

#define MIXER_ENDIAN			        0
#define FRAME_STRIDE                    1920
#define FRAME_ENDIAN                    1               // 0 (little endian), 1 (big endian)
#define FRAME_MAX_X                     1920
#define FRAME_MAX_Y                     1088
#define MIXER_LAYER_DISP                0

enum{ DIS_MODE_NTSC = 0, DIS_MODE_PAL };

#define MAX_FRAME				(16+1+4)     // AVC REF 16, REC 1 , FULL DUP 4	
#define STREAM_FILL_SIZE		( 512 * 16)  //  4 * 1024 | 512 | 512+256( wrap around test )
#define STREAM_END_SIZE			( 0 )			 //  
#define STREAM_READ_SIZE		( 512 * 4 )
#define MAX_NUM_INSTANCE		16
#define NUM_FRAME_BUF			MAX_FRAME

#define MAX_FILE_PATH			256
#define MAX_DYNAMIC_BUFCOUNT	3
#define	PRJ_CODA_7542			0xE91C

#define MAX_NUM_TO_BE_FLUSHED	10
// Mixer.c
typedef struct{
				int  Format;

				int  Index;
				int  AddrY;
				int  AddrCb;
				int  AddrCr;
				int  StrideY;
				int  StrideC;

				int  DispY;
				int  DispCb;
				int  DispCr;

				int  MvColBuf;
}FRAME_BUF;

enum{
				MODE420 = 0,
				MODE422 = 1,
				MODE224 = 2,
				MODE444 = 3,
				MODE400 = 4
};

typedef struct {
	char yuvFileName[MAX_FILE_PATH];
	char cmdFileName[MAX_FILE_PATH];
	char bitstreamFileName[MAX_FILE_PATH];
	char huffFileName[MAX_FILE_PATH];
	char qMatFileName[MAX_FILE_PATH];
	char qpFileName[MAX_FILE_PATH];
	char cfgFileName[MAX_FILE_PATH];	
	int stdMode;
	int picWidth;
	int picHeight;
	int kbps;
	int rotAngle;
	int mirDir;
	int useRot;
	int qpReport;
	int fmoEnable;
	int fmoSliceNum;
	int fmoType;
	int saveEncHeader;
	int dynamicAllocEnable;
	int ringBufferEnable;
	int rcIntraQp;
	int mjpgChromaFormat;
	int picQp;
#ifdef JPEG_PARTIAL
	int enJpgPartial;
#endif // JPEG_PARTIAL
} EncConfigParam;

typedef struct {
	char yuvFileName[MAX_FILE_PATH];
	char bitstreamFileName[MAX_FILE_PATH];
	char qpFileName[MAX_FILE_PATH];
	int  bitFormat;
	int mode264;
	int rotAngle;
	int mirDir;
	int useRot;
	int	useDering;
	int outNum;
	int prescanEnable;
	int checkeos;
	int mp4DeblkEnable;
	int iframeSearchEnable;
	int dynamicAllocEnable;
    int reorder;
	int	mpeg4Class;
#ifdef JPEG_PARTIAL
	int enJpgPartial;
#endif // JPEG_PARTIAL
} DecConfigParam;

typedef struct {
	int codecMode;
	int numMulti;
	int  multiMode[MAX_NUM_INSTANCE];
    char multiFileName[MAX_NUM_INSTANCE][MAX_FILE_PATH];
	union {
		EncConfigParam encConfig;
		DecConfigParam decConfig;
	} ConfigParam;
	int checkeos;
} MultiConfigParam;

typedef struct {
	int codecMode;
	int numMulti;
	int  multiMode[MAX_NUM_INSTANCE];
    char multiFileName[MAX_NUM_INSTANCE][MAX_FILE_PATH];
	char multiEncBsName[MAX_NUM_INSTANCE][MAX_FILE_PATH];
	EncConfigParam encConfig;
	DecConfigParam decConfig;
} DuplexConfigParam;

typedef struct {
	char SrcFileName[256];
	int NumFrame;
	int PicX;
	int PicY;
	int FrameRateDr;
	int FrameRateNr;
	
	// MPEG4 ONLY
	int VerId;
	int DataPartEn;
	int RevVlcEn;
	int ShortVideoHeader;
	int AnnexI;
	int AnnexJ;
	int AnnexK;
	int AnnexT;
	int IntraDcVlcThr;
	int VopQuant;
	
	// H.264 ONLY
	int ConstIntraPredFlag;
	int DisableDeblk;
	int DeblkOffsetA;
	int DeblkOffsetB;
	int ChromaQpOffset;
	int PicQpY;
	
	// MJPEG ONLY
	char HuffTabName[256];
	char QMatTabName[256];
	int VersionID;
	int FrmFormat;
	int RstIntval;
	int ThumbEnable;
	int ThumbSizeX;
	int ThumbSizeY;
	
	// COMMON
	int GopPicNum;
	int SliceMode;
	int SliceSizeMode;
	int SliceSizeNum;
	
	int IntraRefreshNum;
	
	int IntraQPEnable;
	int RCIntraQP;
	int MaxQpSetEnable;
	int MaxQp;
	int GammaSetEnable;
	int Gamma;
	int HecEnable;
	
	// RC
	int RcEnable;
	int RcBitRate;
	int RcInitDelay;
	int RcBufSize;
	
	
	// NEW RC Scheme
	int RcIntervalMode;
	int RcMBInterval;
	
	int Intra16x16ModeOnly;
} ENC_CFG;

#ifdef __cplusplus
extern "C" {
#endif

	void FrameBufferInit(int stdMode,int format, int picX, int picY, int frameBufNum);
	FRAME_BUF *GetFrameBuffer(int index);
	void	ClearFrameBuffer(int index);
#if 0 // FIXME: ybfqing	
	int		SetMixerDecOut(int index);
	int		SetMixerDecOutLayer(int index, int picX, int picY);
	void	WaitMixerInt(void);
	void	SetVideoIn(int index, int picX, int picY);
	void	WaitVideoInInt(void);
	int		SetFreqICS307M(int Device, int OutFreqMHz, int InFreqMHz);
	int		SetHPITimingOpt(void);
#endif
#ifdef __cplusplus
}
#endif // end of __cplusplus

#endif // end of _VPUMAIN_H_
