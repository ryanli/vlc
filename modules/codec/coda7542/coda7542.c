/*****************************************************************************
 * coda7542.c: Decoder for Coda7542 Video Codec IP
 *****************************************************************************
 * Copyright © 2012 Robert Mugabe
 *
 * Authors: Robert Mugabe <robert.mugabe@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

/*****************************************************************************
 * Preamble
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

/* VLC includes */
#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_codec.h>

/* VPU includes */
#include "VpuApi.h"
#include "VpuMain.h"
#include "IOFunc.h"

/*****************************************************************************
 * Module descriptor
 *****************************************************************************/
static int		OpenDecoder  ( vlc_object_t * );
static void	   CloseDecoder ( vlc_object_t * );

vlc_module_begin ()
	set_category( CAT_INPUT )
	set_subcategory( SUBCAT_INPUT_VCODEC )
	set_description( N_("Coda7542 Video Codec IP") )
	set_capability( "decoder", 1000 )
	set_callbacks( OpenDecoder, CloseDecoder )
	add_shortcut( "coda7542" )
vlc_module_end ()

/*****************************************************************************
 * Local prototypes
 *****************************************************************************/
static picture_t *DecodeBlock(decoder_t *p_dec, block_t **pp_block);

/*****************************************************************************
 * decoder_sys_t : Coda7542 decoder structure
 *****************************************************************************/
struct decoder_sys_t
{
	// FIXME - fill in this structure
	DecHandle handle;
	DecInitialInfo init_info;
	bool inited;
	FrameBuffer *p_fb;
};

extern int vpu_mem_base;

/*****************************************************************************
* OpenDecoder: probe the decoder and return score
*****************************************************************************/
static int OpenDecoder(vlc_object_t *p_this)
{
	// FIXME - fill in this function
	decoder_t *p_dec = (decoder_t *) p_this;
	decoder_sys_t *p_sys;

	DecOpenParam dec_params = {0};

	BufInfo buf_info = {0};

	int ret;

	// VPU initialization
	;
	if ((ret = VPU_Init(ADDR_BIT_WORK)) != RETCODE_SUCCESS) {
		msg_Err(p_dec, "Error initializing Coda7542 VPU: error 0x%x", ret);
		return VLC_EGENERIC;
	}

	// check firmware version
	uint32_t version = 0;
	if ((ret = VPU_GetVersionInfo(&version)) != RETCODE_SUCCESS) {
		msg_Err(p_dec, "Error getting VPU version: error 0x%x", ret);
		return VLC_EGENERIC;
	}
	msg_Dbg(p_dec, "Coda7542 product ID: 0x%04x, firmware version: %d.%d-r%d",
		(version>>16) & 0xffff, (version>>12) & 0xf, (version>>8) & 0xf, version & 0xff);

	// decoder parameters: bitstream format
	switch (p_dec->fmt_in.i_codec) {
		case VLC_CODEC_H264:
			dec_params.bitstreamFormat = STD_AVC;
			break;
		case VLC_CODEC_VC1:
			dec_params.bitstreamFormat = STD_VC1;
			break;
		case VLC_CODEC_DIV3:
			dec_params.bitstreamFormat = STD_DIV3;
			break;
		case VLC_CODEC_MJPG:
			dec_params.bitstreamFormat = STD_MJPG;
		default:
			msg_Dbg(p_dec, "Unsupported bitstream format: 0x%x.", p_dec->fmt_in.i_codec);
			return VLC_EGENERIC;
	}

	// decoder parameters: others
	dec_params.bitstreamBuffer = vpu_mem_base + ADDR_BIT_STREAM;
	dec_params.bitstreamBufferSize = SMALL_STREAM_BUF_SIZE;
	dec_params.mp4DeblkEnable = 0;
	dec_params.reorderEnable = 1;
	dec_params.mp4Class = 0;
	dec_params.psSaveBuffer = vpu_mem_base + ADDR_PS_SAVE_BUFFER;
	dec_params.psSaveBufferSize = PS_SAVE_SIZE;

	// allocate p_sys
	p_sys = malloc(sizeof(*p_sys));
	if (!p_sys)
		return VLC_ENOMEM;

	p_dec->p_sys = p_sys;

	p_sys->inited = false;

	// open decoder
	if ((ret = VPU_DecOpen(&p_sys->handle, &dec_params)) != RETCODE_SUCCESS) {
		msg_Dbg(p_dec, "Error opening decoder at VPU: error 0x%x", ret);
		return VLC_EGENERIC;
	}

	p_dec->fmt_out.i_cat = VIDEO_ES;
    p_dec->fmt_out.i_codec = VLC_CODEC_YUYV;
    p_dec->fmt_out.video.i_width = p_dec->fmt_in.video.i_width;
    p_dec->fmt_out.video.i_height = p_dec->fmt_in.video.i_height;
    p_dec->b_need_packetized = true;

	p_dec->pf_decode_video = DecodeBlock;

	msg_Err(p_dec, "Coda7542 doesn't have a brain yet.");
	return VLC_SUCCESS;
}

/*****************************************************************************
 * CloseDecoder: decoder destruction
 *****************************************************************************/
static void CloseDecoder(vlc_object_t *p_this)
{
	// FIXME - fill in this function
	decoder_t *p_dec = (decoder_t *) p_this;
	decoder_sys_t *p_sys = p_dec->p_sys;
	VPU_DecClose(p_sys->handle);
}

RetCode WriteBsBufHelper(DecHandle handle, BufInfo *pBufInfo,
		phyaddr_t paBsBufStart, phyaddr_t paBsBufEnd,
		int defaultsize, int checkeos, int *pstreameos)
{
	RetCode ret = RETCODE_SUCCESS;
	uint32_t size = 0;
	int fillSize = 0;
	phyaddr_t paRdPtr, paWrPtr;

	ret = VPU_DecGetBitstreamBuffer(handle, &paRdPtr, &paWrPtr, &size);
	if( ret != RETCODE_SUCCESS )
	{
		fprintf(stderr, "VPU_DecGetBitstreamBuffer failed Error code is 0x%x \n", ret );
		goto FILL_BS_ERROR;
	}

	if( size <= 0 )
		return RETCODE_SUCCESS;

	if( defaultsize > 0 )
	{
		if( size < defaultsize )
			return RETCODE_SUCCESS;

		fillSize = defaultsize;
	}
	else
	{
		fillSize = ( ( size >> 9 ) << 9 );
	}

	if( fillSize == 0 )
		return RETCODE_SUCCESS;

#ifdef	PRE_SCAN
	fillSize = 128*8;
#endif

	fillSize = FillSdramBurst(pBufInfo, paWrPtr, paBsBufStart, paBsBufEnd,
			fillSize, STREAM_ENDIAN, checkeos, pstreameos);

	if( *pstreameos == 0 )
	{
		ret = VPU_DecUpdateBitstreamBuffer( handle, fillSize );
		if( ret != RETCODE_SUCCESS )
		{
			fprintf(stderr, "VPU_DecUpdateBitstreamBuffer failed Error code is 0x%x \n", ret );
			goto FILL_BS_ERROR;
		}
	}
	else
	{
		if( !pBufInfo->fillendbs )
		{
			ret = VPU_DecUpdateBitstreamBuffer(handle, STREAM_END_SIZE) ;
			if( ret != RETCODE_SUCCESS )
			{
				fprintf(stderr, "VPU_DecUpdateBitstreamBuffer failed Error code is 0x%x \n", ret );
				goto FILL_BS_ERROR;
			}

			pBufInfo->fillendbs = 1;
		}
	}

FILL_BS_ERROR:

	return ret;
}

/****************************************************************************
 * DecodeBlock: the whole thing
 ****************************************************************************/
static picture_t *DecodeBlock(decoder_t *p_dec, block_t **pp_block)
{
	// FIXME - fill in this function
	decoder_sys_t *p_sys = p_dec->p_sys;
	block_t *p_block = *pp_block;
	picture_t *p_pic;

	phyaddr_t pa_buf_start, pa_buf_end;
	uint32_t buf_size;

	BufInfo buf_info = {0};

	DecParam dec_param = {0};
	DecBufInfo dec_buf_info;
	DecOutputInfo out_info;

	int check_eos;

	int ret;

	buf_info.buf = p_block->p_buffer;
	buf_info.size = p_block->i_buffer;
	buf_info.point = 0;

	WriteBsBufHelper(p_sys->handle, &buf_info, vpu_mem_base+ADDR_BIT_STREAM,
		vpu_mem_base+ADDR_BIT_STREAM+SMALL_STREAM_BUF_SIZE, 0, 1, &check_eos);

	if (!p_sys->inited) {
		// get initial info of video
		VPU_DecSetEscSeqInit(p_sys->handle, 1);
		VPU_DecGetInitialInfo(p_sys->handle, &p_sys->init_info);
		VPU_DecSetEscSeqInit(p_sys->handle, 0);

		dec_buf_info.avcSliceBufInfo.sliceSaveBuffer = vpu_mem_base+ADDR_SLICE_SAVE_BUFFER;
		dec_buf_info.avcSliceBufInfo.sliceSaveBufferSize = SLICE_SAVE_SIZE;

		// register framebuffer
		p_sys->p_fb = malloc(p_sys->init_info.minFrameBufferCount * sizeof(*p_sys->p_fb));
		VPU_DecRegisterFrameBuffer(p_sys->handle, p_sys->p_fb,
			p_sys->init_info.minFrameBufferCount, p_sys->init_info.picWidth, &dec_buf_info);
	}

	VPU_DecStartOneFrame(p_sys->handle, &dec_param);

	while (VPU_IsBusy()) {
	}

	if ((ret = VPU_DecGetOutputInfo(p_sys->handle, &out_info)) != RETCODE_SUCCESS) {
		msg_Err(p_dec, "Error getting output info: error %x", ret);
		return NULL;
	}

	p_pic = decoder_NewPicture(p_dec);
	return p_pic;
}
