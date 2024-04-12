/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2012-2016 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2023-2024 Florian Walpen <dev@submerge.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#define	PCI_VENDOR_XILINX		0x10ee
#define	PCI_DEVICE_XILINX_HDSP		0x3fc5 /* HDSP 9652 */
/* #define	PCI_CLASS_REVISION		0x08 */
#define	PCI_REVISION_AIO		212 /* TODO: HDSP 9632 revision */
#define	PCI_REVISION_RAYDAT		0x6c

#define	HDSP_9632			0
#define	HDSP_9652			1

/* Hardware mixer */
#define	HDSP_OUT_ENABLE_BASE		128
#define	HDSP_IN_ENABLE_BASE		384
/* TODO: Mixer base is probably 4096... */
#define	HDSP_MIXER_BASE		32768
#define	HDSP_MAX_GAIN			32768

/* Buffer */
/* TODO: These are probably set in buffer address registers. */
#define	HDSP_PAGE_ADDR_BUF_OUT		8192
#define	HDSP_PAGE_ADDR_BUF_IN		(HDSP_PAGE_ADDR_BUF_OUT + 64 * 16 * 4)
#define	HDSP_BUF_POSITION_MASK		0x000FFC0

/* Frequency */
#define	HDSP_FREQ_0			(1 << 6)
#define	HDSP_FREQ_1			(1 << 7)
#define	HDSP_FREQ_DOUBLE		(1 << 8)
#define	HDSP_FREQ_QUAD			(1 << 31)

#define	HDSP_FREQ_32000		HDSP_FREQ_0
#define	HDSP_FREQ_44100		HDSP_FREQ_1
#define	HDSP_FREQ_48000		(HDSP_FREQ_0 | HDSP_FREQ_1)
#define	HDSP_FREQ_MASK			(HDSP_FREQ_0 | HDSP_FREQ_1 |	\
					HDSP_FREQ_DOUBLE | HDSP_FREQ_QUAD)
#define	HDSP_FREQ_MASK_DEFAULT		HDSP_FREQ_48000
/* Only needed for 9632, firmware >= 152? */
#define	HDSP_FREQ_REG			256
#define	HDSP_FREQ_AIO			104857600000000ULL

#define	HDSP_SPEED_DEFAULT		48000

/* Latency */
#define	HDSP_LAT_0			(1 << 1)
#define	HDSP_LAT_1			(1 << 2)
#define	HDSP_LAT_2			(1 << 3)
#define	HDSP_LAT_MASK			(HDSP_LAT_0 | HDSP_LAT_1 | HDSP_LAT_2)
#define	HDSP_LAT_BYTES_MAX		(4096 * 4)
#define	HDSP_LAT_BYTES_MIN		(32 * 4)
#define	hdsp_encode_latency(x)		(((x)<<1) & HDSP_LAT_MASK)

/* Gain */
#define	HDSP_ADGain0			(1 << 25)
#define	HDSP_ADGain1			(1 << 26)
#define	HDSP_DAGain0			(1 << 27)
#define	HDSP_DAGain1			(1 << 28)
#define	HDSP_PhoneGain0			(1 << 29)
#define	HDSP_PhoneGain1			(1 << 30)

#define	HDSP_ADGainMask			(HDSP_ADGain0 | HDSP_ADGain1)
#define	HDSP_ADGainMinus10dBV		(HDSP_ADGainMask)
#define	HDSP_ADGainPlus4dBu		(HDSP_ADGain0)
#define	HDSP_ADGainLowGain		0

#define	HDSP_DAGainMask			(HDSP_DAGain0 | HDSP_DAGain1)
#define	HDSP_DAGainHighGain		(HDSP_DAGainMask)
#define	HDSP_DAGainPlus4dBu		(HDSP_DAGain0)
#define	HDSP_DAGainMinus10dBV		0

#define	HDSP_PhoneGainMask		(HDSP_PhoneGain0|HDSP_PhoneGain1)
#define	HDSP_PhoneGain0dB		HDSP_PhoneGainMask
#define	HDSP_PhoneGainMinus6dB		(HDSP_PhoneGain0)
#define	HDSP_PhoneGainMinus12dB		0

/* Settings */
/* TODO: Settings register sets frequency, for 9632 firmware >= 152. */
#define	HDSP_SETTINGS_REG		0
/* TODO: Out buffer address register (32) and in buffer address register (36) */
#define	HDSP_CONTROL_REG		64
#define	HDSP_STATUS_REG			0
/* TODO: No status1 register, selected autosync ref in status2! */
#define	HDSP_STATUS1_REG		64
#define	HDSP_STATUS2_REG		192
#define	HDSP_ENABLE			(1 << 0)

/* Interrupts */
#define	HDSP_AUDIO_IRQ_PENDING		(1 << 0)
#define	HDSP_AUDIO_INT_ENABLE		(1 << 5)
#define	HDSP_INTERRUPT_ACK		96

/* Channels */
#define	HDSP_MAX_SLOTS			64 /* Mono channels */
#define	HDSP_MAX_CHANS			(HDSP_MAX_SLOTS / 2) /* Stereo pairs */

#define	HDSP_CHANBUF_SAMPLES		(16 * 1024)
#define	HDSP_CHANBUF_SIZE		(4 * HDSP_CHANBUF_SAMPLES)
#define	HDSP_DMASEGSIZE			(HDSP_CHANBUF_SIZE * HDSP_MAX_SLOTS)

/* TODO: Channel mapping for HDSP 9632. */
#define	HDSP_CHAN_AIO_LINE		(1 << 0)
#define	HDSP_CHAN_AIO_PHONE		(1 << 1)
#define	HDSP_CHAN_AIO_AES		(1 << 2)
#define	HDSP_CHAN_AIO_SPDIF		(1 << 3)
#define	HDSP_CHAN_AIO_ADAT		(1 << 4)
#define	HDSP_CHAN_AIO_ALL_REC		(HDSP_CHAN_AIO_LINE | \
					HDSP_CHAN_AIO_AES | \
					HDSP_CHAN_AIO_SPDIF | \
					HDSP_CHAN_AIO_ADAT)
#define	HDSP_CHAN_AIO_ALL		(HDSP_CHAN_AIO_ALL_REC | \
					HDSP_CHAN_AIO_PHONE) \

/* TODO: Channel mapping for HDSP 9652. */
#define	HDSP_CHAN_RAY_AES		(1 << 5)
#define	HDSP_CHAN_RAY_SPDIF		(1 << 6)
#define	HDSP_CHAN_RAY_ADAT1		(1 << 7)
#define	HDSP_CHAN_RAY_ADAT2		(1 << 8)
#define	HDSP_CHAN_RAY_ADAT3		(1 << 9)
#define	HDSP_CHAN_RAY_ADAT4		(1 << 10)
#define	HDSP_CHAN_RAY_ALL		(HDSP_CHAN_RAY_AES | \
					HDSP_CHAN_RAY_SPDIF | \
					HDSP_CHAN_RAY_ADAT1 | \
					HDSP_CHAN_RAY_ADAT2 | \
					HDSP_CHAN_RAY_ADAT3 | \
					HDSP_CHAN_RAY_ADAT4)

struct hdsp_channel {
	uint32_t	ports;
	char		*descr;
};

/* Preferred clock source. */
#define	HDSP_CONTROL_MASTER		(1 << 4)
#define HDSP_CONTROL_CLOCK_MASK		(HDSP_CONTROL_MASTER | (1 << 13) | \
					(1 << 16) | (1 << 17))
#define HDSP_CONTROL_CLOCK(n)		(((n & 0x04) << 11) | ((n & 0x03) << 16))

/* Autosync selected clock source. */
#define HDSP_STATUS2_CLOCK(n)		((n & 0x07) << 8)
#define HDSP_STATUS2_CLOCK_MASK		HDSP_STATUS2_CLOCK(0x07);

/* TODO: Lock and sync bits are read from status / status2 register. */

struct hdsp_clock_source {
	char		*name;
	uint32_t	control;
	uint32_t	status2;
	uint32_t	lock_bit;
	uint32_t	sync_bit;
};

static MALLOC_DEFINE(M_HDSP, "hdsp", "hdsp audio");

/* Channel registers */
struct sc_chinfo {
	struct snd_dbuf		*buffer;
	struct pcm_channel	*channel;
	struct sc_pcminfo	*parent;

	/* Channel information */
	struct pcmchan_caps	*caps;
	uint32_t	cap_fmts[4];
	uint32_t	dir;
	uint32_t	format;
	uint32_t	ports;
	uint32_t	lvol;
	uint32_t	rvol;

	/* Buffer */
	uint32_t	*data;
	uint32_t	size;
	uint32_t	position;

	/* Flags */
	uint32_t	run;
};

/* PCM device private data */
struct sc_pcminfo {
	device_t		dev;
	uint32_t		(*ih) (struct sc_pcminfo *scp);
	uint32_t		chnum;
	struct sc_chinfo	chan[HDSP_MAX_CHANS];
	struct sc_info		*sc;
	struct hdsp_channel	*hc;
};

/* HDSP device private data */
struct sc_info {
	device_t		dev;
	struct mtx		*lock;

	uint32_t		ctrl_register;
	uint32_t		settings_register;
	uint32_t		type;

	/* Control/Status register */
	struct resource		*cs;
	int			csid;
	bus_space_tag_t		cst;
	bus_space_handle_t	csh;

	struct resource		*irq;
	int			irqid;
	void			*ih;
	bus_dma_tag_t		dmat;

	/* Play/Record DMA buffers */
	uint32_t		*pbuf;
	uint32_t		*rbuf;
	uint32_t		bufsize;
	bus_dmamap_t		pmap;
	bus_dmamap_t		rmap;
	uint32_t		period;
	uint32_t		speed;
	uint32_t		force_period;
	uint32_t		force_speed;
};

#define	hdsp_read_1(sc, regno)						\
	bus_space_read_1((sc)->cst, (sc)->csh, (regno))
#define	hdsp_read_2(sc, regno)						\
	bus_space_read_2((sc)->cst, (sc)->csh, (regno))
#define	hdsp_read_4(sc, regno)						\
	bus_space_read_4((sc)->cst, (sc)->csh, (regno))

#define	hdsp_write_1(sc, regno, data)					\
	bus_space_write_1((sc)->cst, (sc)->csh, (regno), (data))
#define	hdsp_write_2(sc, regno, data)					\
	bus_space_write_2((sc)->cst, (sc)->csh, (regno), (data))
#define	hdsp_write_4(sc, regno, data)					\
	bus_space_write_4((sc)->cst, (sc)->csh, (regno), (data))
