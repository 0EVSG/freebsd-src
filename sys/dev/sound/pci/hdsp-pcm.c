/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2012-2021 Ruslan Bukin <br@bsdpad.com>
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

/*
 * RME HDSP driver for FreeBSD (pcm-part).
 * Supported cards: HDSP 9632, HDSP 9652.
 */

#include <dev/sound/pcm/sound.h>
#include <dev/sound/pci/hdsp.h>
#include <dev/sound/chip.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <mixer_if.h>

#define HDSP_MATRIX_MAX	8

struct hdsp_latency {
	uint32_t n;
	uint32_t period;
	float ms;
};

static struct hdsp_latency latency_map[] = {
	{ 7,   32, 0.7 },
	{ 0,   64, 1.5 },
	{ 1,  128,   3 },
	{ 2,  256,   6 },
	{ 3,  512,  12 },
	{ 4, 1024,  23 },
	{ 5, 2048,  46 },
	{ 6, 4096,  93 },

	{ 0,    0,   0 },
};

struct hdsp_rate {
	uint32_t speed;
	uint32_t reg;
};

static struct hdsp_rate rate_map[] = {
	{  32000, (HDSP_FREQ_32000) },
	{  44100, (HDSP_FREQ_44100) },
	{  48000, (HDSP_FREQ_48000) },
	{  64000, (HDSP_FREQ_32000 | HDSP_FREQ_DOUBLE) },
	{  88200, (HDSP_FREQ_44100 | HDSP_FREQ_DOUBLE) },
	{  96000, (HDSP_FREQ_48000 | HDSP_FREQ_DOUBLE) },
	/* TODO: Allow quad speed sample rates for HDSP 9632. */
	/*{ 128000, (HDSP_FREQ_32000 | HDSP_FREQ_QUAD)   },
	{ 176400, (HDSP_FREQ_44100 | HDSP_FREQ_QUAD)   },
	{ 192000, (HDSP_FREQ_48000 | HDSP_FREQ_QUAD)   },*/

	{ 0, 0 },
};

static uint32_t
hdsp_channel_play_ports(struct hdsp_channel *hc)
{
	return (hc->ports & (HDSP_CHAN_9632_ALL | HDSP_CHAN_9652_ALL));
}

static uint32_t
hdsp_channel_rec_ports(struct hdsp_channel *hc)
{
	return (hc->ports & (HDSP_CHAN_9632_ALL | HDSP_CHAN_9652_ALL));
}

static unsigned int
hdsp_adat_width(uint32_t speed)
{
	if (speed > 96000)
		return (2);
	if (speed > 48000)
		return (4);
	return (8);
}

static uint32_t
hdsp_port_first(uint32_t ports)
{
	return (ports & (~(ports - 1)));	/* Extract first bit set. */
}

static uint32_t
hdsp_port_first_row(uint32_t ports)
{
	uint32_t ends;

	/* Restrict ports to one set with contiguous slots. */
	if (ports & HDSP_CHAN_9632_ADAT)
		ports = HDSP_CHAN_9632_ADAT;
	else if (ports & HDSP_CHAN_9632_ALL)	/* Gap after ADAT for 96kHz. */
		ports &= HDSP_CHAN_9632_ALL;
	else if (ports & HDSP_CHAN_9652_ADAT_ALL)
		ports &= HDSP_CHAN_9652_ADAT_ALL;
	else if (ports & HDSP_CHAN_9652_SPDIF)	/* Gap after ADAT for 96kHz. */
		ports &= HDSP_CHAN_9652_SPDIF;

	/* Ends of port rows are followed by a port which is not in the set. */
	ends = ports & (~(ports >> 1));
	/* First row of contiguous ports ends in the first row end. */
	return (ports & (ends ^ (ends - 1)));
}

static unsigned int
hdsp_channel_count(uint32_t ports, uint32_t adat_width)
{
	unsigned int count = 0;

	if (ports & HDSP_CHAN_9632_ALL) {
		/* HDSP 9632 ports. */
		if (ports & HDSP_CHAN_9632_ADAT)
			count += adat_width;
		if (ports & HDSP_CHAN_9632_SPDIF)
			count += 2;
		if (ports & HDSP_CHAN_9632_LINE)
			count += 2;
	} else if (ports & HDSP_CHAN_9652_ALL) {
		/* HDSP 9652 ports. */
		if (ports & HDSP_CHAN_9652_ADAT1)
			count += adat_width;
		if (ports & HDSP_CHAN_9652_ADAT2)
			count += adat_width;
		if (ports & HDSP_CHAN_9652_ADAT3)
			count += adat_width;
		if (ports & HDSP_CHAN_9652_SPDIF)
			count += 2;
	}

	return (count);
}

static unsigned int
hdsp_channel_offset(uint32_t subset, uint32_t ports, unsigned int adat_width)
{
	uint32_t preceding;

	/* Make sure we have a subset of ports. */
	subset &= ports;
	/* Include all ports preceding the first one of the subset. */
	preceding = ports & (~subset & (subset - 1));

	if (preceding & HDSP_CHAN_9632_ALL)
		preceding &= HDSP_CHAN_9632_ALL;	/* Contiguous 9632 slots. */
	else if (preceding & HDSP_CHAN_9652_ALL)
		preceding &= HDSP_CHAN_9652_ALL;	/* Contiguous 9652 slots. */

	return (hdsp_channel_count(preceding, adat_width));
}

static unsigned int
hdsp_port_slot_offset(uint32_t port, unsigned int adat_width)
{
	/* Exctract the first port (lowest bit) if set of ports. */
	switch (hdsp_port_first(port)) {
	/* HDSP 9632 ports */
	case HDSP_CHAN_9632_ADAT:
		return (0);
	case HDSP_CHAN_9632_SPDIF:
		return (8);
	case HDSP_CHAN_9632_LINE:
		return (10);

	/* HDSP 9652 ports */
	case HDSP_CHAN_9652_ADAT1:
		return (0);
	case HDSP_CHAN_9652_ADAT2:
		return (adat_width);
	case HDSP_CHAN_9652_ADAT3:
		return (2 * adat_width);
	case HDSP_CHAN_9652_SPDIF:
		return (24);
	default:
		return (0);
	}
}

static unsigned int
hdsp_port_slot_width(uint32_t ports, unsigned int adat_width)
{
	uint32_t row;

	/* Count number of contiguous slots from the first physical port. */
	row = hdsp_port_first_row(ports);
	return (hdsp_channel_count(row, adat_width));
}

static int
hdsp_hw_mixer(struct sc_chinfo *ch, unsigned int dst,
    unsigned int src, unsigned short data)
{
	struct sc_pcminfo *scp;
	struct sc_info *sc;
	int offs;

	scp = ch->parent;
	sc = scp->sc;

	offs = 0;
	if (ch->dir == PCMDIR_PLAY)
		offs = 64;

	hdsp_write_4(sc, HDSP_MIXER_BASE +
	    ((offs + src + 128 * dst) * sizeof(uint32_t)),
	    data & 0xFFFF);

	return (0);
};

static int
hdspchan_setgain(struct sc_chinfo *ch)
{
	struct sc_info *sc;
	uint32_t port, ports;
	unsigned int slot, end_slot;
	unsigned short volume;

	sc = ch->parent->sc;

	/* Iterate through all physical ports of the channel. */
	ports = ch->ports;
	port = hdsp_port_first(ports);
	while (port != 0) {
		/* Get slot range of the physical port. */
		slot =
		    hdsp_port_slot_offset(port, hdsp_adat_width(sc->speed));
		end_slot = slot +
		    hdsp_port_slot_width(port, hdsp_adat_width(sc->speed));

		/* Treat first slot as left channel. */
		volume = ch->lvol * HDSP_MAX_GAIN / 100;
		for (; slot < end_slot; slot++) {
			hdsp_hw_mixer(ch, slot, slot, volume);
			/* Subsequent slots all get the right channel volume. */
			volume = ch->rvol * HDSP_MAX_GAIN / 100;
		}

		ports &= ~port;
		port = hdsp_port_first(ports);
	}

	return (0);
}

static int
hdspmixer_init(struct snd_mixer *m)
{
	struct sc_pcminfo *scp;
	struct sc_info *sc;
	int mask;

	scp = mix_getdevinfo(m);
	sc = scp->sc;
	if (sc == NULL)
		return (-1);

	mask = SOUND_MASK_PCM;

	if (hdsp_channel_play_ports(scp->hc))
		mask |= SOUND_MASK_VOLUME;

	if (hdsp_channel_rec_ports(scp->hc))
		mask |= SOUND_MASK_RECLEV;

	snd_mtxlock(sc->lock);
	pcm_setflags(scp->dev, pcm_getflags(scp->dev) | SD_F_SOFTPCMVOL);
	mix_setdevs(m, mask);
	snd_mtxunlock(sc->lock);

	return (0);
}

static int
hdspmixer_set(struct snd_mixer *m, unsigned dev,
    unsigned left, unsigned right)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	int i;

	scp = mix_getdevinfo(m);

#if 0
	device_printf(scp->dev, "hdspmixer_set() %d %d\n",
	    left, right);
#endif

	for (i = 0; i < scp->chnum; i++) {
		ch = &scp->chan[i];
		if ((dev == SOUND_MIXER_VOLUME && ch->dir == PCMDIR_PLAY) ||
		    (dev == SOUND_MIXER_RECLEV && ch->dir == PCMDIR_REC)) {
			ch->lvol = left;
			ch->rvol = right;
			if (ch->run)
				hdspchan_setgain(ch);
		}
	}

	return (0);
}

static kobj_method_t hdspmixer_methods[] = {
	KOBJMETHOD(mixer_init,      hdspmixer_init),
	KOBJMETHOD(mixer_set,       hdspmixer_set),
	KOBJMETHOD_END
};
MIXER_DECLARE(hdspmixer);

static void
hdspchan_enable(struct sc_chinfo *ch, int value)
{
	struct sc_pcminfo *scp;
	struct sc_info *sc;
	uint32_t row, ports;
	int reg;
	unsigned int slot, end_slot;

	scp = ch->parent;
	sc = scp->sc;

	if (ch->dir == PCMDIR_PLAY)
		reg = HDSP_OUT_ENABLE_BASE;
	else
		reg = HDSP_IN_ENABLE_BASE;

	ch->run = value;

	/* Iterate through rows of ports with contiguous slots. */
	ports = ch->ports;
	row = hdsp_port_first_row(ports);
	while (row != 0) {
		slot =
		    hdsp_port_slot_offset(row, hdsp_adat_width(sc->speed));
		end_slot = slot +
		    hdsp_port_slot_width(row, hdsp_adat_width(sc->speed));

		for (; slot < end_slot; slot++) {
			hdsp_write_1(sc, reg + (4 * slot), value);
		}

		ports &= ~row;
		row = hdsp_port_first_row(ports);
	}
}

static int
hdsp_running(struct sc_info *sc)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	device_t *devlist;
	int devcount;
	int i, j;
	int err;

	if ((err = device_get_children(sc->dev, &devlist, &devcount)) != 0)
		goto bad;

	for (i = 0; i < devcount; i++) {
		scp = device_get_ivars(devlist[i]);
		for (j = 0; j < scp->chnum; j++) {
			ch = &scp->chan[j];
			if (ch->run)
				goto bad;
		}
	}

	free(devlist, M_TEMP);

	return (0);
bad:

#if 0
	device_printf(sc->dev, "hdsp is running\n");
#endif

	free(devlist, M_TEMP);

	return (1);
}

static void
hdsp_start_audio(struct sc_info *sc)
{

	sc->ctrl_register |= (HDSP_AUDIO_INT_ENABLE | HDSP_ENABLE);
	hdsp_write_4(sc, HDSP_CONTROL_REG, sc->ctrl_register);
}

static void
hdsp_stop_audio(struct sc_info *sc)
{

	if (hdsp_running(sc) == 1)
		return;

	sc->ctrl_register &= ~(HDSP_AUDIO_INT_ENABLE | HDSP_ENABLE);
	hdsp_write_4(sc, HDSP_CONTROL_REG, sc->ctrl_register);
}

static void
buffer_mux_write(uint32_t *dma, uint32_t *pcm, unsigned int pos,
    unsigned int samples, unsigned int slots, unsigned int channels)
{
	int slot;

	for (; samples > 0; samples--) {
		for (slot = 0; slot < slots; slot++) {
			dma[slot * HDSP_CHANBUF_SAMPLES + pos] =
			    pcm[pos * channels + slot];
		}
		pos = (pos + 1) % HDSP_CHANBUF_SAMPLES;
	}
}

static void
buffer_mux_port(uint32_t *dma, uint32_t *pcm, uint32_t subset, uint32_t ports,
    unsigned int pos, unsigned int samples, unsigned int adat_width,
    unsigned int pcm_width)
{
	unsigned int slot_offset, slots;
	unsigned int channels, chan_pos;

	/* Translate DMA slot offset to DMA buffer offset. */
	slot_offset = hdsp_port_slot_offset(subset, adat_width);
	dma += slot_offset * HDSP_CHANBUF_SAMPLES;

	/* Channel position of the port subset and total number of channels. */
	chan_pos = hdsp_channel_offset(subset, ports, pcm_width);
	pcm += chan_pos;
	channels = hdsp_channel_count(ports, pcm_width);

	/* Only copy as much as supported by both hardware and pcm channel. */
	slots = hdsp_port_slot_width(subset, MIN(adat_width, pcm_width));

	/* Let the compiler inline and loop unroll common cases. */
	if (slots == 2)
		buffer_mux_write(dma, pcm, pos, samples, 2, channels);
	else if (slots == 4)
		buffer_mux_write(dma, pcm, pos, samples, 4, channels);
	else if (slots == 8)
		buffer_mux_write(dma, pcm, pos, samples, 8, channels);
	else
		buffer_mux_write(dma, pcm, pos, samples, slots, channels);
}

static void
buffer_demux_read(uint32_t *dma, uint32_t *pcm, unsigned int pos,
    unsigned int samples, unsigned int slots, unsigned int channels)
{
	int slot;

	for (; samples > 0; samples--) {
		for (slot = 0; slot < slots; slot++) {
			pcm[pos * channels + slot] =
			    dma[slot * HDSP_CHANBUF_SAMPLES + pos];
		}
		pos = (pos + 1) % HDSP_CHANBUF_SAMPLES;
	}
}

static void
buffer_demux_port(uint32_t *dma, uint32_t *pcm, uint32_t subset, uint32_t ports,
    unsigned int pos, unsigned int samples, unsigned int adat_width,
    unsigned int pcm_width)
{
	unsigned int slot_offset, slots;
	unsigned int channels, chan_pos;

	/* Translate port slot offset to DMA buffer offset. */
	slot_offset = hdsp_port_slot_offset(subset, adat_width);
	dma += slot_offset * HDSP_CHANBUF_SAMPLES;

	/* Channel position of the port subset and total number of channels. */
	chan_pos = hdsp_channel_offset(subset, ports, pcm_width);
	pcm += chan_pos;
	channels = hdsp_channel_count(ports, pcm_width);

	/* Only copy as much as supported by both hardware and pcm channel. */
	slots = hdsp_port_slot_width(subset, MIN(adat_width, pcm_width));

	/* Let the compiler inline and loop unroll common cases. */
	if (slots == 2)
		buffer_demux_read(dma, pcm, pos, samples, 2, channels);
	else if (slots == 4)
		buffer_demux_read(dma, pcm, pos, samples, 4, channels);
	else if (slots == 8)
		buffer_demux_read(dma, pcm, pos, samples, 8, channels);
	else
		buffer_demux_read(dma, pcm, pos, samples, slots, channels);
}


/* Copy data between DMA and PCM buffers. */
static void
buffer_copy(struct sc_chinfo *ch)
{
	struct sc_pcminfo *scp;
	struct sc_info *sc;
	uint32_t row, ports;
	uint32_t dma_pos;
	unsigned int pos, length, offset;
	unsigned int n;
	unsigned int adat_width, pcm_width;

	scp = ch->parent;
	sc = scp->sc;

	n = AFMT_CHANNEL(ch->format); /* n channels */

	/* Let pcm formats differ from current hardware ADAT width. */
	adat_width = hdsp_adat_width(sc->speed);
	if (n == hdsp_channel_count(ch->ports, 2))
		pcm_width = 2;
	else if (n == hdsp_channel_count(ch->ports, 4))
		pcm_width = 4;
	else
		pcm_width = 8;

	/* Derive buffer position and length to be copied. */
	if (ch->dir == PCMDIR_PLAY) {
		/* Position per channel is n times smaller than PCM. */
		pos = sndbuf_getreadyptr(ch->buffer) / n;
		length = sndbuf_getready(ch->buffer) / n;
		/* Copy no more than 2 periods in advance. */
		if (length > (sc->period * 4 * 2))
			length = (sc->period * 4 * 2);
		/* Skip what was already copied last time. */
		offset = (ch->position + HDSP_CHANBUF_SIZE) - pos;
		offset %= HDSP_CHANBUF_SIZE;
		if (offset <= length) {
			pos = (pos + offset) % HDSP_CHANBUF_SIZE;
			length -= offset;
		}
	} else {
		/* Position per channel is n times smaller than PCM. */
		pos = sndbuf_getfreeptr(ch->buffer) / n;
		/* Get DMA buffer write position. */
		dma_pos = hdsp_read_2(sc, HDSP_STATUS_REG);
		dma_pos &= HDSP_BUF_POSITION_MASK;
		/* Copy what is newly available. */
		length = (dma_pos + HDSP_CHANBUF_SIZE) - pos;
		length %= HDSP_CHANBUF_SIZE;
	}

	/* Position and length in samples (4 bytes). */
	pos /= 4;
	length /= 4;

	/* Iterate through rows of ports with contiguous slots. */
	ports = ch->ports;
	if (pcm_width == adat_width)
		row = hdsp_port_first_row(ports);
	else
		row = hdsp_port_first(ports);

	while (row != 0) {
		if (ch->dir == PCMDIR_PLAY)
			buffer_mux_port(sc->pbuf, ch->data, row, ch->ports, pos,
			    length, adat_width, pcm_width);
		else
			buffer_demux_port(sc->rbuf, ch->data, row, ch->ports,
			    pos, length, adat_width, pcm_width);

		ports &= ~row;
		if (pcm_width == adat_width)
			row = hdsp_port_first_row(ports);
		else
			row = hdsp_port_first(ports);
	}

	ch->position = ((pos + length) * 4) % HDSP_CHANBUF_SIZE;
}

static int
clean(struct sc_chinfo *ch)
{
	struct sc_pcminfo *scp;
	struct sc_info *sc;
	uint32_t *buf;
	uint32_t row, ports;
	unsigned int offset, slots;

	scp = ch->parent;
	sc = scp->sc;
	buf = sc->rbuf;

	if (ch->dir == PCMDIR_PLAY)
		buf = sc->pbuf;

	/* Iterate through rows of ports with contiguous slots. */
	ports = ch->ports;
	row = hdsp_port_first_row(ports);
	while (row != 0) {
		offset = hdsp_port_slot_offset(row,
		    hdsp_adat_width(sc->speed));
		slots = hdsp_port_slot_width(row, hdsp_adat_width(sc->speed));

		bzero(buf + offset * HDSP_CHANBUF_SAMPLES,
		    slots * HDSP_CHANBUF_SIZE);

		ports &= ~row;
		row = hdsp_port_first_row(ports);
	}

	ch->position = 0;

	return (0);
}

/* Channel interface. */
static void *
hdspchan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct sc_info *sc;
	int num;

	scp = devinfo;
	sc = scp->sc;

	snd_mtxlock(sc->lock);
	num = scp->chnum;

	ch = &scp->chan[num];

	if (dir == PCMDIR_PLAY)
		ch->ports = hdsp_channel_play_ports(scp->hc);
	else
		ch->ports = hdsp_channel_rec_ports(scp->hc);

	ch->run = 0;
	ch->lvol = 0;
	ch->rvol = 0;

	/* Support all possible ADAT widths as channel formats. */
	ch->cap_fmts[0] =
	    SND_FORMAT(AFMT_S32_LE, hdsp_channel_count(ch->ports, 2), 0);
	ch->cap_fmts[1] =
	    SND_FORMAT(AFMT_S32_LE, hdsp_channel_count(ch->ports, 4), 0);
	ch->cap_fmts[2] =
	    SND_FORMAT(AFMT_S32_LE, hdsp_channel_count(ch->ports, 8), 0);
	ch->cap_fmts[3] = 0;
	ch->caps = malloc(sizeof(struct pcmchan_caps), M_HDSP, M_NOWAIT);
	/* TODO: Allow quad speed sample rates for HDSP 9632. */
	*(ch->caps) = (struct pcmchan_caps) {32000, 96000, ch->cap_fmts, 0};

	/* Allocate maximum buffer size. */
	ch->size = HDSP_CHANBUF_SIZE * hdsp_channel_count(ch->ports, 8);
	ch->data = malloc(ch->size, M_HDSP, M_NOWAIT);
	ch->position = 0;

	ch->buffer = b;
	ch->channel = c;
	ch->parent = scp;

	ch->dir = dir;

	snd_mtxunlock(sc->lock);

	if (sndbuf_setup(ch->buffer, ch->data, ch->size) != 0) {
		device_printf(scp->dev, "Can't setup sndbuf.\n");
		return (NULL);
	}

	return (ch);
}

static int
hdspchan_trigger(kobj_t obj, void *data, int go)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct sc_info *sc;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	snd_mtxlock(sc->lock);
	switch (go) {
	case PCMTRIG_START:
#if 0
		device_printf(scp->dev, "hdspchan_trigger(): start\n");
#endif
		hdspchan_enable(ch, 1);
		hdspchan_setgain(ch);
		hdsp_start_audio(sc);
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
#if 0
		device_printf(scp->dev, "hdspchan_trigger(): stop or abort\n");
#endif
		clean(ch);
		hdspchan_enable(ch, 0);
		hdsp_stop_audio(sc);
		break;

	case PCMTRIG_EMLDMAWR:
	case PCMTRIG_EMLDMARD:
		if(ch->run)
			buffer_copy(ch);
		break;
	}

	snd_mtxunlock(sc->lock);

	return (0);
}

static uint32_t
hdspchan_getptr(kobj_t obj, void *data)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct sc_info *sc;
	uint32_t ret, pos;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	snd_mtxlock(sc->lock);
	ret = hdsp_read_2(sc, HDSP_STATUS_REG);
	snd_mtxunlock(sc->lock);

	pos = ret & HDSP_BUF_POSITION_MASK;
	pos *= AFMT_CHANNEL(ch->format); /* Hardbuf with multiple channels. */

	return (pos);
}

static int
hdspchan_free(kobj_t obj, void *data)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct sc_info *sc;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

#if 0
	device_printf(scp->dev, "hdspchan_free()\n");
#endif

	snd_mtxlock(sc->lock);
	if (ch->data != NULL) {
		free(ch->data, M_HDSP);
		ch->data = NULL;
	}
	if (ch->caps != NULL) {
		free(ch->caps, M_HDSP);
		ch->caps = NULL;
	}
	snd_mtxunlock(sc->lock);

	return (0);
}

static int
hdspchan_setformat(kobj_t obj, void *data, uint32_t format)
{
	struct sc_chinfo *ch;

	ch = data;

#if 0
	struct sc_pcminfo *scp = ch->parent;
	device_printf(scp->dev, "hdspchan_setformat(%d)\n", format);
#endif

	ch->format = format;

	return (0);
}

static uint32_t
hdspchan_setspeed(kobj_t obj, void *data, uint32_t speed)
{
	struct sc_pcminfo *scp;
	struct hdsp_rate *hr;
	struct sc_chinfo *ch;
	struct sc_info *sc;
	long long period;
	int threshold;
	int i;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;
	hr = NULL;

#if 0
	device_printf(scp->dev, "hdspchan_setspeed(%d)\n", speed);
#endif

	if (hdsp_running(sc) == 1)
		goto end;

	if (sc->force_speed > 0)
		speed = sc->force_speed;

	/* First look for equal frequency. */
	for (i = 0; rate_map[i].speed != 0; i++) {
		if (rate_map[i].speed == speed)
			hr = &rate_map[i];
	}

	/* If no match, just find nearest. */
	if (hr == NULL) {
		for (i = 0; rate_map[i].speed != 0; i++) {
			hr = &rate_map[i];
			threshold = hr->speed + ((rate_map[i + 1].speed != 0) ?
			    ((rate_map[i + 1].speed - hr->speed) >> 1) : 0);
			if (speed < threshold)
				break;
		}
	}

	switch (sc->type) {
	case HDSP_9652:
	case HDSP_9632:
		period = HDSP_FREQ_AIO;
		break;
	default:
		/* Unsupported card. */
		goto end;
	}

	/* Write frequency on the device. */
	sc->ctrl_register &= ~HDSP_FREQ_MASK;
	sc->ctrl_register |= hr->reg;
	hdsp_write_4(sc, HDSP_CONTROL_REG, sc->ctrl_register);

	speed = hr->speed;
	if (speed > 96000)
		speed /= 4;
	else if (speed > 48000)
		speed /= 2;

	/* Set DDS value. */
	period /= speed;
	hdsp_write_4(sc, HDSP_FREQ_REG, period);

	sc->speed = hr->speed;
end:

	return (sc->speed);
}

static uint32_t
hdspchan_setblocksize(kobj_t obj, void *data, uint32_t blocksize)
{
	struct hdsp_latency *hl;
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct sc_info *sc;
	int threshold;
	int i;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;
	hl = NULL;

#if 0
	device_printf(scp->dev, "hdspchan_setblocksize(%d)\n", blocksize);
#endif

	if (hdsp_running(sc) == 1)
		goto end;

	if (blocksize > HDSP_LAT_BYTES_MAX)
		blocksize = HDSP_LAT_BYTES_MAX;
	else if (blocksize < HDSP_LAT_BYTES_MIN)
		blocksize = HDSP_LAT_BYTES_MIN;

	blocksize /= 4 /* samples */;

	if (sc->force_period > 0)
		blocksize = sc->force_period;

	/* First look for equal latency. */
	for (i = 0; latency_map[i].period != 0; i++) {
		if (latency_map[i].period == blocksize)
			hl = &latency_map[i];
	}

	/* If no match, just find nearest. */
	if (hl == NULL) {
		for (i = 0; latency_map[i].period != 0; i++) {
			hl = &latency_map[i];
			threshold = hl->period + ((latency_map[i + 1].period != 0) ?
			    ((latency_map[i + 1].period - hl->period) >> 1) : 0);
			if (blocksize < threshold)
				break;
		}
	}

	snd_mtxlock(sc->lock);
	sc->ctrl_register &= ~HDSP_LAT_MASK;
	sc->ctrl_register |= hdsp_encode_latency(hl->n);
	hdsp_write_4(sc, HDSP_CONTROL_REG, sc->ctrl_register);
	sc->period = hl->period;
	snd_mtxunlock(sc->lock);

#if 0
	device_printf(scp->dev, "New period=%d\n", sc->period);
#endif

	sndbuf_resize(ch->buffer,
	    (HDSP_CHANBUF_SIZE * AFMT_CHANNEL(ch->format)) / (sc->period * 4),
	    (sc->period * 4));
end:

	return (sndbuf_getblksz(ch->buffer));
}

static uint32_t hdsp_bkp_fmt[] = {
	SND_FORMAT(AFMT_S32_LE, 2, 0),
	0
};

/* TODO: Allow quad speed sample rates for HDSP 9632. */
static struct pcmchan_caps hdsp_bkp_caps = {32000, 96000, hdsp_bkp_fmt, 0};

static struct pcmchan_caps *
hdspchan_getcaps(kobj_t obj, void *data)
{
	struct sc_chinfo *ch;

	ch = data;

#if 0
	struct sc_pcminfo *scl = ch->parent;
	device_printf(scp->dev, "hdspchan_getcaps()\n");
#endif

	if (ch->caps != NULL)
		return (ch->caps);

	return (&hdsp_bkp_caps);
}

static kobj_method_t hdspchan_methods[] = {
	KOBJMETHOD(channel_init,         hdspchan_init),
	KOBJMETHOD(channel_free,         hdspchan_free),
	KOBJMETHOD(channel_setformat,    hdspchan_setformat),
	KOBJMETHOD(channel_setspeed,     hdspchan_setspeed),
	KOBJMETHOD(channel_setblocksize, hdspchan_setblocksize),
	KOBJMETHOD(channel_trigger,      hdspchan_trigger),
	KOBJMETHOD(channel_getptr,       hdspchan_getptr),
	KOBJMETHOD(channel_getcaps,      hdspchan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(hdspchan);

static int
hdsp_pcm_probe(device_t dev)
{

#if 0
	device_printf(dev,"hdsp_pcm_probe()\n");
#endif

	return (0);
}

static uint32_t
hdsp_pcm_intr(struct sc_pcminfo *scp)
{
	struct sc_chinfo *ch;
	struct sc_info *sc;
	int i;

	sc = scp->sc;

	for (i = 0; i < scp->chnum; i++) {
		ch = &scp->chan[i];
		snd_mtxunlock(sc->lock);
		chn_intr(ch->channel);
		snd_mtxlock(sc->lock);
	}

	return (0);
}

static int
hdsp_pcm_attach(device_t dev)
{
	char status[SND_STATUSLEN];
	struct sc_pcminfo *scp;
	const char *buf;
	uint32_t pcm_flags;
	int err;
	int play, rec;

	scp = device_get_ivars(dev);
	scp->ih = &hdsp_pcm_intr;

	if (scp->hc->ports & HDSP_CHAN_9632_ALL)
		buf = "AIO";
	else if (scp->hc->ports & HDSP_CHAN_9652_ALL)
		buf = "RayDAT";
	else
		buf = "?";
	device_set_descf(dev, "HDSP %s [%s]", buf, scp->hc->descr);

	/*
	 * We don't register interrupt handler with snd_setup_intr
	 * in pcm device. Mark pcm device as MPSAFE manually.
	 */
	pcm_flags = pcm_getflags(dev) | SD_F_MPSAFE;
	if (hdsp_channel_count(scp->hc->ports, 8) > HDSP_MATRIX_MAX)
		/* Disable vchan conversion, too many channels. */
		pcm_flags |= SD_F_BITPERFECT;
	pcm_setflags(dev, pcm_flags);

	play = (hdsp_channel_play_ports(scp->hc)) ? 1 : 0;
	rec = (hdsp_channel_rec_ports(scp->hc)) ? 1 : 0;
	err = pcm_register(dev, scp, play, rec);
	if (err) {
		device_printf(dev, "Can't register pcm.\n");
		return (ENXIO);
	}

	scp->chnum = 0;
	if (play) {
		pcm_addchan(dev, PCMDIR_PLAY, &hdspchan_class, scp);
		scp->chnum++;
	}

	if (rec) {
		pcm_addchan(dev, PCMDIR_REC, &hdspchan_class, scp);
		scp->chnum++;
	}

	snprintf(status, SND_STATUSLEN, "port 0x%jx irq %jd on %s",
	    rman_get_start(scp->sc->cs),
	    rman_get_start(scp->sc->irq),
	    device_get_nameunit(device_get_parent(dev)));
	pcm_setstatus(dev, status);

	mixer_init(dev, &hdspmixer_class, scp);

	return (0);
}

static int
hdsp_pcm_detach(device_t dev)
{
	int err;

	err = pcm_unregister(dev);
	if (err) {
		device_printf(dev, "Can't unregister device.\n");
		return (err);
	}

	return (0);
}

static device_method_t hdsp_pcm_methods[] = {
	DEVMETHOD(device_probe,     hdsp_pcm_probe),
	DEVMETHOD(device_attach,    hdsp_pcm_attach),
	DEVMETHOD(device_detach,    hdsp_pcm_detach),
	{ 0, 0 }
};

static driver_t hdsp_pcm_driver = {
	"pcm",
	hdsp_pcm_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(snd_hdsp_pcm, hdsp, hdsp_pcm_driver, 0, 0);
MODULE_DEPEND(snd_hdsp, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
MODULE_VERSION(snd_hdsp, 1);
