/*
 *	             PPP Async HDLC Module
 *
 *	    Written by Toshiharu OHNO (tony-o@iij.ad.jp)
 *
 *   Copyright (C) 1993, Internet Initiative Japan, Inc. All rights reserverd.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the Internet Initiative Japan, Inc.  The name of the
 * IIJ may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * $Id: async.c,v 1.15.2.4 1998/02/02 19:33:29 brian Exp $
 *
 */
#include <sys/param.h>
#include <netinet/in.h>

#include <stdio.h>
#include <string.h>
#include <termios.h>

#include "command.h"
#include "mbuf.h"
#include "log.h"
#include "defs.h"
#include "timer.h"
#include "fsm.h"
#include "hdlc.h"
#include "lcp.h"
#include "lcpproto.h"
#include "modem.h"
#include "loadalias.h"
#include "vars.h"
#include "async.h"
#include "throughput.h"
#include "link.h"
#include "physical.h"

#define MODE_HUNT 0x01
#define MODE_ESC  0x02

void
async_Init(struct async *async)
{
  async->mode = MODE_HUNT;
  async->length = 0;
  async->my_accmap = async->his_accmap = 0xffffffff;
}

void
async_SetLinkParams(struct async *async, struct lcp *lcp)
{
  async->my_accmap = lcp->want_accmap;
  async->his_accmap = lcp->his_accmap;
}

/*
 * Encode into async HDLC byte code if necessary
 */
static void
HdlcPutByte(struct async *async, u_char **cp, u_char c, int proto)
{
  u_char *wp;

  wp = *cp;
  if ((c < 0x20 && (proto == PROTO_LCP || (async->his_accmap & (1 << c))))
      || (c == HDLC_ESC) || (c == HDLC_SYN)) {
    *wp++ = HDLC_ESC;
    c ^= HDLC_XOR;
  }
  if (EscMap[32] && EscMap[c >> 3] & (1 << (c & 7))) {
    *wp++ = HDLC_ESC;
    c ^= HDLC_XOR;
  }
  *wp++ = c;
  *cp = wp;
}

void
async_Output(int pri, struct mbuf *bp, int proto, struct physical *physical)
{
  u_char *cp, *sp, *ep;
  struct mbuf *wp;
  int cnt;

  if (plength(bp) > HDLCSIZE) {
    pfree(bp);
    return;
  }
  cp = physical->async.xbuff;
  ep = cp + HDLCSIZE - 10;
  wp = bp;
  *cp++ = HDLC_SYN;
  while (wp) {
    sp = MBUF_CTOP(wp);
    for (cnt = wp->cnt; cnt > 0; cnt--) {
      HdlcPutByte(&physical->async, &cp, *sp++, proto);
      if (cp >= ep) {
	pfree(bp);
	return;
      }
    }
    wp = wp->next;
  }
  *cp++ = HDLC_SYN;

  cnt = cp - physical->async.xbuff;
  LogDumpBuff(LogASYNC, "WriteModem", physical->async.xbuff, cnt);
  link_Write(physical2link(physical), pri, (char *)physical->async.xbuff, cnt);
  link_AddOutOctets(physical2link(physical), cnt);
  pfree(bp);
}

static struct mbuf *
async_Decode(struct async *async, u_char c)
{
  struct mbuf *bp;

  if ((async->mode & MODE_HUNT) && c != HDLC_SYN)
    return NULL;

  switch (c) {
  case HDLC_SYN:
    async->mode &= ~MODE_HUNT;
    if (async->length) {		/* packet is ready. */
      bp = mballoc(async->length, MB_ASYNC);
      mbwrite(bp, async->hbuff, async->length);
      async->length = 0;
      return bp;
    }
    break;
  case HDLC_ESC:
    if (!(async->mode & MODE_ESC)) {
      async->mode |= MODE_ESC;
      break;
    }
    /* Fall into ... */
  default:
    if (async->length >= HDLCSIZE) {
      /* packet is too large, discard it */
      LogPrintf(LogERROR, "Packet too large (%d), discarding.\n", async->length);
      async->length = 0;
      async->mode = MODE_HUNT;
      break;
    }
    if (async->mode & MODE_ESC) {
      c ^= HDLC_XOR;
      async->mode &= ~MODE_ESC;
    }
    async->hbuff[async->length++] = c;
    break;
  }
  return NULL;
}

void
async_Input(struct bundle *bundle, u_char *buff, int cnt,
            struct physical *physical)
{
  struct mbuf *bp;

  link_AddInOctets(physical2link(physical), cnt);

  if (Physical_IsSync(physical)) {
    bp = mballoc(cnt, MB_ASYNC);
    memcpy(MBUF_CTOP(bp), buff, cnt);
    bp->cnt = cnt;
    HdlcInput(bundle, bp, physical);
  } else {
    while (cnt > 0) {
      bp = async_Decode(&physical->async, *buff++);
      if (bp)
	HdlcInput(bundle, bp, physical);
      cnt--;
    }
  }
}
