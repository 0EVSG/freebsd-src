/*
 *	      PPP Line Quality Monitoring (LQM) Module
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
 * $Id: lqr.c,v 1.22.2.11 1998/03/13 21:07:08 brian Exp $
 *
 *	o LQR based on RFC1333
 *
 * TODO:
 *	o LQM policy
 *	o Allow user to configure LQM method and interval.
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
#include "lcpproto.h"
#include "lcp.h"
#include "lqr.h"
#include "hdlc.h"
#include "async.h"
#include "throughput.h"
#include "link.h"
#include "descriptor.h"
#include "physical.h"
#include "iplist.h"
#include "ipcp.h"
#include "bundle.h"
#include "loadalias.h"
#include "vars.h"

struct echolqr {
  u_int32_t magic;
  u_int32_t signature;
  u_int32_t sequence;
};

#define	SIGNATURE  0x594e4f54

static void
SendEchoReq(struct lcp *lcp)
{
  struct hdlc *hdlc = &link2physical(lcp->fsm.link)->hdlc;
  struct echolqr echo;

  LogPrintf(LogLQM, "Send echo LQR [%lu]\n", (u_long)hdlc->lqm.echo.seq_sent);

  echo.magic = htonl(lcp->want_magic);
  echo.signature = htonl(SIGNATURE);
  echo.sequence = htonl(hdlc->lqm.echo.seq_sent++);
  FsmOutput(&lcp->fsm, CODE_ECHOREQ, lcp->fsm.reqid++,
            (u_char *)&echo, sizeof echo);
}

void
RecvEchoLqr(struct fsm *fp, struct mbuf * bp)
{
  struct hdlc *hdlc = &link2physical(fp->link)->hdlc;
  struct echolqr *lqr;
  u_int32_t seq;

  if (plength(bp) == sizeof(struct echolqr)) {
    lqr = (struct echolqr *) MBUF_CTOP(bp);
    if (ntohl(lqr->signature) == SIGNATURE) {
      seq = ntohl(lqr->sequence);
      LogPrintf(LogLQM, "Got echo LQR [%d]\n", ntohl(lqr->sequence));
      /* careful not to update lqm.echo.seq_recv with older values */
      if ((hdlc->lqm.echo.seq_recv > (u_int32_t)0 - 5 && seq < 5) ||
          (hdlc->lqm.echo.seq_recv <= (u_int32_t)0 - 5 &&
           seq > hdlc->lqm.echo.seq_recv))
        hdlc->lqm.echo.seq_recv = seq;
    } else
      LogPrintf(LogERROR, "RecvEchoLqr: Got sig 0x%08x, expecting 0x%08x !\n",
                ntohl(lqr->signature), SIGNATURE);
  } else
    LogPrintf(LogERROR, "RecvEchoLqr: Got packet size %d, expecting %d !\n",
              plength(bp), sizeof(struct echolqr));
}

void
LqrChangeOrder(struct lqrdata * src, struct lqrdata * dst)
{
  u_long *sp, *dp;
  int n;

  sp = (u_long *) src;
  dp = (u_long *) dst;
  for (n = 0; n < sizeof(struct lqrdata) / sizeof(u_int32_t); n++)
    *dp++ = ntohl(*sp++);
}

static void
SendLqrReport(void *v)
{
  struct lcp *lcp = (struct lcp *)v;
  struct hdlc *hdlc = &link2physical(lcp->fsm.link)->hdlc;
  struct mbuf *bp;

  StopTimer(&hdlc->lqm.timer);

  if (hdlc->lqm.method & LQM_LQR) {
    if (hdlc->lqm.lqr.resent > 5) {
      /* XXX: Should implement LQM strategy */
      LogPrintf(LogPHASE, "** Too many LQR packets lost **\n");
      LogPrintf(LogLQM, "LqrOutput: Too many LQR packets lost\n");
      hdlc->lqm.method = 0;	/* Prevent recursion via bundle_Close() */
      bundle_Close(lcp->fsm.bundle, NULL, 1);
    } else {
      bp = mballoc(sizeof(struct lqrdata), MB_LQR);
      HdlcOutput(lcp->fsm.link, PRI_LINK, PROTO_LQR, bp);
      hdlc->lqm.lqr.resent++;
    }
  } else if (hdlc->lqm.method & LQM_ECHO) {
    if ((hdlc->lqm.echo.seq_sent > 5 &&
         hdlc->lqm.echo.seq_sent - 5 > hdlc->lqm.echo.seq_recv) ||
        (hdlc->lqm.echo.seq_sent <= 5 &&
         hdlc->lqm.echo.seq_sent > hdlc->lqm.echo.seq_recv + 5)) {
      LogPrintf(LogPHASE, "** Too many ECHO LQR packets lost **\n");
      LogPrintf(LogLQM, "LqrOutput: Too many ECHO LQR packets lost\n");
      hdlc->lqm.method = 0;	/* Prevent recursion via bundle_Close() */
      bundle_Close(lcp->fsm.bundle, NULL, 1);
    } else
      SendEchoReq(lcp);
  }
  if (hdlc->lqm.method && hdlc->lqm.timer.load)
    StartTimer(&hdlc->lqm.timer);
}

void
LqrInput(struct physical *physical, struct mbuf *bp)
{
  int len;

  len = plength(bp);
  if (len != sizeof(struct lqrdata))
    LogPrintf(LogERROR, "LqrInput: Got packet size %d, expecting %d !\n",
              len, sizeof(struct lqrdata));
  else if (!Acceptable(ConfLqr)) {
    bp->offset -= 2;
    bp->cnt += 2;
    lcp_SendProtoRej(physical->hdlc.lqm.owner, MBUF_CTOP(bp), bp->cnt);
  } else {
    struct lqrdata *lqr;
    struct lcp *lcp;
    u_int32_t lastLQR;

    lqr = (struct lqrdata *)MBUF_CTOP(bp);
    lcp = physical->hdlc.lqm.owner;
    if (ntohl(lqr->MagicNumber) != physical->hdlc.lqm.owner->his_magic)
      LogPrintf(LogERROR, "LqrInput: magic %x != expecting %x\n",
		ntohl(lqr->MagicNumber), physical->hdlc.lqm.owner->his_magic);
    else {
      /*
       * Remember our PeerInLQRs, then convert byte order and save
       */
      lastLQR = physical->hdlc.lqm.lqr.peer.PeerInLQRs;
      physical->hdlc.lqm.method |= LQM_LQR;

      LqrChangeOrder(lqr, &physical->hdlc.lqm.lqr.peer);
      LqrDump("Input", &physical->hdlc.lqm.lqr.peer);
      /* we have received an LQR from peer */
      physical->hdlc.lqm.lqr.resent = 0;

      /*
       * Generate an LQR response if we're not running an LQR timer OR
       * two successive LQR's PeerInLQRs are the same OR we're not going to
       * send our next one before the peers max timeout.
       */
      if (physical->hdlc.lqm.timer.load == 0 ||
          (lastLQR && lastLQR == physical->hdlc.lqm.lqr.peer.PeerInLQRs) ||
          (physical->hdlc.lqm.lqr.peer_timeout && 
           physical->hdlc.lqm.timer.rest * 100 / SECTICKS >
           physical->hdlc.lqm.lqr.peer_timeout))
        SendLqrReport(physical->hdlc.lqm.owner);
    }
  }
  pfree(bp);
}

/*
 *  When LCP is reached to opened state, We'll start LQM activity.
 */
void
StartLqm(struct lcp *lcp)
{
  struct physical *physical = link2physical(lcp->fsm.link);

  physical->hdlc.lqm.lqr.resent = 0;
  physical->hdlc.lqm.echo.seq_sent = 0;
  physical->hdlc.lqm.echo.seq_recv = 0;
  memset(&physical->hdlc.lqm.lqr.peer, '\0',
         sizeof physical->hdlc.lqm.lqr.peer);

  physical->hdlc.lqm.method = LQM_ECHO;
  if (Enabled(ConfLqr) && !REJECTED(lcp, TY_QUALPROTO))
    physical->hdlc.lqm.method |= LQM_LQR;
  StopTimer(&physical->hdlc.lqm.timer);

  physical->hdlc.lqm.lqr.peer_timeout = lcp->his_lqrperiod;
  physical->hdlc.lqm.owner = lcp;
  if (lcp->his_lqrperiod)
    LogPrintf(LogLQM, "Expecting LQR every %d.%02d secs\n",
	      lcp->his_lqrperiod / 100, lcp->his_lqrperiod % 100);

  if (lcp->want_lqrperiod) {
    LogPrintf(LogLQM, "Will send %s every %d.%02d secs\n",
              physical->hdlc.lqm.method & LQM_LQR ? "LQR" : "ECHO LQR",
              lcp->want_lqrperiod / 100, lcp->want_lqrperiod % 100);
    physical->hdlc.lqm.timer.state = TIMER_STOPPED;
    physical->hdlc.lqm.timer.load = lcp->want_lqrperiod * SECTICKS / 100;
    physical->hdlc.lqm.timer.func = SendLqrReport;
    physical->hdlc.lqm.timer.arg = lcp;
    SendLqrReport(lcp);
  } else {
    physical->hdlc.lqm.timer.load = 0;
    if (!lcp->his_lqrperiod)
      LogPrintf(LogLQM, "LQR/ECHO LQR not negotiated\n");
  }
}

void
StopLqrTimer(struct physical *physical)
{
  StopTimer(&physical->hdlc.lqm.timer);
}

void
StopLqr(struct physical *physical, int method)
{
  LogPrintf(LogLQM, "StopLqr method = %x\n", method);

  if (method == LQM_LQR)
    LogPrintf(LogLQM, "Stop sending LQR, Use LCP ECHO instead.\n");
  if (method == LQM_ECHO)
    LogPrintf(LogLQM, "Stop sending LCP ECHO.\n");
  physical->hdlc.lqm.method &= ~method;
  if (physical->hdlc.lqm.method)
    SendLqrReport(physical->hdlc.lqm.owner);
  else
    StopTimer(&physical->hdlc.lqm.timer);
}

void
LqrDump(const char *message, const struct lqrdata * lqr)
{
  if (LogIsKept(LogLQM)) {
    LogPrintf(LogLQM, "%s:\n", message);
    LogPrintf(LogLQM, "  Magic:          %08x   LastOutLQRs:    %08x\n",
	      lqr->MagicNumber, lqr->LastOutLQRs);
    LogPrintf(LogLQM, "  LastOutPackets: %08x   LastOutOctets:  %08x\n",
	      lqr->LastOutPackets, lqr->LastOutOctets);
    LogPrintf(LogLQM, "  PeerInLQRs:     %08x   PeerInPackets:  %08x\n",
	      lqr->PeerInLQRs, lqr->PeerInPackets);
    LogPrintf(LogLQM, "  PeerInDiscards: %08x   PeerInErrors:   %08x\n",
	      lqr->PeerInDiscards, lqr->PeerInErrors);
    LogPrintf(LogLQM, "  PeerInOctets:   %08x   PeerOutLQRs:    %08x\n",
	      lqr->PeerInOctets, lqr->PeerOutLQRs);
    LogPrintf(LogLQM, "  PeerOutPackets: %08x   PeerOutOctets:  %08x\n",
	      lqr->PeerOutPackets, lqr->PeerOutOctets);
  }
}
