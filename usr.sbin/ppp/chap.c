/*
 *			PPP CHAP Module
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
 * $Id: chap.c,v 1.28.2.20 1998/04/03 19:24:42 brian Exp $
 *
 *	TODO:
 */
#include <sys/param.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>

#include <ctype.h>
#ifdef HAVE_DES
#include <md4.h>
#endif
#include <md5.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#ifdef __OpenBSD__
#include <util.h>
#else
#include <libutil.h>
#endif

#include "command.h"
#include "mbuf.h"
#include "log.h"
#include "defs.h"
#include "timer.h"
#include "fsm.h"
#include "chap_ms.h"
#include "lcpproto.h"
#include "lcp.h"
#include "lqr.h"
#include "hdlc.h"
#include "loadalias.h"
#include "vars.h"
#include "auth.h"
#include "chap.h"
#include "async.h"
#include "throughput.h"
#include "descriptor.h"
#include "iplist.h"
#include "slcompress.h"
#include "ipcp.h"
#include "filter.h"
#include "ccp.h"
#include "link.h"
#include "physical.h"
#include "mp.h"
#include "bundle.h"
#include "chat.h"
#include "datalink.h"

static const char *chapcodes[] = {
  "???", "CHALLENGE", "RESPONSE", "SUCCESS", "FAILURE"
};

static void
ChapOutput(struct physical *physical, u_int code, u_int id,
	   const u_char * ptr, int count)
{
  int plen;
  struct fsmheader lh;
  struct mbuf *bp;

  plen = sizeof(struct fsmheader) + count;
  lh.code = code;
  lh.id = id;
  lh.length = htons(plen);
  bp = mballoc(plen, MB_FSM);
  memcpy(MBUF_CTOP(bp), &lh, sizeof(struct fsmheader));
  if (count)
    memcpy(MBUF_CTOP(bp) + sizeof(struct fsmheader), ptr, count);
  LogDumpBp(LogDEBUG, "ChapOutput", bp);
  LogPrintf(LogLCP, "ChapOutput: %s\n", chapcodes[code]);
  HdlcOutput(physical2link(physical), PRI_LINK, PROTO_CHAP, bp);
}

void
SendChapChallenge(struct authinfo *auth, int chapid, struct physical *physical)
{
  struct chap *chap = auth2chap(auth);
  int len, i;
  char *cp;

  randinit();
  cp = chap->challenge_data;
  *cp++ = chap->challenge_len = random() % 32 + 16;
  for (i = 0; i < chap->challenge_len; i++)
    *cp++ = random() & 0xff;
  len = strlen(physical->dl->bundle->cfg.auth.name);
  memcpy(cp, physical->dl->bundle->cfg.auth.name, len);
  cp += len;
  ChapOutput(physical, CHAP_CHALLENGE, chapid, chap->challenge_data,
	     cp - chap->challenge_data);
}

static void
RecvChapTalk(struct bundle *bundle, struct fsmheader *chp, struct mbuf *bp,
             struct physical *physical)
{
  int valsize, len;
  int arglen, keylen, namelen;
  char *cp, *argp, *ap, *name, *digest;
  char *keyp;
  MD5_CTX MD5context;		/* context for MD5 */
  char answer[100];
  char cdigest[16];
#ifdef HAVE_DES
  int ix;
  MD4_CTX MD4context;		/* context for MD4 */
#endif

  len = ntohs(chp->length);
  LogPrintf(LogDEBUG, "RecvChapTalk: length: %d\n", len);
  arglen = len - sizeof(struct fsmheader);
  cp = (char *) MBUF_CTOP(bp);
  valsize = *cp++ & 255;
  name = cp + valsize;
  namelen = arglen - valsize - 1;
  name[namelen] = 0;
  LogPrintf(LogLCP, " Valsize = %d, Name = \"%s\"\n", valsize, name);

  switch (chp->code) {
  case CHAP_CHALLENGE:
    keyp = bundle->cfg.auth.key;
    keylen = strlen(bundle->cfg.auth.key);
    name = bundle->cfg.auth.name;
    namelen = strlen(bundle->cfg.auth.name);

#ifdef HAVE_DES
    if (physical->dl->chap.using_MSChap)
      argp = malloc(1 + namelen + MS_CHAP_RESPONSE_LEN);
    else
#endif
      argp = malloc(1 + valsize + namelen + 16);

    if (argp == NULL) {
      ChapOutput(physical, CHAP_FAILURE, chp->id, "Out of memory!", 14);
      return;
    }
#ifdef HAVE_DES
    if (physical->dl->chap.using_MSChap) {
      digest = argp;     /* this is the response */
      *digest++ = MS_CHAP_RESPONSE_LEN;   /* 49 */
      memset(digest, '\0', 24);
      digest += 24;

      ap = answer;       /* this is the challenge */
      memcpy(ap, keyp, keylen);
      ap += 2 * keylen;
      memcpy(ap, cp, valsize);
      LogDumpBuff(LogDEBUG, "recv", ap, valsize);
      ap += valsize;
      for (ix = keylen; ix > 0 ; ix--) {
          answer[2*ix-2] = answer[ix-1];
          answer[2*ix-1] = 0;
      }
      MD4Init(&MD4context);
      MD4Update(&MD4context, answer, 2 * keylen);
      MD4Final(digest, &MD4context);
      memcpy(digest + 25, name, namelen);
      ap += 2 * keylen;
      ChapMS(digest, answer + 2 * keylen, valsize);
      LogDumpBuff(LogDEBUG, "answer", digest, 24);
      ChapOutput(physical, CHAP_RESPONSE, chp->id, argp,
		 namelen + MS_CHAP_RESPONSE_LEN + 1);
    } else {
#endif
      digest = argp;
      *digest++ = 16;		/* value size */
      ap = answer;
      *ap++ = chp->id;
      memcpy(ap, keyp, keylen);
      ap += keylen;
      memcpy(ap, cp, valsize);
      LogDumpBuff(LogDEBUG, "recv", ap, valsize);
      ap += valsize;
      MD5Init(&MD5context);
      MD5Update(&MD5context, answer, ap - answer);
      MD5Final(digest, &MD5context);
      LogDumpBuff(LogDEBUG, "answer", digest, 16);
      memcpy(digest + 16, name, namelen);
      ap += namelen;
      /* Send answer to the peer */
      ChapOutput(physical, CHAP_RESPONSE, chp->id, argp, namelen + 17);
#ifdef HAVE_DES
    }
#endif
    free(argp);
    break;
  case CHAP_RESPONSE:
    /*
     * Get a secret key corresponds to the peer
     */
    keyp = AuthGetSecret(bundle, SECRETFILE, name, namelen,
			 chp->code == CHAP_RESPONSE,
			 physical);
    if (keyp) {
      /*
       * Compute correct digest value
       */
      keylen = strlen(keyp);
      ap = answer;
      *ap++ = chp->id;
      memcpy(ap, keyp, keylen);
      ap += keylen;
      MD5Init(&MD5context);
      MD5Update(&MD5context, answer, ap - answer);
      MD5Update(&MD5context, physical->dl->chap.challenge_data + 1,
                physical->dl->chap.challenge_len);
      MD5Final(cdigest, &MD5context);
      LogDumpBuff(LogDEBUG, "got", cp, 16);
      LogDumpBuff(LogDEBUG, "expect", cdigest, 16);

      /*
       * Compare with the response
       */
      if (memcmp(cp, cdigest, 16) == 0) {
	ChapOutput(physical, CHAP_SUCCESS, chp->id, "Welcome!!", 10);
        Physical_Login(physical, name);

        if (physical->link.lcp.auth_iwait == 0)
          /*
           * Either I didn't need to authenticate, or I've already been
           * told that I got the answer right.
           */
          datalink_AuthOk(physical->dl);

	break;
      }
    }

    /*
     * Peer is not registerd, or response digest is wrong.
     */
    ChapOutput(physical, CHAP_FAILURE, chp->id, "Invalid!!", 9);
    datalink_AuthNotOk(physical->dl);
    break;
  }
}

static void
RecvChapResult(struct bundle *bundle, struct fsmheader *chp, struct mbuf *bp,
	       struct physical *physical)
{
  int len;

  len = ntohs(chp->length);
  LogPrintf(LogDEBUG, "RecvChapResult: length: %d\n", len);
  if (chp->code == CHAP_SUCCESS) {
    if (physical->link.lcp.auth_iwait == PROTO_CHAP) {
      physical->link.lcp.auth_iwait = 0;
      if (physical->link.lcp.auth_ineed == 0)
        /*
         * We've succeeded in our ``login''
         * If we're not expecting  the peer to authenticate (or he already
         * has), proceed to network phase.
         */
        datalink_AuthOk(physical->dl);
    }
  } else {
    /* CHAP failed - it's not going to get any better */
    LogPrintf(LogPHASE, "Received CHAP_FAILURE\n");
    datalink_AuthNotOk(physical->dl);
  }
}

void
ChapInput(struct bundle *bundle, struct mbuf *bp, struct physical *physical)
{
  int len = plength(bp);
  struct fsmheader *chp;

  if (len >= sizeof(struct fsmheader)) {
    chp = (struct fsmheader *) MBUF_CTOP(bp);
    if (len >= ntohs(chp->length)) {
      if (chp->code < 1 || chp->code > 4)
	chp->code = 0;
      LogPrintf(LogLCP, "ChapInput: %s\n", chapcodes[chp->code]);

      bp->offset += sizeof(struct fsmheader);
      bp->cnt -= sizeof(struct fsmheader);

      switch (chp->code) {
      case CHAP_RESPONSE:
	StopAuthTimer(&physical->dl->chap.auth);
	/* Fall into.. */
      case CHAP_CHALLENGE:
	RecvChapTalk(bundle, chp, bp, physical);
	break;
      case CHAP_SUCCESS:
      case CHAP_FAILURE:
	RecvChapResult(bundle, chp, bp, physical);
	break;
      }
    }
  }
  pfree(bp);
}
