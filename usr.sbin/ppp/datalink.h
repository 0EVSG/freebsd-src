/*-
 * Copyright (c) 1998 Brian Somers <brian@Awfulhak.org>
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
 *
 *	$Id: datalink.h,v 1.1.2.5 1998/02/17 01:05:38 brian Exp $
 */

#define DATALINK_CLOSED  (0)
#define DATALINK_OPENING (1)
#define DATALINK_HANGUP  (2)
#define DATALINK_DIAL    (3)
#define DATALINK_LOGIN   (4)
#define DATALINK_READY   (5)
#define DATALINK_OPEN    (6)

struct datalink {
  struct descriptor desc;       /* We play either a physical or a chat */
  int state;			/* Our DATALINK_* state */
  struct physical *physical;	/* Our link */

  struct chat chat;		/* For bringing the link up & down */
  struct {
    char dial[SCRIPT_LEN];	/* dial */
    char login[SCRIPT_LEN];	/* login */
    char hangup[SCRIPT_LEN];	/* hangup */
    unsigned run : 1;		/* run scripts ? */
    unsigned packetmode : 1;	/* Go into packet mode after login ? */
  } script;

  struct pppTimer dial_timer;	/* For timing between opens & scripts */

  int dial_tries;		/* currently try again this number of times */
  int max_dial;			/* initially try again this number of times */
  int dial_timeout;		/* Redial timeout value */
  int dial_next_timeout;	/* Redial next timeout value */

  unsigned reconnect_tries;	/* currently try again this number of times */
  int max_reconnect;		/* initially try again this number of times */
  int reconnect_timeout;	/* Timeout before reconnect on carrier loss */

  char *name;			/* Our name */

#ifdef soon
  struct lcp lcp;		/* Our line control FSM */
  struct ccp ccp;		/* Our compression FSM */
#endif

  struct bundle *bundle;	/* for the moment */
  struct datalink *next;	/* Next in the list */
};

#define datalink2descriptor(dl) (&(dl)->desc)
#define descriptor2datalink(d) \
  ((d)->type == DATALINK_DESCRIPTOR ? (struct datalink *)(d) : NULL)

extern struct datalink *datalink_Create(const char *name, struct bundle *);
extern struct datalink *datalink_Destroy(struct datalink *);
extern void datalink_Up(struct datalink *, int, int);
extern void datalink_Close(struct datalink *, int);
extern void datalink_Down(struct datalink *, int);
extern void datalink_StayDown(struct datalink *);
extern void datalink_Show(struct datalink *);
