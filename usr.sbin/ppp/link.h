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
 *  $Id: link.h,v 1.1.2.1 1998/01/30 19:45:50 brian Exp $
 *
 */


#define PHYSICAL_LINK 1
#define MP_LINK       2

#define LINK_QUEUES (PRI_MAX + 1)
#define NPROTOSTAT 11

struct link {
  int type;                               /* _LINK type */
  char *name;                             /* unique id per link type */
  int len;                                /* full size of parent struct */
  struct pppThroughput throughput;        /* Link throughput statistics */
  struct pppTimer Timer;                  /* inactivity timeout */
  struct mqueue Queue[LINK_QUEUES];       /* Our output queue of mbufs */

  u_long proto_in[NPROTOSTAT];            /* outgoing protocol stats */
  u_long proto_out[NPROTOSTAT];           /* incoming protocol stats */

  /* Implementation routines for use by link_ routines */
  void (*StartOutput)(struct link *);     /* send the queued data */
  int (*IsActive)(struct link *);         /* Are we active ? */
  void (*Close)(struct link *, int);      /* Close the link */
  void (*Destroy)(struct link *);         /* Destructor */
};

extern void link_AddInOctets(struct link *, int);
extern void link_AddOutOctets(struct link *, int);

extern void link_SequenceQueue(struct link *);
extern int link_QueueLen(struct link *);
extern struct mbuf *link_Dequeue(struct link *);
extern void link_Write(struct link *, int, const char *, int);
extern void link_StartOutput(struct link *);
extern void link_Output(struct link *, int, struct mbuf *);

#define PROTO_IN  1                       /* third arg to link_ProtocolRecord */
#define PROTO_OUT 2
extern void link_ProtocolRecord(struct link *, u_short, int);
extern void link_ReportProtocolStatus(struct link *);

extern int link_IsActive(struct link *);
extern void link_Close(struct link *, int);
extern void link_Destroy(struct link *);
