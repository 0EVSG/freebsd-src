/*
 *	PPP IP Control Protocol (IPCP) Module
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
 * $Id: ipcp.c,v 1.50.2.34 1998/04/07 23:45:55 brian Exp $
 *
 *	TODO:
 *		o More RFC1772 backwoard compatibility
 */
#include <sys/param.h>
#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <net/if.h>
#include <sys/sockio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/errno.h>
#include <termios.h>
#include <unistd.h>

#include "command.h"
#include "mbuf.h"
#include "log.h"
#include "defs.h"
#include "timer.h"
#include "fsm.h"
#include "lcpproto.h"
#include "lcp.h"
#include "iplist.h"
#include "throughput.h"
#include "slcompress.h"
#include "ipcp.h"
#include "filter.h"
#include "descriptor.h"
#include "loadalias.h"
#include "vars.h"
#include "vjcomp.h"
#include "lqr.h"
#include "hdlc.h"
#include "async.h"
#include "ccp.h"
#include "link.h"
#include "physical.h"
#include "mp.h"
#include "bundle.h"
#include "id.h"
#include "arp.h"
#include "systems.h"
#include "prompt.h"

#undef REJECTED
#define	REJECTED(p, x)	((p)->peer_reject & (1<<(x)))

struct compreq {
  u_short proto;
  u_char slots;
  u_char compcid;
};

static void IpcpLayerUp(struct fsm *);
static void IpcpLayerDown(struct fsm *);
static void IpcpLayerStart(struct fsm *);
static void IpcpLayerFinish(struct fsm *);
static void IpcpInitRestartCounter(struct fsm *);
static void IpcpSendConfigReq(struct fsm *);
static void IpcpSentTerminateReq(struct fsm *);
static void IpcpSendTerminateAck(struct fsm *, u_char);
static void IpcpDecodeConfig(struct fsm *, u_char *, int, int,
                             struct fsm_decode *);

static struct fsm_callbacks ipcp_Callbacks = {
  IpcpLayerUp,
  IpcpLayerDown,
  IpcpLayerStart,
  IpcpLayerFinish,
  IpcpInitRestartCounter,
  IpcpSendConfigReq,
  IpcpSentTerminateReq,
  IpcpSendTerminateAck,
  IpcpDecodeConfig,
  NullRecvResetReq,
  NullRecvResetAck
};

static const char *cftypes[] = {
  /* Check out the latest ``Assigned numbers'' rfc (rfc1700.txt) */
  "???",
  "IPADDRS",	/* 1: IP-Addresses */	/* deprecated */
  "COMPPROTO",	/* 2: IP-Compression-Protocol */
  "IPADDR",	/* 3: IP-Address */
};

#define NCFTYPES (sizeof cftypes/sizeof cftypes[0])

static const char *cftypes128[] = {
  /* Check out the latest ``Assigned numbers'' rfc (rfc1700.txt) */
  "???",
  "PRIDNS",	/* 129: Primary DNS Server Address */
  "PRINBNS",	/* 130: Primary NBNS Server Address */
  "SECDNS",	/* 131: Secondary DNS Server Address */
  "SECNBNS",	/* 132: Secondary NBNS Server Address */
};

#define NCFTYPES128 (sizeof cftypes128/sizeof cftypes128[0])

void
ipcp_AddInOctets(struct ipcp *ipcp, int n)
{
  throughput_addin(&ipcp->throughput, n);
}

void
ipcp_AddOutOctets(struct ipcp *ipcp, int n)
{
  throughput_addout(&ipcp->throughput, n);
}

int
ReportIpcpStatus(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, "%s [%s]\n", arg->bundle->ncp.ipcp.fsm.name,
          State2Nam(arg->bundle->ncp.ipcp.fsm.state));
  if (arg->bundle->ncp.ipcp.fsm.state == ST_OPENED) {
    prompt_Printf(arg->prompt, " His side:               %s, %s\n",
	    inet_ntoa(arg->bundle->ncp.ipcp.peer_ip),
            vj2asc(arg->bundle->ncp.ipcp.peer_compproto));
    prompt_Printf(arg->prompt, " My side:                %s, %s\n",
	    inet_ntoa(arg->bundle->ncp.ipcp.my_ip),
            vj2asc(arg->bundle->ncp.ipcp.my_compproto));
  }

  prompt_Printf(arg->prompt, "\nDefaults:\n");
  prompt_Printf(arg->prompt, " My Address:             %s/%d\n",
	  inet_ntoa(arg->bundle->ncp.ipcp.cfg.my_range.ipaddr),
          arg->bundle->ncp.ipcp.cfg.my_range.width);
  if (iplist_isvalid(&arg->bundle->ncp.ipcp.cfg.peer_list))
    prompt_Printf(arg->prompt, " His Address:            %s\n",
            arg->bundle->ncp.ipcp.cfg.peer_list.src);
  else
    prompt_Printf(arg->prompt, " His Address:            %s/%d\n",
	  inet_ntoa(arg->bundle->ncp.ipcp.cfg.peer_range.ipaddr),
          arg->bundle->ncp.ipcp.cfg.peer_range.width);
  if (arg->bundle->ncp.ipcp.cfg.HaveTriggerAddress)
    prompt_Printf(arg->prompt, " Negotiation(trigger):   %s\n",
            inet_ntoa(arg->bundle->ncp.ipcp.cfg.TriggerAddress));
  else
    prompt_Printf(arg->prompt, " Negotiation(trigger):   MYADDR\n");
  prompt_Printf(arg->prompt, " Initial VJ slots:       %d\n",
                arg->bundle->ncp.ipcp.cfg.VJInitSlots);
  prompt_Printf(arg->prompt, " Initial VJ compression: %s\n",
          arg->bundle->ncp.ipcp.cfg.VJInitComp ? "on" : "off");

  prompt_Printf(arg->prompt, "\n");
  throughput_disp(&arg->bundle->ncp.ipcp.throughput, arg->prompt);

  return 0;
}

int
SetInitVJ(struct cmdargs const *arg)
{
  if (arg->argc != 2)
    return -1;
  if (!strcasecmp(arg->argv[0], "slots")) {
    int slots;

    slots = atoi(arg->argv[1]);
    if (slots < 4 || slots > 16)
      return 1;
    arg->bundle->ncp.ipcp.cfg.VJInitSlots = slots;
    return 0;
  } else if (!strcasecmp(arg->argv[0], "slotcomp")) {
    if (!strcasecmp(arg->argv[1], "on"))
      arg->bundle->ncp.ipcp.cfg.VJInitComp = 1;
    else if (!strcasecmp(arg->argv[1], "off"))
      arg->bundle->ncp.ipcp.cfg.VJInitComp = 0;
    else
      return 2;
    return 0;
  }
  return -1;
}

void
ipcp_Init(struct ipcp *ipcp, struct bundle *bundle, struct link *l,
          const struct fsm_parent *parent)
{
  struct hostent *hp;
  char name[MAXHOSTNAMELEN];
  static const char *timer_names[] =
    {"IPCP restart", "IPCP openmode", "IPCP stopped"};

  fsm_Init(&ipcp->fsm, "IPCP", PROTO_IPCP, 1, IPCP_MAXCODE, 10, LogIPCP,
           bundle, l, parent, &ipcp_Callbacks, timer_names);

  ipcp->cfg.VJInitSlots = DEF_VJ_STATES;
  ipcp->cfg.VJInitComp = 1;
  memset(&ipcp->cfg.my_range, '\0', sizeof ipcp->cfg.my_range);
  if (gethostname(name, sizeof name) == 0) {
    hp = gethostbyname(name);
    if (hp && hp->h_addrtype == AF_INET) {
      memcpy(&ipcp->cfg.my_range.ipaddr.s_addr, hp->h_addr, hp->h_length);
      ipcp->cfg.peer_range.mask.s_addr = INADDR_BROADCAST;
      ipcp->cfg.peer_range.width = 32;
    }
  }
  ipcp->cfg.netmask.s_addr = INADDR_ANY;
  memset(&ipcp->cfg.peer_range, '\0', sizeof ipcp->cfg.peer_range);
  iplist_setsrc(&ipcp->cfg.peer_list, "");
  ipcp->cfg.HaveTriggerAddress = 0;

#ifndef NOMSEXT
  ipcp->cfg.ns_entries[0].s_addr = INADDR_ANY;
  ipcp->cfg.ns_entries[1].s_addr = INADDR_ANY;
  ipcp->cfg.nbns_entries[0].s_addr = INADDR_ANY;
  ipcp->cfg.nbns_entries[1].s_addr = INADDR_ANY;
#endif

  ipcp->cfg.fsmretry = DEF_FSMRETRY;

  memset(&ipcp->vj, '\0', sizeof ipcp->vj);

  ipcp->my_ifip.s_addr = INADDR_ANY;
  ipcp->peer_ifip.s_addr = INADDR_ANY;

  ipcp_Setup(ipcp);
}

void
ipcp_Setup(struct ipcp *ipcp)
{
  int pos;

  ipcp->fsm.open_mode = 0;
  ipcp->fsm.maxconfig = 10;

  if (iplist_isvalid(&ipcp->cfg.peer_list)) {
    if (ipcp->my_ifip.s_addr != INADDR_ANY &&
        (pos = iplist_ip2pos(&ipcp->cfg.peer_list, ipcp->my_ifip)) != -1)
      ipcp->cfg.peer_range.ipaddr = iplist_setcurpos(&ipcp->cfg.peer_list, pos);
    else
      ipcp->cfg.peer_range.ipaddr = iplist_setrandpos(&ipcp->cfg.peer_list);
    ipcp->cfg.peer_range.mask.s_addr = INADDR_BROADCAST;
    ipcp->cfg.peer_range.width = 32;
  }

  ipcp->heis1172 = 0;

  ipcp->peer_ip = ipcp->cfg.peer_range.ipaddr;
  ipcp->peer_compproto = 0;

  if (ipcp->cfg.HaveTriggerAddress) {
    /*
     * Some implementations of PPP require that we send a
     * *special* value as our address, even though the rfc specifies
     * full negotiation (e.g. "0.0.0.0" or Not "0.0.0.0").
     */
    ipcp->my_ip = ipcp->cfg.TriggerAddress;
    LogPrintf(LogIPCP, "Using trigger address %s\n",
              inet_ntoa(ipcp->cfg.TriggerAddress));
  } else if ((ipcp->my_ifip.s_addr & ipcp->cfg.my_range.mask.s_addr) ==
             (ipcp->cfg.my_range.ipaddr.s_addr &
              ipcp->cfg.my_range.mask.s_addr))
    /*
     * Otherwise, if we've been assigned an IP number before, we really
     * want to keep the same IP number so that we can keep any existing
     * connections that are bound to that IP.
     */
    ipcp->my_ip = ipcp->my_ifip;
  else
    ipcp->my_ip = ipcp->cfg.my_range.ipaddr;

  if (Enabled(ConfVjcomp))
    ipcp->my_compproto = (PROTO_VJCOMP << 16) +
                         ((ipcp->cfg.VJInitSlots - 1) << 8) +
                         ipcp->cfg.VJInitComp;
  else
    ipcp->my_compproto = 0;
  sl_compress_init(&ipcp->vj.cslc, ipcp->cfg.VJInitSlots - 1);

  ipcp->peer_reject = 0;
  ipcp->my_reject = 0;

  throughput_init(&ipcp->throughput);
}

static int
ipcp_SetIPaddress(struct bundle *bundle, struct in_addr myaddr,
                  struct in_addr hisaddr, int silent)
{
  struct sockaddr_in *sock_in;
  int s;
  u_long mask, addr;
  struct ifaliasreq ifra;

  /* If given addresses are alreay set, then ignore this request */
  if (bundle->ncp.ipcp.my_ifip.s_addr == myaddr.s_addr &&
      bundle->ncp.ipcp.peer_ifip.s_addr == hisaddr.s_addr)
    return 0;

  IpcpCleanInterface(&bundle->ncp.ipcp);

  s = ID0socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    LogPrintf(LogERROR, "SetIpDevice: socket(): %s\n", strerror(errno));
    return (-1);
  }

  memset(&ifra, '\0', sizeof ifra);
  strncpy(ifra.ifra_name, bundle->ifname, sizeof ifra.ifra_name - 1);
  ifra.ifra_name[sizeof ifra.ifra_name - 1] = '\0';

  /* Set interface address */
  sock_in = (struct sockaddr_in *)&ifra.ifra_addr;
  sock_in->sin_family = AF_INET;
  sock_in->sin_addr = myaddr;
  sock_in->sin_len = sizeof *sock_in;

  /* Set destination address */
  sock_in = (struct sockaddr_in *)&ifra.ifra_broadaddr;
  sock_in->sin_family = AF_INET;
  sock_in->sin_addr = hisaddr;
  sock_in->sin_len = sizeof *sock_in;

  addr = ntohl(myaddr.s_addr);
  if (IN_CLASSA(addr))
    mask = IN_CLASSA_NET;
  else if (IN_CLASSB(addr))
    mask = IN_CLASSB_NET;
  else
    mask = IN_CLASSC_NET;

  /* if subnet mask is given, use it instead of class mask */
  if (bundle->ncp.ipcp.cfg.netmask.s_addr != INADDR_ANY &&
      (ntohl(bundle->ncp.ipcp.cfg.netmask.s_addr) & mask) == mask)
    mask = ntohl(bundle->ncp.ipcp.cfg.netmask.s_addr);

  sock_in = (struct sockaddr_in *)&ifra.ifra_mask;
  sock_in->sin_family = AF_INET;
  sock_in->sin_addr.s_addr = htonl(mask);
  sock_in->sin_len = sizeof *sock_in;

  if (ID0ioctl(s, SIOCAIFADDR, &ifra) < 0) {
    if (!silent)
      LogPrintf(LogERROR, "SetIpDevice: ioctl(SIOCAIFADDR): %s\n",
		strerror(errno));
    close(s);
    return (-1);
  }

  bundle->ncp.ipcp.peer_ifip.s_addr = hisaddr.s_addr;
  bundle->ncp.ipcp.my_ifip.s_addr = myaddr.s_addr;

  if (Enabled(ConfProxy))
    sifproxyarp(bundle, bundle->ncp.ipcp.peer_ifip, s);

  close(s);
  return (0);
}

static struct in_addr
ChooseHisAddr(struct bundle *bundle, const struct in_addr gw)
{
  struct in_addr try;
  int f;

  for (f = 0; f < bundle->ncp.ipcp.cfg.peer_list.nItems; f++) {
    try = iplist_next(&bundle->ncp.ipcp.cfg.peer_list);
    LogPrintf(LogDEBUG, "ChooseHisAddr: Check item %d (%s)\n",
              f, inet_ntoa(try));
    if (ipcp_SetIPaddress(bundle, gw, try, 1) == 0) {
      LogPrintf(LogIPCP, "ChooseHisAddr: Selected IP address %s\n",
                inet_ntoa(try));
      break;
    }
  }

  if (f == bundle->ncp.ipcp.cfg.peer_list.nItems) {
    LogPrintf(LogDEBUG, "ChooseHisAddr: All addresses in use !\n");
    try.s_addr = INADDR_ANY;
  }

  return try;
}

static void
IpcpInitRestartCounter(struct fsm * fp)
{
  /* Set fsm timer load */
  struct ipcp *ipcp = fsm2ipcp(fp);

  fp->FsmTimer.load = ipcp->cfg.fsmretry * SECTICKS;
  fp->restart = 5;
}

static void
IpcpSendConfigReq(struct fsm *fp)
{
  /* Send config REQ please */
  struct physical *p = link2physical(fp->link);
  struct ipcp *ipcp = fsm2ipcp(fp);
  u_char buff[12];
  struct lcp_opt *o;

  o = (struct lcp_opt *)buff;

  if ((p && !Physical_IsSync(p)) || !REJECTED(ipcp, TY_IPADDR)) {
    *(u_int32_t *)o->data = ipcp->my_ip.s_addr;
    INC_LCP_OPT(TY_IPADDR, 6, o);
  }

  if (ipcp->my_compproto && !REJECTED(ipcp, TY_COMPPROTO))
    if (ipcp->heis1172) {
      *(u_short *)o->data = htons(PROTO_VJCOMP);
      INC_LCP_OPT(TY_COMPPROTO, 4, o);
    } else {
      *(u_long *)o->data = htonl(ipcp->my_compproto);
      INC_LCP_OPT(TY_COMPPROTO, 6, o);
    }

  FsmOutput(fp, CODE_CONFIGREQ, fp->reqid, buff, (u_char *)o - buff);
}

static void
IpcpSentTerminateReq(struct fsm * fp)
{
  /* Term REQ just sent by FSM */
}

static void
IpcpSendTerminateAck(struct fsm *fp, u_char id)
{
  /* Send Term ACK please */
  FsmOutput(fp, CODE_TERMACK, id, NULL, 0);
}

static void
IpcpLayerStart(struct fsm * fp)
{
  /* We're about to start up ! */
  LogPrintf(LogIPCP, "IpcpLayerStart.\n");

  /* This is where we should be setting up the interface in DEMAND mode */
}

static void
IpcpLayerFinish(struct fsm *fp)
{
  /* We're now down */
  LogPrintf(LogIPCP, "IpcpLayerFinish.\n");
}

void
IpcpCleanInterface(struct ipcp *ipcp)
{
  struct ifaliasreq ifra;
  struct sockaddr_in *me, *peer;
  int s;

  s = ID0socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    LogPrintf(LogERROR, "IpcpCleanInterface: socket: %s\n", strerror(errno));
    return;
  }

  if (Enabled(ConfProxy))
    cifproxyarp(ipcp->fsm.bundle, ipcp->peer_ifip, s);

  if (ipcp->my_ifip.s_addr != INADDR_ANY ||
      ipcp->peer_ifip.s_addr != INADDR_ANY) {
    memset(&ifra, '\0', sizeof ifra);
    strncpy(ifra.ifra_name, ipcp->fsm.bundle->ifname,
            sizeof ifra.ifra_name - 1);
    ifra.ifra_name[sizeof ifra.ifra_name - 1] = '\0';
    me = (struct sockaddr_in *)&ifra.ifra_addr;
    peer = (struct sockaddr_in *)&ifra.ifra_broadaddr;
    me->sin_family = peer->sin_family = AF_INET;
    me->sin_len = peer->sin_len = sizeof(struct sockaddr_in);
    me->sin_addr = ipcp->my_ifip;
    peer->sin_addr = ipcp->peer_ifip;
    if (ID0ioctl(s, SIOCDIFADDR, &ifra) < 0) {
      LogPrintf(LogERROR, "IpcpCleanInterface: ioctl(SIOCDIFADDR): %s\n",
                strerror(errno));
      close(s);
    }
    ipcp->my_ifip.s_addr = ipcp->peer_ifip.s_addr = INADDR_ANY;
  }

  close(s);
}

static void
IpcpLayerDown(struct fsm *fp)
{
  /* About to come down */
  struct ipcp *ipcp = fsm2ipcp(fp);
  const char *s;

  s = inet_ntoa(ipcp->peer_ifip);
  LogPrintf(LogIsKept(LogLINK) ? LogLINK : LogIPCP, "IpcpLayerDown: %s\n", s);

  throughput_stop(&ipcp->throughput);
  throughput_log(&ipcp->throughput, LogIPCP, NULL);
  /*
   * XXX this stuff should really live in the FSM.  Our config should
   * associate executable sections in files with events.
   */
  if (SelectSystem(fp->bundle, s, LINKDOWNFILE, NULL) < 0)
    if (GetLabel()) {
       if (SelectSystem(fp->bundle, GetLabel(), LINKDOWNFILE, NULL) < 0)
       SelectSystem(fp->bundle, "MYADDR", LINKDOWNFILE, NULL);
    } else
      SelectSystem(fp->bundle, "MYADDR", LINKDOWNFILE, NULL);

  if (ipcp->fsm.bundle->phys_type & PHYS_DEMAND)
    IpcpCleanInterface(ipcp);
}

static void
IpcpLayerUp(struct fsm *fp)
{
  /* We're now up */
  struct ipcp *ipcp = fsm2ipcp(fp);
  char tbuff[100];

  LogPrintf(LogIPCP, "IpcpLayerUp.\n");
  snprintf(tbuff, sizeof tbuff, "myaddr = %s ", inet_ntoa(ipcp->my_ip));
  LogPrintf(LogIsKept(LogIPCP) ? LogIPCP : LogLINK, " %s hisaddr = %s\n",
	    tbuff, inet_ntoa(ipcp->peer_ip));

  if (ipcp->peer_compproto >> 16 == PROTO_VJCOMP)
    sl_compress_init(&ipcp->vj.cslc, (ipcp->peer_compproto >> 8) & 255);

  if (ipcp_SetIPaddress(fp->bundle, ipcp->my_ip,
                        ipcp->peer_ip, 0) < 0) {
    LogPrintf(LogERROR, "IpcpLayerUp: unable to set ip address\n");
    return;
  }

#ifndef NOALIAS
  if (AliasEnabled())
    (*PacketAlias.SetAddress)(ipcp->my_ip);
#endif

  LogPrintf(LogIsKept(LogLINK) ? LogLINK : LogIPCP, "IpcpLayerUp: %s\n",
            inet_ntoa(ipcp->peer_ifip));

  /*
   * XXX this stuff should really live in the FSM.  Our config should
   * associate executable sections in files with events.
   */
  if (SelectSystem(fp->bundle, inet_ntoa(ipcp->my_ifip), LINKUPFILE, NULL) < 0)
    if (GetLabel()) {
      if (SelectSystem(fp->bundle, GetLabel(), LINKUPFILE, NULL) < 0)
        SelectSystem(fp->bundle, "MYADDR", LINKUPFILE, NULL);
    } else
      SelectSystem(fp->bundle, "MYADDR", LINKUPFILE, NULL);

  throughput_start(&ipcp->throughput, "IPCP throughput");
  bundle_DisplayPrompt(fp->bundle);
}

static int
AcceptableAddr(struct in_range *prange, struct in_addr ipaddr)
{
  /* Is the given IP in the given range ? */
  LogPrintf(LogDEBUG, "requested = %x\n", htonl(ipaddr.s_addr));
  LogPrintf(LogDEBUG, "range = %x\n", htonl(prange->ipaddr.s_addr));
  LogPrintf(LogDEBUG, "/%x\n", htonl(prange->mask.s_addr));
  LogPrintf(LogDEBUG, "%x, %x\n", htonl(prange->ipaddr.s_addr & prange->
		  mask.s_addr), htonl(ipaddr.s_addr & prange->mask.s_addr));
  return (prange->ipaddr.s_addr & prange->mask.s_addr) ==
    (ipaddr.s_addr & prange->mask.s_addr) && ipaddr.s_addr;
}

static void
IpcpDecodeConfig(struct fsm *fp, u_char * cp, int plen, int mode_type,
                 struct fsm_decode *dec)
{
  /* Deal with incoming PROTO_IPCP */
  struct ipcp *ipcp = fsm2ipcp(fp);
  int type, length;
  u_long *lp, compproto;
  struct compreq *pcomp;
  struct in_addr ipaddr, dstipaddr, dnsstuff, ms_info_req;
  char tbuff[100], tbuff2[100];

  while (plen >= sizeof(struct fsmconfig)) {
    type = *cp;
    length = cp[1];
    if (type < NCFTYPES)
      snprintf(tbuff, sizeof tbuff, " %s[%d] ", cftypes[type], length);
    else if (type > 128 && type < 128 + NCFTYPES128)
      snprintf(tbuff, sizeof tbuff, " %s[%d] ", cftypes128[type-128], length);
    else
      snprintf(tbuff, sizeof tbuff, " <%d>[%d] ", type, length);

    switch (type) {
    case TY_IPADDR:		/* RFC1332 */
      lp = (u_long *) (cp + 2);
      ipaddr.s_addr = *lp;
      LogPrintf(LogIPCP, "%s %s\n", tbuff, inet_ntoa(ipaddr));

      switch (mode_type) {
      case MODE_REQ:
        if (iplist_isvalid(&ipcp->cfg.peer_list)) {
          if (ipaddr.s_addr == INADDR_ANY ||
              iplist_ip2pos(&ipcp->cfg.peer_list, ipaddr) < 0 ||
              ipcp_SetIPaddress(fp->bundle, ipcp->cfg.my_range.ipaddr,
                                ipaddr, 1)) {
            LogPrintf(LogIPCP, "%s: Address invalid or already in use\n",
                      inet_ntoa(ipaddr));
            if (iplist_ip2pos(&ipcp->cfg.peer_list, ipcp->peer_ifip) >= 0)
              /*
               * If we've already got a valid address configured for the peer
               * (in DEMAND mode), try NAKing with that so that we don't
               * have to upset things too much.
               */
              ipcp->peer_ip = ipcp->peer_ifip;
            else
              /* Just pick an IP number from our list */
              ipcp->peer_ip = ChooseHisAddr
                (fp->bundle, ipcp->cfg.my_range.ipaddr);

            if (ipcp->peer_ip.s_addr == INADDR_ANY) {
	      memcpy(dec->rejend, cp, length);
	      dec->rejend += length;
            } else {
	      memcpy(dec->nakend, cp, 2);
	      memcpy(dec->nakend+2, &ipcp->peer_ip.s_addr, length - 2);
	      dec->nakend += length;
            }
	    break;
          }
	} else if (!AcceptableAddr(&ipcp->cfg.peer_range, ipaddr)) {
	  /*
	   * If destination address is not acceptable, NAK with what we
	   * want to use.
	   */
	  memcpy(dec->nakend, cp, 2);
          if ((ipcp->peer_ifip.s_addr & ipcp->cfg.peer_range.mask.s_addr) ==
             (ipcp->cfg.peer_range.ipaddr.s_addr &
              ipcp->cfg.peer_range.mask.s_addr))
            /* We prefer the already-configured address */
	    memcpy(dec->nakend+2, &ipcp->peer_ifip.s_addr, length - 2);
          else
	    memcpy(dec->nakend+2, &ipcp->peer_ip.s_addr, length - 2);
	  dec->nakend += length;
	  break;
	}
	ipcp->peer_ip = ipaddr;
	memcpy(dec->ackend, cp, length);
	dec->ackend += length;
	break;
      case MODE_NAK:
	if (AcceptableAddr(&ipcp->cfg.my_range, ipaddr)) {
	  /* Use address suggested by peer */
	  snprintf(tbuff2, sizeof tbuff2, "%s changing address: %s ", tbuff,
		   inet_ntoa(ipcp->my_ip));
	  LogPrintf(LogIPCP, "%s --> %s\n", tbuff2, inet_ntoa(ipaddr));
	  ipcp->my_ip = ipaddr;
	} else {
	  LogPrintf(LogIsKept(LogIPCP) ? LogIPCP : LogPHASE,
                    "%s: Unacceptable address!\n", inet_ntoa(ipaddr));
          FsmClose(&ipcp->fsm);
	}
	break;
      case MODE_REJ:
	ipcp->peer_reject |= (1 << type);
	break;
      }
      break;
    case TY_COMPPROTO:
      lp = (u_long *) (cp + 2);
      compproto = htonl(*lp);
      LogPrintf(LogIPCP, "%s %s\n", tbuff, vj2asc(compproto));

      switch (mode_type) {
      case MODE_REQ:
	if (!Acceptable(ConfVjcomp)) {
	  memcpy(dec->rejend, cp, length);
	  dec->rejend += length;
	} else {
	  pcomp = (struct compreq *) (cp + 2);
	  switch (length) {
	  case 4:		/* RFC1172 */
	    if (ntohs(pcomp->proto) == PROTO_VJCOMP) {
	      LogPrintf(LogWARN, "Peer is speaking RFC1172 compression protocol !\n");
	      ipcp->heis1172 = 1;
	      ipcp->peer_compproto = compproto;
	      memcpy(dec->ackend, cp, length);
	      dec->ackend += length;
	    } else {
	      memcpy(dec->nakend, cp, 2);
	      pcomp->proto = htons(PROTO_VJCOMP);
	      memcpy(dec->nakend+2, &pcomp, 2);
	      dec->nakend += length;
	    }
	    break;
	  case 6:		/* RFC1332 */
	    if (ntohs(pcomp->proto) == PROTO_VJCOMP
		&& pcomp->slots <= MAX_VJ_STATES
                && pcomp->slots >= MIN_VJ_STATES) {
	      ipcp->peer_compproto = compproto;
	      ipcp->heis1172 = 0;
	      memcpy(dec->ackend, cp, length);
	      dec->ackend += length;
	    } else {
	      memcpy(dec->nakend, cp, 2);
	      pcomp->proto = htons(PROTO_VJCOMP);
	      pcomp->slots = DEF_VJ_STATES;
	      pcomp->compcid = 0;
	      memcpy(dec->nakend+2, &pcomp, sizeof pcomp);
	      dec->nakend += length;
	    }
	    break;
	  default:
	    memcpy(dec->rejend, cp, length);
	    dec->rejend += length;
	    break;
	  }
	}
	break;
      case MODE_NAK:
	LogPrintf(LogIPCP, "%s changing compproto: %08x --> %08x\n",
		  tbuff, ipcp->my_compproto, compproto);
	ipcp->my_compproto = compproto;
	break;
      case MODE_REJ:
	ipcp->peer_reject |= (1 << type);
	break;
      }
      break;
    case TY_IPADDRS:		/* RFC1172 */
      lp = (u_long *) (cp + 2);
      ipaddr.s_addr = *lp;
      lp = (u_long *) (cp + 6);
      dstipaddr.s_addr = *lp;
      snprintf(tbuff2, sizeof tbuff2, "%s %s,", tbuff, inet_ntoa(ipaddr));
      LogPrintf(LogIPCP, "%s %s\n", tbuff2, inet_ntoa(dstipaddr));

      switch (mode_type) {
      case MODE_REQ:
	ipcp->peer_ip = ipaddr;
	ipcp->my_ip = dstipaddr;
	memcpy(dec->ackend, cp, length);
	dec->ackend += length;
	break;
      case MODE_NAK:
        snprintf(tbuff2, sizeof tbuff2, "%s changing address: %s", tbuff,
		 inet_ntoa(ipcp->my_ip));
	LogPrintf(LogIPCP, "%s --> %s\n", tbuff2, inet_ntoa(ipaddr));
	ipcp->my_ip = ipaddr;
	ipcp->peer_ip = dstipaddr;
	break;
      case MODE_REJ:
	ipcp->peer_reject |= (1 << type);
	break;
      }
      break;

      /*
       * MS extensions for MS's PPP
       */

#ifndef NOMSEXT
    case TY_PRIMARY_DNS:	/* MS PPP DNS negotiation hack */
    case TY_SECONDARY_DNS:
      switch (mode_type) {
      case MODE_REQ:
        if (!Enabled(ConfMSExt)) {
	  LogPrintf(LogIPCP, "MS NS req - rejected - msext disabled\n");
	  ipcp->my_reject |= (1 << type);
	  memcpy(dec->rejend, cp, length);
	  dec->rejend += length;
	  break;
        }
	lp = (u_long *) (cp + 2);
	dnsstuff.s_addr = *lp;
	ms_info_req.s_addr = ipcp->cfg.ns_entries
          [(type - TY_PRIMARY_DNS) ? 1 : 0].s_addr;
	if (dnsstuff.s_addr != ms_info_req.s_addr) {

	  /*
	   * So the client has got the DNS stuff wrong (first request) so
	   * we'll tell 'em how it is
	   */
	  memcpy(dec->nakend, cp, 2);	/* copy first two (type/length) */
	  LogPrintf(LogIPCP, "MS NS req %d:%s->%s - nak\n",
		    type, inet_ntoa(dnsstuff), inet_ntoa(ms_info_req));
	  memcpy(dec->nakend+2, &ms_info_req, length);
	  dec->nakend += length;
	  break;
	}

	/*
	 * Otherwise they have it right (this time) so we send a ack packet
	 * back confirming it... end of story
	 */
	LogPrintf(LogIPCP, "MS NS req %d:%s ok - ack\n",
		  type, inet_ntoa(ms_info_req));
	memcpy(dec->ackend, cp, length);
	dec->ackend += length;
	break;
      case MODE_NAK:		/* what does this mean?? */
	LogPrintf(LogIPCP, "MS NS req %d - NAK??\n", type);
	break;
      case MODE_REJ:		/* confused?? me to :) */
	LogPrintf(LogIPCP, "MS NS req %d - REJ??\n", type);
	break;
      }
      break;

    case TY_PRIMARY_NBNS:	/* MS PPP NetBIOS nameserver hack */
    case TY_SECONDARY_NBNS:
      switch (mode_type) {
      case MODE_REQ:
        if (!Enabled(ConfMSExt)) {
	  LogPrintf(LogIPCP, "MS NBNS req - rejected - msext disabled\n");
	  ipcp->my_reject |= (1 << type);
	  memcpy(dec->rejend, cp, length);
	  dec->rejend += length;
	  break;
        }
	lp = (u_long *) (cp + 2);
	dnsstuff.s_addr = *lp;
	ms_info_req.s_addr = ipcp->cfg.nbns_entries
          [(type - TY_PRIMARY_NBNS) ? 1 : 0].s_addr;
	if (dnsstuff.s_addr != ms_info_req.s_addr) {
	  memcpy(dec->nakend, cp, 2);
	  memcpy(dec->nakend+2, &ms_info_req.s_addr, length);
	  LogPrintf(LogIPCP, "MS NBNS req %d:%s->%s - nak\n",
		    type, inet_ntoa(dnsstuff), inet_ntoa(ms_info_req));
	  dec->nakend += length;
	  break;
	}
	LogPrintf(LogIPCP, "MS NBNS req %d:%s ok - ack\n",
		  type, inet_ntoa(ms_info_req));
	memcpy(dec->ackend, cp, length);
	dec->ackend += length;
	break;
      case MODE_NAK:
	LogPrintf(LogIPCP, "MS NBNS req %d - NAK??\n", type);
	break;
      case MODE_REJ:
	LogPrintf(LogIPCP, "MS NBNS req %d - REJ??\n", type);
	break;
      }
      break;

#endif

    default:
      if (mode_type != MODE_NOP) {
        ipcp->my_reject |= (1 << type);
        memcpy(dec->rejend, cp, length);
        dec->rejend += length;
      }
      break;
    }
    plen -= length;
    cp += length;
  }
}

void
IpcpInput(struct ipcp *ipcp, struct mbuf * bp)
{
  /* Got PROTO_IPCP from link */
  FsmInput(&ipcp->fsm, bp);
}

int
UseHisaddr(struct bundle *bundle, const char *hisaddr, int setaddr)
{
  struct ipcp *ipcp = &bundle->ncp.ipcp;

  /* Use `hisaddr' for the peers address (set iface if `setaddr') */
  memset(&ipcp->cfg.peer_range, '\0', sizeof ipcp->cfg.peer_range);
  iplist_reset(&ipcp->cfg.peer_list);
  if (strpbrk(hisaddr, ",-")) {
    iplist_setsrc(&ipcp->cfg.peer_list, hisaddr);
    if (iplist_isvalid(&ipcp->cfg.peer_list)) {
      iplist_setrandpos(&ipcp->cfg.peer_list);
      ipcp->peer_ip = ChooseHisAddr(bundle, ipcp->my_ip);
      if (ipcp->peer_ip.s_addr == INADDR_ANY) {
        LogPrintf(LogWARN, "%s: None available !\n",
                  ipcp->cfg.peer_list.src);
        return(0);
      }
      ipcp->cfg.peer_range.ipaddr.s_addr = ipcp->peer_ip.s_addr;
      ipcp->cfg.peer_range.mask.s_addr = INADDR_BROADCAST;
      ipcp->cfg.peer_range.width = 32;
    } else {
      LogPrintf(LogWARN, "%s: Invalid range !\n", hisaddr);
      return 0;
    }
  } else if (ParseAddr(ipcp, 1, &hisaddr, &ipcp->cfg.peer_range.ipaddr,
		       &ipcp->cfg.peer_range.mask,
                       &ipcp->cfg.peer_range.width) != 0) {
    ipcp->peer_ip.s_addr = ipcp->cfg.peer_range.ipaddr.s_addr;

    if (setaddr && ipcp_SetIPaddress(bundle, ipcp->cfg.my_range.ipaddr,
                                     ipcp->cfg.peer_range.ipaddr, 0) < 0) {
      ipcp->cfg.my_range.ipaddr.s_addr = INADDR_ANY;
      ipcp->cfg.peer_range.ipaddr.s_addr = INADDR_ANY;
      return 0;
    }
  } else
    return 0;

  return 1;
}
