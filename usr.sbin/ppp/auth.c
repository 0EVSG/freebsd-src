/*
 *			PPP Secret Key Module
 *
 *	    Written by Toshiharu OHNO (tony-o@iij.ad.jp)
 *
 *   Copyright (C) 1994, Internet Initiative Japan, Inc. All rights reserverd.
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
 * $Id: auth.c,v 1.27.2.3 1998/01/30 19:45:26 brian Exp $
 *
 *	TODO:
 *		o Implement check against with registered IP addresses.
 */
#include <sys/param.h>
#include <netinet/in.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "command.h"
#include "mbuf.h"
#include "defs.h"
#include "timer.h"
#include "fsm.h"
#include "iplist.h"
#include "throughput.h"
#include "ipcp.h"
#include "loadalias.h"
#include "vars.h"
#include "auth.h"
#include "chat.h"
#include "systems.h"
#include "physical.h"

void
LocalAuthInit()
{
  if (!(mode&MODE_DAEMON))
    /* We're allowed in interactive mode */
    VarLocalAuth = LOCAL_AUTH;
  else if (VarHaveLocalAuthKey)
    VarLocalAuth = *VarLocalAuthKey == '\0' ? LOCAL_AUTH : LOCAL_NO_AUTH;
  else
    switch (LocalAuthValidate(SECRETFILE, VarShortHost, "")) {
    case NOT_FOUND:
      VarLocalAuth = LOCAL_DENY;
      break;
    case VALID:
      VarLocalAuth = LOCAL_AUTH;
      break;
    case INVALID:
      VarLocalAuth = LOCAL_NO_AUTH;
      break;
    }
}

LOCAL_AUTH_VALID
LocalAuthValidate(const char *fname, const char *system, const char *key)
{
  FILE *fp;
  int n;
  char *vector[3];
  char buff[LINE_LEN];
  LOCAL_AUTH_VALID rc;

  rc = NOT_FOUND;		/* No system entry */
  fp = OpenSecret(fname);
  if (fp == NULL)
    return (rc);
  while (fgets(buff, sizeof buff, fp)) {
    if (buff[0] == '#')
      continue;
    buff[strlen(buff) - 1] = 0;
    memset(vector, '\0', sizeof vector);
    n = MakeArgs(buff, vector, VECSIZE(vector));
    if (n < 1)
      continue;
    if (strcmp(vector[0], system) == 0) {
      if ((vector[1] == (char *) NULL && (key == NULL || *key == '\0')) ||
          (vector[1] != (char *) NULL && strcmp(vector[1], key) == 0)) {
	rc = VALID;		/* Valid   */
      } else {
	rc = INVALID;		/* Invalid */
      }
      break;
    }
  }
  CloseSecret(fp);
  return (rc);
}

int
AuthValidate(struct bundle *bundle, const char *fname, const char *system,
             const char *key, struct physical *physical)
{
  FILE *fp;
  int n;
  char *vector[5];
  char buff[LINE_LEN];
  char passwd[100];

  fp = OpenSecret(fname);
  if (fp == NULL)
    return (0);
  while (fgets(buff, sizeof buff, fp)) {
    if (buff[0] == '#')
      continue;
    buff[strlen(buff) - 1] = 0;
    memset(vector, '\0', sizeof vector);
    n = MakeArgs(buff, vector, VECSIZE(vector));
    if (n < 2)
      continue;
    if (strcmp(vector[0], system) == 0) {
      ExpandString(vector[1], passwd, sizeof passwd, 0);
      if (strcmp(passwd, key) == 0) {
	CloseSecret(fp);
	if (n > 2 && !UseHisaddr(bundle, vector[2], 1))
	    return (0);
        /* XXX This should be deferred - we may join an existing bundle ! */
	IpcpInit(bundle, physical2link(physical));
	if (n > 3)
	  SetLabel(vector[3]);
	return (1);		/* Valid */
      }
    }
  }
  CloseSecret(fp);
  return (0);			/* Invalid */
}

char *
AuthGetSecret(struct bundle *bundle, const char *fname, const char *system,
              int len, int setaddr, struct physical *physical)
{
  FILE *fp;
  int n;
  char *vector[5];
  char buff[LINE_LEN];
  static char passwd[100];

  fp = OpenSecret(fname);
  if (fp == NULL)
    return (NULL);
  while (fgets(buff, sizeof buff, fp)) {
    if (buff[0] == '#')
      continue;
    buff[strlen(buff) - 1] = 0;
    memset(vector, '\0', sizeof vector);
    n = MakeArgs(buff, vector, VECSIZE(vector));
    if (n < 2)
      continue;
    if (strlen(vector[0]) == len && strncmp(vector[0], system, len) == 0) {
      ExpandString(vector[1], passwd, sizeof passwd, 0);
      if (setaddr)
	memset(&IpcpInfo.DefHisAddress, '\0', sizeof IpcpInfo.DefHisAddress);
      if (n > 2 && setaddr)
	if (UseHisaddr(bundle, vector[2], 1))
          /* XXX This should be deferred - we may join an existing bundle ! */
	  IpcpInit(bundle, physical2link(physical));
        else
          return NULL;
      if (n > 3)
        SetLabel(vector[3]);
      return (passwd);
    }
  }
  CloseSecret(fp);
  return (NULL);		/* Invalid */
}

static void
AuthTimeout(void *vauthp)
{
  struct pppTimer *tp;
  struct authinfo *authp = (struct authinfo *)vauthp;

  tp = &authp->authtimer;
  StopTimer(tp);
  if (--authp->retry > 0) {
    StartTimer(tp);
    (authp->ChallengeFunc) (++authp->id, authp->physical);
  }
}

void
StartAuthChallenge(struct authinfo *authp, struct physical *physical)
{
  struct pppTimer *tp;

  assert(authp->physical == NULL);

  authp->physical = physical;

  tp = &authp->authtimer;
  StopTimer(tp);
  tp->func = AuthTimeout;
  tp->load = VarRetryTimeout * SECTICKS;
  tp->state = TIMER_STOPPED;
  tp->arg = (void *) authp;
  StartTimer(tp);
  authp->retry = 3;
  authp->id = 1;
  (authp->ChallengeFunc) (authp->id, physical);
}

void
StopAuthTimer(struct authinfo *authp)
{
  StopTimer(&authp->authtimer);
  authp->physical = NULL;
}
