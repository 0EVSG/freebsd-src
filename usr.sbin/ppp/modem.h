/*
 *	    Written by Toshiharu OHNO (tony-o@iij.ad.jp)
 *
 *   Copyright (C) 1993, Internet Initiative Japan, Inc. All rights reserverd.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by the Internet Initiative Japan.  The name of the
 * IIJ may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * $Id: modem.h,v 1.16.2.6 1998/02/13 05:10:21 brian Exp $
 *
 *	TODO:
 */

struct physical;

extern int modem_Raw(struct physical *, struct bundle *);
extern struct physical *modem_Create(const char *);
extern int modem_Open(struct physical *, struct bundle *);
extern int modem_Speed(struct physical *);
extern speed_t IntToSpeed(int);
extern int modem_SetParity(struct physical *, const char *);
extern int modem_ShowStatus(struct cmdargs const *);
extern void modem_Close(struct physical *);
extern void modem_Offline(struct physical *);
