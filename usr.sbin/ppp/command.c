/*
 *		PPP User command processing module
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
 * $Id: command.c,v 1.131.2.53 1998/04/05 22:48:08 brian Exp $
 *
 */
#include <sys/param.h>
#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/route.h>
#include <netdb.h>

#ifndef NOALIAS
#include <alias.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "command.h"
#include "mbuf.h"
#include "log.h"
#include "defs.h"
#include "timer.h"
#include "fsm.h"
#include "lcp.h"
#include "iplist.h"
#include "throughput.h"
#include "slcompress.h"
#include "ipcp.h"
#include "modem.h"
#ifndef NOALIAS
#include "alias_cmd.h"
#endif
#include "lqr.h"
#include "hdlc.h"
#include "loadalias.h"
#include "vars.h"
#include "systems.h"
#include "filter.h"
#include "descriptor.h"
#include "main.h"
#include "route.h"
#include "ccp.h"
#include "auth.h"
#include "async.h"
#include "link.h"
#include "physical.h"
#include "mp.h"
#include "bundle.h"
#include "server.h"
#include "prompt.h"
#include "chat.h"
#include "chap.h"
#include "datalink.h"

static const char *HIDDEN = "********";

static int ShowCommand(struct cmdargs const *);
static int TerminalCommand(struct cmdargs const *);
static int QuitCommand(struct cmdargs const *);
static int CloseCommand(struct cmdargs const *);
static int DownCommand(struct cmdargs const *);
static int AllowCommand(struct cmdargs const *);
static int SetCommand(struct cmdargs const *);
static int LinkCommand(struct cmdargs const *);
static int AddCommand(struct cmdargs const *);
static int DeleteCommand(struct cmdargs const *);
#ifndef NOALIAS
static int AliasCommand(struct cmdargs const *);
static int AliasEnable(struct cmdargs const *);
static int AliasOption(struct cmdargs const *);
#endif

static int
HelpCommand(struct cmdargs const *arg)
{
  struct cmdtab const *cmd;
  int n, cmax, dmax, cols;

  if (!arg->prompt) {
    LogPrintf(LogWARN, "help: Cannot help without a prompt\n");
    return 0;
  }

  if (arg->argc > 0) {
    for (cmd = arg->cmdtab; cmd->name || cmd->alias; cmd++)
      if ((cmd->lauth & arg->prompt->auth) &&
          ((cmd->name && !strcasecmp(cmd->name, *arg->argv)) ||
           (cmd->alias && !strcasecmp(cmd->alias, *arg->argv)))) {
	prompt_Printf(arg->prompt, "%s\n", cmd->syntax);
	return 0;
      }
    return -1;
  }

  cmax = dmax = 0;
  for (cmd = arg->cmdtab; cmd->func; cmd++)
    if (cmd->name && (cmd->lauth & arg->prompt->auth)) {
      if ((n = strlen(cmd->name)) > cmax)
        cmax = n;
      if ((n = strlen(cmd->helpmes)) > dmax)
        dmax = n;
    }

  cols = 80 / (dmax + cmax + 3);
  n = 0;
  for (cmd = arg->cmdtab; cmd->func; cmd++)
    if (cmd->name && (cmd->lauth & arg->prompt->auth)) {
      prompt_Printf(arg->prompt, " %-*.*s: %-*.*s",
              cmax, cmax, cmd->name, dmax, dmax, cmd->helpmes);
      if (++n % cols == 0)
        prompt_Printf(arg->prompt, "\n");
    }
  if (n % cols != 0)
    prompt_Printf(arg->prompt, "\n");

  return 0;
}

int
IsInteractive(struct prompt *prompt)
{
  const char *m = NULL;

  if (mode & MODE_DDIAL)
    m = "direct dial";
  else if (mode & MODE_BACKGROUND)
    m = "background";
  else if (mode & MODE_AUTO)
    m = "auto";
  else if (mode & MODE_DEDICATED)
    m = "dedicated";
  else if (mode & MODE_INTER)
    m = "interactive";

  if (m)
    prompt_Printf(prompt, "Working in %s mode\n", m);

  return mode & MODE_INTER;
}

static int
CloneCommand(struct cmdargs const *arg)
{
  int f;

  if (arg->argc == 0)
    return -1;

  if (!arg->bundle->ncp.mp.active) {
    LogPrintf(LogWARN, "clone: Only available in multilink mode\n");
    return 1;
  }

  for (f = 0; f < arg->argc; f++)
    bundle_DatalinkClone(arg->bundle, arg->cx, arg->argv[f]);
  return 0;
}

static int
RemoveCommand(struct cmdargs const *arg)
{
  if (arg->argc != 0)
    return -1;

  if (!arg->bundle->ncp.mp.active) {
    LogPrintf(LogWARN, "remove: Only available in multilink mode\n");
    return 1;
  }

  if (arg->cx->state != DATALINK_CLOSED) {
    LogPrintf(LogWARN, "remove: Cannot delete links that aren't closed\n");
    return 2;
  }

  bundle_DatalinkRemove(arg->bundle, arg->cx);
  return 0;
}

static int
DialCommand(struct cmdargs const *arg)
{
  int res;

  if ((mode & MODE_DAEMON) && !(mode & MODE_AUTO)) {
    LogPrintf(LogWARN,
              "Manual dial is only available in auto and interactive mode\n");
    return 1;
  }

  if (arg->argc > 0 && (res = LoadCommand(arg)) != 0)
    return res;

  bundle_Open(arg->bundle, arg->cx ? arg->cx->name : NULL);

  return 0;
}

static int
ShellCommand(struct cmdargs const *arg, int bg)
{
  const char *shell;
  pid_t shpid;
  int argc;
  char *argv[MAXARGS];

#ifdef SHELL_ONLY_INTERACTIVELY
  /* we're only allowed to shell when we run ppp interactively */
  if (mode != MODE_INTER) {
    LogPrintf(LogWARN, "Can only start a shell in interactive mode\n");
    return 1;
  }
#endif
#ifdef NO_SHELL_IN_AUTO_INTERACTIVE

  /*
   * we want to stop shell commands when we've got a telnet connection to an
   * auto mode ppp
   */
  if (arg->prompt && !(mode & MODE_INTER)) {
    LogPrintf(LogWARN, "Shell is not allowed interactively in auto mode\n");
    return 1;
  }
#endif

  if (arg->argc == 0)
    if (!(mode & MODE_INTER)) {
      if (arg->prompt)
        LogPrintf(LogWARN, "Can't start an interactive shell from"
		  " a telnet session\n");
      else
        LogPrintf(LogWARN, "Can only start an interactive shell in"
		  " interactive mode\n");
      return 1;
    } else if (bg) {
      LogPrintf(LogWARN, "Can only start an interactive shell in"
		" the foreground mode\n");
      return 1;
    }
  if ((shell = getenv("SHELL")) == 0)
    shell = _PATH_BSHELL;

  if ((shpid = fork()) == 0) {
    int dtablesize, i, fd;

    TermTimerService();
    signal(SIGINT, SIG_DFL);
    signal(SIGQUIT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    signal(SIGHUP, SIG_DFL);
    signal(SIGALRM, SIG_DFL);

    if (arg->prompt)
      fd = arg->prompt->fd_out;
    else if ((fd = open("/dev/null", O_RDWR)) == -1) {
      LogPrintf(LogALERT, "Failed to open /dev/null: %s\n", strerror(errno));
      exit(1);
    }
    for (i = 0; i < 3; i++)
      dup2(fd, i);

    for (dtablesize = getdtablesize(), i = 3; i < dtablesize; i++)
      close(i);

    prompt_TtyOldMode(arg->prompt);
    setuid(geteuid());
    if (arg->argc > 0) {
      /* substitute pseudo args */
      argv[0] = strdup(arg->argv[0]);
      for (argc = 1; argc < arg->argc; argc++) {
	if (strcasecmp(arg->argv[argc], "HISADDR") == 0)
	  argv[argc] = strdup(inet_ntoa(arg->bundle->ncp.ipcp.peer_ip));
	else if (strcasecmp(arg->argv[argc], "INTERFACE") == 0)
	  argv[argc] = strdup(arg->bundle->ifname);
	else if (strcasecmp(arg->argv[argc], "MYADDR") == 0)
	  argv[argc] = strdup(inet_ntoa(arg->bundle->ncp.ipcp.my_ip));
        else
          argv[argc] = strdup(arg->argv[argc]);
      }
      argv[argc] = NULL;
      if (bg) {
	pid_t p;

	p = getpid();
	if (daemon(1, 1) == -1) {
	  LogPrintf(LogERROR, "%d: daemon: %s\n", p, strerror(errno));
	  exit(1);
	}
      } else if (arg->prompt)
        printf("ppp: Pausing until %s finishes\n", arg->argv[0]);
      execvp(argv[0], argv);
    } else {
      if (arg->prompt)
        printf("ppp: Pausing until %s finishes\n", shell);
      execl(shell, shell, NULL);
    }

    LogPrintf(LogWARN, "exec() of %s failed\n",
              arg->argc > 0 ? arg->argv[0] : shell);
    exit(255);
  }
  if (shpid == (pid_t) - 1) {
    LogPrintf(LogERROR, "Fork failed: %s\n", strerror(errno));
  } else {
    int status;

    waitpid(shpid, &status, 0);
  }

  if (arg->prompt)
    prompt_TtyCommandMode(arg->prompt);

  return (0);
}

static int
BgShellCommand(struct cmdargs const *arg)
{
  if (arg->argc == 0)
    return -1;
  return ShellCommand(arg, 1);
}

static int
FgShellCommand(struct cmdargs const *arg)
{
  return ShellCommand(arg, 0);
}

static struct cmdtab const Commands[] = {
  {"accept", NULL, AcceptCommand, LOCAL_AUTH,
  "accept option request", "accept option .."},
  {"add", NULL, AddCommand, LOCAL_AUTH,
  "add route", "add dest mask gateway", NULL},
  {NULL, "add!", AddCommand, LOCAL_AUTH,
  "add or change route", "add! dest mask gateway", (void *)1},
#ifndef NOALIAS
  {"alias", NULL, AliasCommand, LOCAL_AUTH,
  "alias control", "alias option [yes|no]"},
#endif
  {"allow", "auth", AllowCommand, LOCAL_AUTH,
  "Allow ppp access", "allow users|modes ...."},
  {"bg", "!bg", BgShellCommand, LOCAL_AUTH,
  "Run a background command", "[!]bg command"},
  {"clone", NULL, CloneCommand, LOCAL_AUTH | LOCAL_CX,
  "Clone a link", "clone newname..."},
  {"close", NULL, CloseCommand, LOCAL_AUTH | LOCAL_CX_OPT,
  "Close connection", "close"},
  {"delete", NULL, DeleteCommand, LOCAL_AUTH,
  "delete route", "delete dest", NULL},
  {NULL, "delete!", DeleteCommand, LOCAL_AUTH,
  "delete a route if it exists", "delete! dest", (void *)1},
  {"deny", NULL, DenyCommand, LOCAL_AUTH,
  "Deny option request", "deny option .."},
  {"dial", "call", DialCommand, LOCAL_AUTH | LOCAL_CX_OPT,
  "Dial and login", "dial|call [remote]"},
  {"disable", NULL, DisableCommand, LOCAL_AUTH,
  "Disable option", "disable option .."},
  {"display", NULL, DisplayCommand, LOCAL_AUTH,
  "Display option configs", "display"},
  {"down", NULL, DownCommand, LOCAL_AUTH | LOCAL_CX,
  "Generate a down event", "down"},
  {"enable", NULL, EnableCommand, LOCAL_AUTH,
  "Enable option", "enable option .."},
  {"link", NULL, LinkCommand, LOCAL_AUTH,
  "Link specific commands", "link name command ..."},
  {"load", NULL, LoadCommand, LOCAL_AUTH,
  "Load settings", "load [remote]"},
  {"passwd", NULL, PasswdCommand, LOCAL_NO_AUTH,
  "Password for manipulation", "passwd LocalPassword"},
  {"quit", "bye", QuitCommand, LOCAL_AUTH | LOCAL_NO_AUTH,
  "Quit PPP program", "quit|bye [all]"},
  {"remove", NULL, RemoveCommand, LOCAL_AUTH | LOCAL_CX,
  "Remove a link", "remove"},
  {"save", NULL, SaveCommand, LOCAL_AUTH,
  "Save settings", "save"},
  {"set", "setup", SetCommand, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set parameters", "set[up] var value"},
  {"shell", "!", FgShellCommand, LOCAL_AUTH,
  "Run a subshell", "shell|! [sh command]"},
  {"show", NULL, ShowCommand, LOCAL_AUTH,
  "Show status and stats", "show var"},
  {"term", NULL, TerminalCommand, LOCAL_AUTH | LOCAL_CX,
  "Enter terminal mode", "term"},
  {"help", "?", HelpCommand, LOCAL_AUTH | LOCAL_NO_AUTH,
  "Display this message", "help|? [command]", Commands},
  {NULL, NULL, NULL},
};

static int
ShowEscape(struct cmdargs const *arg)
{
  if (arg->cx->physical->async.cfg.EscMap[32]) {
    int code, bit;
    char *sep = "";

    for (code = 0; code < 32; code++)
      if (arg->cx->physical->async.cfg.EscMap[code])
	for (bit = 0; bit < 8; bit++)
	  if (arg->cx->physical->async.cfg.EscMap[code] & (1 << bit)) {
	    prompt_Printf(arg->prompt, "%s0x%02x", sep, (code << 3) + bit);
            sep = ", ";
          }
    prompt_Printf(arg->prompt, "\n");
  }
  return 0;
}

static int
ShowTimeout(struct cmdargs const *arg)
{
  int remaining;

  prompt_Printf(arg->prompt, "Idle Timer: %ds\n",
                arg->bundle->cfg.idle_timeout);
  remaining = bundle_RemainingIdleTime(arg->bundle);
  if (remaining != -1)
    prompt_Printf(arg->prompt, "Remaining:  %ds\n", remaining);

  return 0;
}

static int
ShowTimerList(struct cmdargs const *arg)
{
  ShowTimers(0, arg->prompt);
  return 0;
}

static int
ShowStopped(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, " Stopped Timer:  LCP: ");
  if (!arg->cx->physical->link.lcp.fsm.StoppedTimer.load)
    prompt_Printf(arg->prompt, "Disabled");
  else
    prompt_Printf(arg->prompt, "%ld secs",
                  arg->cx->physical->link.lcp.fsm.StoppedTimer.load / SECTICKS);

  prompt_Printf(arg->prompt, ", CCP: ");
  if (!arg->cx->physical->link.ccp.fsm.StoppedTimer.load)
    prompt_Printf(arg->prompt, "Disabled");
  else
    prompt_Printf(arg->prompt, "%ld secs",
                  arg->cx->physical->link.ccp.fsm.StoppedTimer.load / SECTICKS);

  prompt_Printf(arg->prompt, "\n");

  return 0;
}

static int
ShowAuthKey(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, "AuthName = %s\n", arg->bundle->cfg.auth.name);
  prompt_Printf(arg->prompt, "AuthKey  = %s\n", HIDDEN);
  return 0;
}

static int
ShowVersion(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, "%s - %s \n", VarVersion, VarLocalVersion);
  return 0;
}

int
ShowProtocolStats(struct cmdargs const *arg)
{
  struct link *l = ChooseLink(arg);

  prompt_Printf(arg->prompt, "%s:\n", l->name);
  link_ReportProtocolStatus(l, arg->prompt);
  return 0;
}


static int
ShowReconnect(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, "%s: Reconnect Timer:  %d,  %d tries\n",
                arg->cx->name, arg->cx->cfg.reconnect_timeout,
                arg->cx->cfg.max_reconnect);
  return 0;
}

static int
ShowRedial(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, " Redial Timer: ");

  if (arg->cx->cfg.dial_timeout >= 0)
    prompt_Printf(arg->prompt, " %d seconds, ", arg->cx->cfg.dial_timeout);
  else
    prompt_Printf(arg->prompt, " Random 0 - %d seconds, ", DIAL_TIMEOUT);

  prompt_Printf(arg->prompt, " Redial Next Timer: ");

  if (arg->cx->cfg.dial_next_timeout >= 0)
    prompt_Printf(arg->prompt, " %d seconds, ", arg->cx->cfg.dial_next_timeout);
  else
    prompt_Printf(arg->prompt, " Random 0 - %d seconds, ", DIAL_TIMEOUT);

  if (arg->cx->cfg.max_dial)
    prompt_Printf(arg->prompt, "%d dial tries", arg->cx->cfg.max_dial);

  prompt_Printf(arg->prompt, "\n");

  return 0;
}

#ifndef NOMSEXT
static int
ShowMSExt(struct cmdargs const *arg)
{
  prompt_Printf(arg->prompt, " MS PPP extention values \n");
  prompt_Printf(arg->prompt, "   Primary NS     : %s\n",
                inet_ntoa(arg->bundle->ncp.ipcp.cfg.ns_entries[0]));
  prompt_Printf(arg->prompt, "   Secondary NS   : %s\n",
                inet_ntoa(arg->bundle->ncp.ipcp.cfg.ns_entries[1]));
  prompt_Printf(arg->prompt, "   Primary NBNS   : %s\n",
                inet_ntoa(arg->bundle->ncp.ipcp.cfg.nbns_entries[0]));
  prompt_Printf(arg->prompt, "   Secondary NBNS : %s\n",
                inet_ntoa(arg->bundle->ncp.ipcp.cfg.nbns_entries[1]));

  return 0;
}

#endif

static struct cmdtab const ShowCommands[] = {
  {"auth", NULL, ShowAuthKey, LOCAL_AUTH,
  "Show auth details", "show auth"},
  {"ccp", NULL, ccp_ReportStatus, LOCAL_AUTH | LOCAL_CX_OPT,
  "Show CCP status", "show cpp"},
  {"compress", NULL, ReportCompress, LOCAL_AUTH,
  "Show compression stats", "show compress"},
  {"escape", NULL, ShowEscape, LOCAL_AUTH | LOCAL_CX,
  "Show escape characters", "show escape"},
  {"filter", NULL, ShowFilter, LOCAL_AUTH,
  "Show packet filters", "show filter [in|out|dial|alive]"},
  {"hdlc", NULL, hdlc_ReportStatus, LOCAL_AUTH | LOCAL_CX,
  "Show HDLC errors", "show hdlc"},
  {"ipcp", NULL, ReportIpcpStatus, LOCAL_AUTH,
  "Show IPCP status", "show ipcp"},
  {"lcp", NULL, lcp_ReportStatus, LOCAL_AUTH | LOCAL_CX_OPT,
  "Show LCP status", "show lcp"},
  {"links", "link", bundle_ShowLinks, LOCAL_AUTH,
  "Show available link names", "show links"},
  {"log", NULL, log_ShowLevel, LOCAL_AUTH,
  "Show log levels", "show log"},
  {"mem", NULL, ShowMemMap, LOCAL_AUTH,
  "Show memory map", "show mem"},
  {"modem", NULL, modem_ShowStatus, LOCAL_AUTH | LOCAL_CX,
  "Show modem setups", "show modem"},
#ifndef NOMSEXT
  {"msext", NULL, ShowMSExt, LOCAL_AUTH,
  "Show MS PPP extentions", "show msext"},
#endif
  {"proto", NULL, ShowProtocolStats, LOCAL_AUTH | LOCAL_CX_OPT,
  "Show protocol summary", "show proto"},
  {"reconnect", NULL, ShowReconnect, LOCAL_AUTH | LOCAL_CX,
  "Show reconnect timer", "show reconnect"},
  {"redial", NULL, ShowRedial, LOCAL_AUTH | LOCAL_CX,
  "Show Redial timeout", "show redial"},
  {"route", NULL, ShowRoute, LOCAL_AUTH,
  "Show routing table", "show route"},
  {"stopped", NULL, ShowStopped, LOCAL_AUTH | LOCAL_CX,
  "Show STOPPED timeout", "show stopped"},
  {"timeout", NULL, ShowTimeout, LOCAL_AUTH,
  "Show Idle timeout", "show timeout"},
  {"timers", NULL, ShowTimerList, LOCAL_AUTH,
  "Show alarm timers", "show timers"},
  {"version", NULL, ShowVersion, LOCAL_NO_AUTH | LOCAL_AUTH,
  "Show version string", "show version"},
  {"who", NULL, log_ShowWho, LOCAL_AUTH,
  "Show client list", "show who"},
  {"help", "?", HelpCommand, LOCAL_NO_AUTH | LOCAL_AUTH,
  "Display this message", "show help|? [command]", ShowCommands},
  {NULL, NULL, NULL},
};

static struct cmdtab const *
FindCommand(struct cmdtab const *cmds, const char *str, int *pmatch)
{
  int nmatch;
  int len;
  struct cmdtab const *found;

  found = NULL;
  len = strlen(str);
  nmatch = 0;
  while (cmds->func) {
    if (cmds->name && strncasecmp(str, cmds->name, len) == 0) {
      if (cmds->name[len] == '\0') {
	*pmatch = 1;
	return cmds;
      }
      nmatch++;
      found = cmds;
    } else if (cmds->alias && strncasecmp(str, cmds->alias, len) == 0) {
      if (cmds->alias[len] == '\0') {
	*pmatch = 1;
	return cmds;
      }
      nmatch++;
      found = cmds;
    }
    cmds++;
  }
  *pmatch = nmatch;
  return found;
}

static int
FindExec(struct bundle *bundle, struct cmdtab const *cmds, int argc,
         char const *const *argv, const char *prefix, struct prompt *prompt,
         struct datalink *cx)
{
  struct cmdtab const *cmd;
  int val = 1;
  int nmatch;
  struct cmdargs arg;

  cmd = FindCommand(cmds, *argv, &nmatch);
  if (nmatch > 1)
    LogPrintf(LogWARN, "%s%s: Ambiguous command\n", prefix, *argv);
  else if (cmd && (!prompt || (cmd->lauth & prompt->auth))) {
    if ((cmd->lauth & LOCAL_CX) && !cx)
      /* We've got no context, but we require it */
      cx = bundle2datalink(bundle, NULL);

    if ((cmd->lauth & LOCAL_CX) && !cx)
      LogPrintf(LogWARN, "%s%s: No context (use the `link' command)\n",
                prefix, *argv);
    else {
      if (cx && !(cmd->lauth & (LOCAL_CX|LOCAL_CX_OPT))) {
        LogPrintf(LogWARN, "%s%s: Redundant context (%s) ignored\n",
                  prefix, *argv, cx->name);
        cx = NULL;
      }
      arg.cmdtab = cmds;
      arg.cmd = cmd;
      arg.argc = argc-1;
      arg.argv = argv+1;
      arg.bundle = bundle;
      arg.cx = cx;
      arg.prompt = prompt;
      val = (cmd->func) (&arg);
    }
  } else
    LogPrintf(LogWARN, "%s%s: Invalid command\n", prefix, *argv);

  if (val == -1)
    LogPrintf(LogWARN, "Usage: %s\n", cmd->syntax);
  else if (val)
    LogPrintf(LogWARN, "%s%s: Failed %d\n", prefix, *argv, val);

  return val;
}

void
InterpretCommand(char *buff, int nb, int *argc, char ***argv)
{
  static char *vector[MAXARGS];
  char *cp;

  if (nb > 0) {
    cp = buff + strcspn(buff, "\r\n");
    if (cp)
      *cp = '\0';
    *argc = MakeArgs(buff, vector, VECSIZE(vector));
    *argv = vector;
  } else
    *argc = 0;
}

static int
arghidden(int argc, char const *const *argv, int n)
{
  /* Is arg n of the given command to be hidden from the log ? */

  /* set authkey xxxxx */
  /* set key xxxxx */
  if (n == 2 && !strncasecmp(argv[0], "se", 2) &&
      (!strncasecmp(argv[1], "authk", 5) || !strncasecmp(argv[1], "ke", 2)))
    return 1;

  /* passwd xxxxx */
  if (n == 1 && !strncasecmp(argv[0], "p", 1))
    return 1;

  return 0;
}

void
RunCommand(struct bundle *bundle, int argc, char const *const *argv,
           struct prompt *prompt, const char *label)
{
  if (argc > 0) {
    if (LogIsKept(LogCOMMAND)) {
      static char buf[LINE_LEN];
      int f, n;

      *buf = '\0';
      if (label) {
        strncpy(buf, label, sizeof buf - 3);
        buf[sizeof buf - 3] = '\0';
        strcat(buf, ": ");
      }
      n = strlen(buf);
      for (f = 0; f < argc; f++) {
        if (n < sizeof buf - 1 && f)
          buf[n++] = ' ';
        if (arghidden(argc, argv, f))
          strncpy(buf+n, HIDDEN, sizeof buf - n - 1);
        else
          strncpy(buf+n, argv[f], sizeof buf - n - 1);
        n += strlen(buf+n);
      }
      LogPrintf(LogCOMMAND, "%s\n", buf);
    }
    FindExec(bundle, Commands, argc, argv, "", prompt, NULL);
  }
}

void
DecodeCommand(struct bundle *bundle, char *buff, int nb, struct prompt *prompt,
              const char *label)
{
  int argc;
  char **argv;

  InterpretCommand(buff, nb, &argc, &argv);
  RunCommand(bundle, argc, (char const *const *)argv, prompt, label);
}

static int
ShowCommand(struct cmdargs const *arg)
{
  if (!arg->prompt)
    LogPrintf(LogWARN, "show: Cannot show without a prompt\n");
  else if (arg->argc > 0)
    FindExec(arg->bundle, ShowCommands, arg->argc, arg->argv, "show ",
             arg->prompt, arg->cx);
  else
    prompt_Printf(arg->prompt, "Use ``show ?'' to get a list.\n");

  return 0;
}

static int
TerminalCommand(struct cmdargs const *arg)
{
  if (!arg->prompt) {
    LogPrintf(LogWARN, "term: Need a prompt\n");
    return 1;
  }

  if (arg->cx->physical->link.lcp.fsm.state > ST_CLOSED) {
    prompt_Printf(arg->prompt, "LCP state is [%s]\n",
                  State2Nam(arg->cx->physical->link.lcp.fsm.state));
    return 1;
  }

  if (!IsInteractive(arg->prompt))
    return (1);

  datalink_Up(arg->cx, 0, 0);
  prompt_TtyTermMode(arg->prompt, arg->cx);
  return 0;
}

static int
QuitCommand(struct cmdargs const *arg)
{
  if (!arg->prompt || prompt_IsController(arg->prompt) ||
      (arg->argc > 0 && !strcasecmp(*arg->argv, "all") &&
       (arg->prompt->auth & LOCAL_AUTH)))
    Cleanup(EX_NORMAL);
  if (arg->prompt)
    prompt_Destroy(arg->prompt, 1);

  return 0;
}

static int
CloseCommand(struct cmdargs const *arg)
{
  bundle_Close(arg->bundle, arg->cx ? arg->cx->name : NULL, 1);
  return 0;
}

static int
DownCommand(struct cmdargs const *arg)
{
  datalink_Down(arg->cx, 1);
  return 0;
}

static int
SetModemSpeed(struct cmdargs const *arg)
{
  long speed;
  char *end;

  if (arg->argc > 0 && **arg->argv) {
    if (arg->argc > 1) {
      LogPrintf(LogWARN, "SetModemSpeed: Too many arguments");
      return -1;
    }
    if (strcasecmp(*arg->argv, "sync") == 0) {
      Physical_SetSync(arg->cx->physical);
      return 0;
    }
    end = NULL;
    speed = strtol(*arg->argv, &end, 10);
    if (*end) {
      LogPrintf(LogWARN, "SetModemSpeed: Bad argument \"%s\"", *arg->argv);
      return -1;
    }
    if (Physical_SetSpeed(arg->cx->physical, speed))
      return 0;
    LogPrintf(LogWARN, "%s: Invalid speed\n", *arg->argv);
  } else
    LogPrintf(LogWARN, "SetModemSpeed: No speed specified\n");

  return -1;
}

static int
SetReconnect(struct cmdargs const *arg)
{
  if (arg->argc == 2) {
    arg->cx->cfg.reconnect_timeout = atoi(arg->argv[0]);
    arg->cx->cfg.max_reconnect = (mode & MODE_DIRECT) ? 0 : atoi(arg->argv[1]);
    return 0;
  }
  return -1;
}

static int
SetRedialTimeout(struct cmdargs const *arg)
{
  int timeout;
  int tries;
  char *dot;

  if (arg->argc == 1 || arg->argc == 2) {
    if (strncasecmp(arg->argv[0], "random", 6) == 0 &&
	(arg->argv[0][6] == '\0' || arg->argv[0][6] == '.')) {
      arg->cx->cfg.dial_timeout = -1;
      randinit();
    } else {
      timeout = atoi(arg->argv[0]);

      if (timeout >= 0)
	arg->cx->cfg.dial_timeout = timeout;
      else {
	LogPrintf(LogWARN, "Invalid redial timeout\n");
	return -1;
      }
    }

    dot = strchr(arg->argv[0], '.');
    if (dot) {
      if (strcasecmp(++dot, "random") == 0) {
	arg->cx->cfg.dial_next_timeout = -1;
	randinit();
      } else {
	timeout = atoi(dot);
	if (timeout >= 0)
	  arg->cx->cfg.dial_next_timeout = timeout;
	else {
	  LogPrintf(LogWARN, "Invalid next redial timeout\n");
	  return -1;
	}
      }
    } else
      /* Default next timeout */
      arg->cx->cfg.dial_next_timeout = DIAL_NEXT_TIMEOUT;

    if (arg->argc == 2) {
      tries = atoi(arg->argv[1]);

      if (tries >= 0) {
	arg->cx->cfg.max_dial = tries;
      } else {
	LogPrintf(LogWARN, "Invalid retry value\n");
	return 1;
      }
    }
    return 0;
  }
  return -1;
}

static int
SetStoppedTimeout(struct cmdargs const *arg)
{
  struct link *l = &arg->cx->physical->link;

  l->lcp.fsm.StoppedTimer.load = 0;
  l->ccp.fsm.StoppedTimer.load = 0;
  if (arg->argc <= 2) {
    if (arg->argc > 0) {
      l->lcp.fsm.StoppedTimer.load = atoi(arg->argv[0]) * SECTICKS;
      if (arg->argc > 1)
        l->ccp.fsm.StoppedTimer.load = atoi(arg->argv[1]) * SECTICKS;
    }
    return 0;
  }
  return -1;
}

#define ismask(x) \
  (*x == '0' && strlen(x) == 4 && strspn(x+1, "0123456789.") == 3)

static int
SetServer(struct cmdargs const *arg)
{
  int res = -1;

  if (arg->argc > 0 && arg->argc < 4) {
    const char *port, *passwd, *mask;

    /* What's what ? */
    port = arg->argv[0];
    if (arg->argc == 2) {
      passwd = arg->argv[1];
      mask = NULL;
    } else if (arg->argc == 3) {
      passwd = arg->argv[1];
      mask = arg->argv[2];
      if (!ismask(mask))
        return -1;
    } else if (strcasecmp(port, "none") == 0) {
      if (mask != NULL || passwd != NULL)
        return -1;

      if (ServerClose(arg->bundle))
        LogPrintf(LogPHASE, "Disabled server port.\n");
      return 0;
    } else
      return -1;

    strncpy(server.passwd, passwd, sizeof server.passwd - 1);
    server.passwd[sizeof server.passwd - 1] = '\0';

    if (*port == '/') {
      mode_t imask;

      if (mask != NULL) {
	unsigned m;

	if (sscanf(mask, "%o", &m) == 1)
	  imask = m;
        else
          return -1;
      } else
        imask = (mode_t)-1;
      res = ServerLocalOpen(arg->bundle, port, imask);
    } else {
      int iport;

      if (mask != NULL)
        return -1;

      if (strspn(port, "0123456789") != strlen(port)) {
        struct servent *s;

        if ((s = getservbyname(port, "tcp")) == NULL) {
	  iport = 0;
	  LogPrintf(LogWARN, "%s: Invalid port or service\n", port);
	} else
	  iport = ntohs(s->s_port);
      } else
        iport = atoi(port);
      res = iport ? ServerTcpOpen(arg->bundle, iport) : -1;
    }
  }

  return res;
}

static int
SetModemParity(struct cmdargs const *arg)
{
  return arg->argc > 0 ? modem_SetParity(arg->cx->physical, *arg->argv) : -1;
}

static int
SetEscape(struct cmdargs const *arg)
{
  int code;
  int argc = arg->argc;
  char const *const *argv = arg->argv;

  for (code = 0; code < 33; code++)
    arg->cx->physical->async.cfg.EscMap[code] = 0;

  while (argc-- > 0) {
    sscanf(*argv++, "%x", &code);
    code &= 0xff;
    arg->cx->physical->async.cfg.EscMap[code >> 3] |= (1 << (code & 7));
    arg->cx->physical->async.cfg.EscMap[32] = 1;
  }
  return 0;
}

static struct in_addr
GetIpAddr(const char *cp)
{
  struct hostent *hp;
  struct in_addr ipaddr;

  if (inet_aton(cp, &ipaddr) == 0) {
    hp = gethostbyname(cp);
    if (hp && hp->h_addrtype == AF_INET)
      memcpy(&ipaddr, hp->h_addr, hp->h_length);
    else
      ipaddr.s_addr = 0;
  }
  return (ipaddr);
}

static int
SetInterfaceAddr(struct cmdargs const *arg)
{
  struct ipcp *ipcp = &arg->bundle->ncp.ipcp;
  const char *hisaddr;

  hisaddr = NULL;
  ipcp->cfg.my_range.ipaddr.s_addr = INADDR_ANY;
  ipcp->cfg.peer_range.ipaddr.s_addr = INADDR_ANY;

  if (arg->argc > 4)
    return -1;

  ipcp->cfg.HaveTriggerAddress = 0;
  ipcp->cfg.netmask.s_addr = INADDR_ANY;
  iplist_reset(&ipcp->cfg.peer_list);

  if (arg->argc > 0) {
    if (!ParseAddr(ipcp, arg->argc, arg->argv, &ipcp->cfg.my_range.ipaddr,
		   &ipcp->cfg.my_range.mask, &ipcp->cfg.my_range.width))
      return 1;
    if (arg->argc > 1) {
      hisaddr = arg->argv[1];
      if (arg->argc > 2) {
        ipcp->cfg.netmask = GetIpAddr(arg->argv[2]);
	if (arg->argc > 3) {
	  ipcp->cfg.TriggerAddress = GetIpAddr(arg->argv[3]);
	  ipcp->cfg.HaveTriggerAddress = 1;
	}
      }
    }
  }

  /*
   * For backwards compatibility, 0.0.0.0 means any address.
   */
  if (ipcp->cfg.my_range.ipaddr.s_addr == INADDR_ANY) {
    ipcp->cfg.my_range.mask.s_addr = INADDR_ANY;
    ipcp->cfg.my_range.width = 0;
  }
  ipcp->my_ip.s_addr = ipcp->cfg.my_range.ipaddr.s_addr;

  if (ipcp->cfg.peer_range.ipaddr.s_addr == INADDR_ANY) {
    ipcp->cfg.peer_range.mask.s_addr = INADDR_ANY;
    ipcp->cfg.peer_range.width = 0;
  }

  if (hisaddr && !UseHisaddr(arg->bundle, hisaddr, mode & MODE_AUTO))
    return 4;

  return 0;
}

#ifndef NOMSEXT

static void
SetMSEXT(struct ipcp *ipcp, struct in_addr * pri_addr,
	 struct in_addr * sec_addr, int argc, char const *const *argv)
{
  int dummyint;
  struct in_addr dummyaddr;

  if (!(mode & MODE_DIRECT)) {
    LogPrintf(LogWARN, "set nbns|ns: Only available in direct mode\n");
    return;
  }

  pri_addr->s_addr = sec_addr->s_addr = 0L;

  if (argc > 0) {
    ParseAddr(ipcp, argc, argv++, pri_addr, &dummyaddr, &dummyint);
    if (--argc > 0)
      ParseAddr(ipcp, argc, argv++, sec_addr, &dummyaddr, &dummyint);
    else
      sec_addr->s_addr = pri_addr->s_addr;
  }

  /*
   * if the primary/secondary ns entries are 0.0.0.0 we should set them to
   * either the localhost's ip, or the values in /etc/resolv.conf ??
   * 
   * up to you if you want to implement this...
   */

}

static int
SetNS(struct cmdargs const *arg)
{
  SetMSEXT(&arg->bundle->ncp.ipcp, &arg->bundle->ncp.ipcp.cfg.ns_entries[0],
           &arg->bundle->ncp.ipcp.cfg.ns_entries[1], arg->argc, arg->argv);
  return 0;
}

static int
SetNBNS(struct cmdargs const *arg)
{
  SetMSEXT(&arg->bundle->ncp.ipcp, &arg->bundle->ncp.ipcp.cfg.nbns_entries[0],
           &arg->bundle->ncp.ipcp.cfg.nbns_entries[1], arg->argc, arg->argv);
  return 0;
}

#endif				/* MS_EXT */

static int
SetVariable(struct cmdargs const *arg)
{
  u_long ulong_val;
  const char *argp;
  int param = (int)arg->cmd->args;
  struct datalink *cx = arg->cx;	/* AUTH_CX uses this */
  const char *err = NULL;
  struct link *l = ChooseLink(arg);	/* AUTH_CX_OPT uses this */

  if (arg->argc > 0)
    argp = *arg->argv;
  else
    argp = "";

  if ((arg->cmd->lauth & LOCAL_CX) && !cx) {
    LogPrintf(LogWARN, "set %s: No context (use the `link' command)\n",
              arg->cmd->name);
    return 1;
  } else if (cx && !(arg->cmd->lauth & (LOCAL_CX|LOCAL_CX_OPT))) {
    LogPrintf(LogWARN, "set %s: Redundant context (%s) ignored\n",
              arg->cmd->name, cx->name);
    cx = NULL;
  }

  switch (param) {
  case VAR_AUTHKEY:
    if (bundle_Phase(arg->bundle) == PHASE_DEAD) {
      strncpy(arg->bundle->cfg.auth.key, argp,
              sizeof arg->bundle->cfg.auth.key - 1);
      arg->bundle->cfg.auth.key[sizeof arg->bundle->cfg.auth.key - 1] = '\0';
    } else {
      err = "set authkey: Only available at phase DEAD\n";
      LogPrintf(LogWARN, err);
    }
    break;
  case VAR_AUTHNAME:
    if (bundle_Phase(arg->bundle) == PHASE_DEAD) {
      strncpy(arg->bundle->cfg.auth.name, argp,
              sizeof arg->bundle->cfg.auth.name - 1);
      arg->bundle->cfg.auth.name[sizeof arg->bundle->cfg.auth.name - 1] = '\0';
    } else {
      err = "set authname: Only available at phase DEAD\n";
      LogPrintf(LogWARN, err);
    }
    break;
  case VAR_DIAL:
    if (!(mode & (MODE_DIRECT|MODE_DEDICATED))) {
      strncpy(cx->cfg.script.dial, argp, sizeof cx->cfg.script.dial - 1);
      cx->cfg.script.dial[sizeof cx->cfg.script.dial - 1] = '\0';
    }
    break;
  case VAR_LOGIN:
    if (!(mode & (MODE_DIRECT|MODE_DEDICATED))) {
      strncpy(cx->cfg.script.login, argp, sizeof cx->cfg.script.login - 1);
      cx->cfg.script.login[sizeof cx->cfg.script.login - 1] = '\0';
    }
    break;
  case VAR_WINSIZE:
    if (arg->argc > 0) {
      l->ccp.cfg.deflate.out.winsize = atoi(arg->argv[0]);
      if (l->ccp.cfg.deflate.out.winsize < 8 ||
          l->ccp.cfg.deflate.out.winsize > 15) {
          LogPrintf(LogWARN, "%d: Invalid outgoing window size\n",
                    l->ccp.cfg.deflate.out.winsize);
          l->ccp.cfg.deflate.out.winsize = 15;
      }
      if (arg->argc > 1) {
        l->ccp.cfg.deflate.in.winsize = atoi(arg->argv[1]);
        if (l->ccp.cfg.deflate.in.winsize < 8 ||
            l->ccp.cfg.deflate.in.winsize > 15) {
            LogPrintf(LogWARN, "%d: Invalid incoming window size\n",
                      l->ccp.cfg.deflate.in.winsize);
            l->ccp.cfg.deflate.in.winsize = 15;
        }
      } else
        l->ccp.cfg.deflate.in.winsize = 0;
    } else {
      err = "No window size specified\n";
      LogPrintf(LogWARN, err);
    }
    break;
  case VAR_DEVICE:
    Physical_SetDeviceList(cx->physical, argp);
    break;
  case VAR_ACCMAP:
    if (arg->argc > 0) {
      sscanf(argp, "%lx", &ulong_val);
      cx->physical->link.lcp.cfg.accmap = ulong_val;
    } else {
      err = "No accmap specified\n";
      LogPrintf(LogWARN, err);
    }
    break;
  case VAR_MRU:
    ulong_val = atol(argp);
    if (ulong_val < MIN_MRU)
      err = "Given MRU value (%lu) is too small.\n";
    else if (ulong_val > MAX_MRU)
      err = "Given MRU value (%lu) is too big.\n";
    else
      l->lcp.cfg.mru = ulong_val;
    if (err)
      LogPrintf(LogWARN, err, ulong_val);
    break;
  case VAR_MTU:
    ulong_val = atol(argp);
    if (ulong_val == 0)
      l->lcp.cfg.mtu = 0;
    else if (ulong_val < MIN_MTU)
      err = "Given MTU value (%lu) is too small.\n";
    else if (ulong_val > MAX_MTU)
      err = "Given MTU value (%lu) is too big.\n";
    else
      l->lcp.cfg.mtu = ulong_val;
    if (err)
      LogPrintf(LogWARN, err, ulong_val);
    break;
  case VAR_OPENMODE:
    if (strcasecmp(argp, "active") == 0)
      cx->physical->link.lcp.cfg.openmode = arg->argc > 1 ?
        atoi(arg->argv[1]) : 1;
    else if (strcasecmp(argp, "passive") == 0)
      cx->physical->link.lcp.cfg.openmode = OPEN_PASSIVE;
    else {
      err = "%s: Invalid openmode\n";
      LogPrintf(LogWARN, err, argp);
    }
    break;
  case VAR_PHONE:
    strncpy(cx->cfg.phone.list, argp, sizeof cx->cfg.phone.list - 1);
    cx->cfg.phone.list[sizeof cx->cfg.phone.list - 1] = '\0';
    break;
  case VAR_HANGUP:
    if (!(mode & (MODE_DIRECT|MODE_DEDICATED))) {
      strncpy(cx->cfg.script.hangup, argp, sizeof cx->cfg.script.hangup - 1);
      cx->cfg.script.hangup[sizeof cx->cfg.script.hangup - 1] = '\0';
    }
    break;
  case VAR_IDLETIMEOUT:
    if (arg->argc > 1)
      err = "Too many idle timeout values\n";
    else if (arg->argc == 1)
      bundle_SetIdleTimer(arg->bundle, atoi(argp));
    if (err)
      LogPrintf(LogWARN, err);
    break;
  case VAR_LQRPERIOD:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid lqr period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      l->lcp.cfg.lqrperiod = ulong_val;
    break;
  case VAR_LCPRETRY:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid LCP FSM retry period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      cx->physical->link.lcp.cfg.fsmretry = ulong_val;
    break;
  case VAR_CHAPRETRY:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid CHAP retry period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      cx->chap.auth.cfg.fsmretry = ulong_val;
    break;
  case VAR_PAPRETRY:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid PAP retry period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      cx->pap.cfg.fsmretry = ulong_val;
    break;
  case VAR_CCPRETRY:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid CCP FSM retry period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      l->ccp.cfg.fsmretry = ulong_val;
    break;
  case VAR_IPCPRETRY:
    ulong_val = atol(argp);
    if (ulong_val <= 0) {
      err = "%s: Invalid IPCP FSM retry period\n";
      LogPrintf(LogWARN, err, argp);
    } else
      arg->bundle->ncp.ipcp.cfg.fsmretry = ulong_val;
    break;
  }

  return err ? 1 : 0;
}

static int 
SetCtsRts(struct cmdargs const *arg)
{
  if (arg->argc == 1) {
    if (strcmp(*arg->argv, "on") == 0)
      Physical_SetRtsCts(arg->cx->physical, 1);
    else if (strcmp(*arg->argv, "off") == 0)
      Physical_SetRtsCts(arg->cx->physical, 0);
    else
      return -1;
    return 0;
  }
  return -1;
}

static struct cmdtab const SetCommands[] = {
  {"accmap", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set accmap value", "set accmap hex-value", (const void *)VAR_ACCMAP},
  {"authkey", "key", SetVariable, LOCAL_AUTH,
  "Set authentication key", "set authkey|key key", (const void *)VAR_AUTHKEY},
  {"authname", NULL, SetVariable, LOCAL_AUTH,
  "Set authentication name", "set authname name", (const void *)VAR_AUTHNAME},
  {"ccpretry", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set FSM retry period", "set ccpretry value", (const void *)VAR_CCPRETRY},
  {"chapretry", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set CHAP retry period", "set chapretry value", (const void *)VAR_CHAPRETRY},
  {"ctsrts", "crtscts", SetCtsRts, LOCAL_AUTH | LOCAL_CX,
  "Use hardware flow control", "set ctsrts [on|off]"},
  {"deflate", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set deflate window sizes", "set deflate out-winsize in-winsize",
  (const void *) VAR_WINSIZE},
  {"device", "line", SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set modem device name", "set device|line device-name[,device-name]",
  (const void *) VAR_DEVICE},
  {"dial", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set dialing script", "set dial chat-script", (const void *) VAR_DIAL},
  {"encrypt", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Select CHAP encryption type", "set encrypt MSChap|MD5",
  (const void *)VAR_ENC},
  {"escape", NULL, SetEscape, LOCAL_AUTH | LOCAL_CX,
  "Set escape characters", "set escape hex-digit ..."},
  {"filter", NULL, SetFilter, LOCAL_AUTH,
  "Set packet filters", "set filter in|out|dial|alive ..."},
  {"hangup", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set hangup script", "set hangup chat-script", (const void *) VAR_HANGUP},
  {"ifaddr", NULL, SetInterfaceAddr, LOCAL_AUTH, "Set destination address",
  "set ifaddr [src-addr [dst-addr [netmask [trg-addr]]]]"},
  {"ipcpretry", NULL, SetVariable, LOCAL_AUTH,
  "Set FSM retry period", "set ipcpretry value", (const void *)VAR_IPCPRETRY},
  {"lcpretry", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set FSM retry period", "set lcpretry value", (const void *)VAR_LCPRETRY},
  {"log", NULL, log_SetLevel, LOCAL_AUTH,
  "Set log level", "set log [local] [+|-]value..."},
  {"login", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set login script", "set login chat-script", (const void *) VAR_LOGIN},
  {"lqrperiod", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set LQR period", "set lqrperiod value", (const void *)VAR_LQRPERIOD},
  {"mru", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set MRU value", "set mru value", (const void *)VAR_MRU},
  {"mtu", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX_OPT,
  "Set MTU value", "set mtu value", (const void *)VAR_MTU},
#ifndef NOMSEXT
  {"nbns", NULL, SetNBNS, LOCAL_AUTH,
  "Set NetBIOS NameServer", "set nbns pri-addr [sec-addr]"},
  {"ns", NULL, SetNS, LOCAL_AUTH,
  "Set NameServer", "set ns pri-addr [sec-addr]"},
#endif
  {"openmode", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX, "Set open mode",
  "set openmode active|passive [secs]", (const void *)VAR_OPENMODE},
  {"papretry", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX,
  "Set PAP retry period", "set papretry value", (const void *)VAR_PAPRETRY},
  {"parity", NULL, SetModemParity, LOCAL_AUTH | LOCAL_CX,
  "Set modem parity", "set parity [odd|even|none]"},
  {"phone", NULL, SetVariable, LOCAL_AUTH | LOCAL_CX, "Set telephone number(s)",
  "set phone phone1[:phone2[...]]", (const void *)VAR_PHONE},
  {"reconnect", NULL, SetReconnect, LOCAL_AUTH | LOCAL_CX,
  "Set Reconnect timeout", "set reconnect value ntries"},
  {"redial", NULL, SetRedialTimeout, LOCAL_AUTH | LOCAL_CX,
  "Set Redial timeout", "set redial value|random[.value|random] [attempts]"},
  {"server", "socket", SetServer, LOCAL_AUTH,
  "Set server port", "set server|socket TcpPort|LocalName|none [mask]"},
  {"speed", NULL, SetModemSpeed, LOCAL_AUTH | LOCAL_CX,
  "Set modem speed", "set speed value"},
  {"stopped", NULL, SetStoppedTimeout, LOCAL_AUTH | LOCAL_CX,
  "Set STOPPED timeouts", "set stopped [LCPseconds [CCPseconds]]"},
  {"timeout", NULL, SetVariable, LOCAL_AUTH, "Set Idle timeout",
  "set timeout idletime", (const void *)VAR_IDLETIMEOUT},
  {"vj", NULL, SetInitVJ, LOCAL_AUTH,
  "Set vj values", "set vj slots|slotcomp [value]"},
  {"weight", NULL, mp_SetDatalinkWeight, LOCAL_AUTH | LOCAL_CX,
  "Set datalink weighting", "set weight n"},
  {"help", "?", HelpCommand, LOCAL_AUTH | LOCAL_NO_AUTH,
  "Display this message", "set help|? [command]", SetCommands},
  {NULL, NULL, NULL},
};

static int
SetCommand(struct cmdargs const *arg)
{
  if (arg->argc > 0)
    FindExec(arg->bundle, SetCommands, arg->argc, arg->argv, "set ",
             arg->prompt, arg->cx);
  else if (arg->prompt)
    prompt_Printf(arg->prompt, "Use `set ?' to get a list or `set ? <var>' for"
	    " syntax help.\n");
  else
    LogPrintf(LogWARN, "set command must have arguments\n");

  return 0;
}


static int
AddCommand(struct cmdargs const *arg)
{
  struct in_addr dest, gateway, netmask;
  int gw;

  if (arg->argc != 3 && arg->argc != 2)
    return -1;

  if (arg->argc == 2)
    if (strcasecmp(arg->argv[0], "default"))
      return -1;
    else {
      dest.s_addr = netmask.s_addr = INADDR_ANY;
      gw = 1;
    }
  else {
    if (strcasecmp(arg->argv[0], "MYADDR") == 0)
      dest = arg->bundle->ncp.ipcp.my_ip;
    else if (strcasecmp(arg->argv[0], "HISADDR") == 0)
      dest = arg->bundle->ncp.ipcp.peer_ip;
    else
      dest = GetIpAddr(arg->argv[0]);
    netmask = GetIpAddr(arg->argv[1]);
    gw = 2;
  }
  if (strcasecmp(arg->argv[gw], "HISADDR") == 0)
    gateway = arg->bundle->ncp.ipcp.peer_ip;
  else if (strcasecmp(arg->argv[gw], "INTERFACE") == 0)
    gateway.s_addr = INADDR_ANY;
  else
    gateway = GetIpAddr(arg->argv[gw]);
  bundle_SetRoute(arg->bundle, RTM_ADD, dest, gateway, netmask,
                  arg->cmd->args ? 1 : 0);
  return 0;
}

static int
DeleteCommand(struct cmdargs const *arg)
{
  struct in_addr dest, none;

  if (arg->argc == 1)
    if(strcasecmp(arg->argv[0], "all") == 0)
      DeleteIfRoutes(arg->bundle, 0);
    else {
      if (strcasecmp(arg->argv[0], "MYADDR") == 0)
        dest = arg->bundle->ncp.ipcp.my_ip;
      else if (strcasecmp(arg->argv[0], "default") == 0)
        dest.s_addr = INADDR_ANY;
      else
        dest = GetIpAddr(arg->argv[0]);
      none.s_addr = INADDR_ANY;
      bundle_SetRoute(arg->bundle, RTM_DELETE, dest, none, none,
                      arg->cmd->args ? 1 : 0);
    }
  else
    return -1;

  return 0;
}

#ifndef NOALIAS
static struct cmdtab const AliasCommands[] =
{
  {"addr", NULL, AliasRedirectAddr, LOCAL_AUTH,
   "static address translation", "alias addr [addr_local addr_alias]"},
  {"deny_incoming", NULL, AliasOption, LOCAL_AUTH,
   "stop incoming connections", "alias deny_incoming [yes|no]",
   (const void *) PKT_ALIAS_DENY_INCOMING},
  {"enable", NULL, AliasEnable, LOCAL_AUTH,
   "enable IP aliasing", "alias enable [yes|no]"},
  {"log", NULL, AliasOption, LOCAL_AUTH,
   "log aliasing link creation", "alias log [yes|no]",
   (const void *) PKT_ALIAS_LOG},
  {"port", NULL, AliasRedirectPort, LOCAL_AUTH,
   "port redirection", "alias port [proto addr_local:port_local  port_alias]"},
  {"same_ports", NULL, AliasOption, LOCAL_AUTH,
   "try to leave port numbers unchanged", "alias same_ports [yes|no]",
   (const void *) PKT_ALIAS_SAME_PORTS},
  {"unregistered_only", NULL, AliasOption, LOCAL_AUTH,
   "alias unregistered (private) IP address space only",
   "alias unregistered_only [yes|no]",
   (const void *) PKT_ALIAS_UNREGISTERED_ONLY},
  {"use_sockets", NULL, AliasOption, LOCAL_AUTH,
   "allocate host sockets", "alias use_sockets [yes|no]",
   (const void *) PKT_ALIAS_USE_SOCKETS},
  {"help", "?", HelpCommand, LOCAL_AUTH | LOCAL_NO_AUTH,
   "Display this message", "alias help|? [command]", AliasCommands},
  {NULL, NULL, NULL},
};


static int
AliasCommand(struct cmdargs const *arg)
{
  if (arg->argc > 0)
    FindExec(arg->bundle, AliasCommands, arg->argc, arg->argv, "alias ",
             arg->prompt, arg->cx);
  else if (arg->prompt)
    prompt_Printf(arg->prompt, "Use `alias help' to get a list or `alias help"
            " <option>' for syntax help.\n");
  else
    LogPrintf(LogWARN, "alias command must have arguments\n");

  return 0;
}

static int
AliasEnable(struct cmdargs const *arg)
{
  if (arg->argc == 1)
    if (strcasecmp(arg->argv[0], "yes") == 0) {
      if (!(mode & MODE_ALIAS)) {
	if (loadAliasHandlers(&VarAliasHandlers) == 0) {
	  mode |= MODE_ALIAS;
	  return 0;
	}
	LogPrintf(LogWARN, "Cannot load alias library\n");
	return 1;
      }
      return 0;
    } else if (strcasecmp(arg->argv[0], "no") == 0) {
      if (mode & MODE_ALIAS) {
	unloadAliasHandlers();
	mode &= ~MODE_ALIAS;
      }
      return 0;
    }
  return -1;
}


static int
AliasOption(struct cmdargs const *arg)
{
  unsigned param = (unsigned)arg->cmd->args;
  if (arg->argc == 1)
    if (strcasecmp(arg->argv[0], "yes") == 0) {
      if (mode & MODE_ALIAS) {
	VarPacketAliasSetMode(param, param);
	return 0;
      }
      LogPrintf(LogWARN, "alias not enabled\n");
    } else if (strcmp(arg->argv[0], "no") == 0) {
      if (mode & MODE_ALIAS) {
	VarPacketAliasSetMode(0, param);
	return 0;
      }
      LogPrintf(LogWARN, "alias not enabled\n");
    }
  return -1;
}
#endif /* #ifndef NOALIAS */

static struct cmdtab const AllowCommands[] = {
  {"modes", "mode", AllowModes, LOCAL_AUTH,
  "Only allow certain ppp modes", "allow modes mode..."},
  {"users", "user", AllowUsers, LOCAL_AUTH,
  "Allow users access to ppp", "allow users logname..."},
  {"help", "?", HelpCommand, LOCAL_AUTH | LOCAL_NO_AUTH,
  "Display this message", "allow help|? [command]", AllowCommands},
  {NULL, NULL, NULL},
};

static int
AllowCommand(struct cmdargs const *arg)
{
  /* arg->bundle may be NULL (see ValidSystem()) ! */
  if (arg->argc > 0)
    FindExec(arg->bundle, AllowCommands, arg->argc, arg->argv, "allow ",
             arg->prompt, arg->cx);
  else if (arg->prompt)
    prompt_Printf(arg->prompt, "Use `allow ?' to get a list or `allow ? <cmd>'"
                  " for syntax help.\n");
  else
    LogPrintf(LogWARN, "allow command must have arguments\n");

  return 0;
}

static int
LinkCommand(struct cmdargs const *arg)
{
  if (arg->argc > 1) {
    struct datalink *cx = bundle2datalink(arg->bundle, arg->argv[0]);
    if (cx)
      FindExec(arg->bundle, Commands, arg->argc - 1, arg->argv + 1, "",
               arg->prompt, cx);
    else {
      LogPrintf(LogWARN, "link: %s: Invalid link name\n", arg->argv[0]);
      return 1;
    }
  } else {
    LogPrintf(LogWARN, "Usage: %s\n", arg->cmd->syntax);
    return 2;
  }

  return 0;
}

struct link *
ChooseLink(struct cmdargs const *arg)
{
  if (arg->cx)
    return &arg->cx->physical->link;
  else if (arg->bundle->ncp.mp.active)
    return &arg->bundle->ncp.mp.link;
  else {
    struct datalink *dl = bundle2datalink(arg->bundle, NULL);
    return dl ? &dl->physical->link : NULL;
  }
}
