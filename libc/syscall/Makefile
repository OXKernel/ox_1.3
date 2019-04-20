##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
# makefile for syscall
#
##########################################################
AR       =  ar
AS       =  as
CC       =  gcc
LD       =  ld
ARFLAGS  =  rcs
LDFLAGS  =  -s -x
IFLAGS   =  -nostdinc -I../../include
CFLAGS	 =  $(IFLAGS) -MD -D_POSIX_SOURCE -Wall -O -fstrength-reduce \
	    -fomit-frame-pointer -finline-functions -m32

# This was broken:
#.c.s:  $(CC) $(CFLAGS)  -S -o $*.s $<
#.s.o:  $(AS) -c -o $*.o $<
#.c.o:  $(CC) $(CFLAGS) -c -o $*.o $<

OBJS = \
	access.o \
	acct.o \
	alarm.o \
	brk.o \
	chdir.o \
	fchdir.o \
	chmod.o \
	chown.o \
	fchown.o \
	lchown.o \
	chroot.o \
	close.o \
	creat.o \
	dup.o \
	dup2.o \
	errno.o \
	execl.o \
	execle.o \
	execlp.o \
	execv.o \
	execve.o \
	execvp.o \
	exit.o \
	fcntl.o \
	fchmod.o \
	fork.o \
	fstat.o \
	ftime.o \
	getcwd.o \
	getwd.o \
	getcurrentdirname.o \
	getegid.o \
	geteuid.o \
	getgid.o \
	getpgid.o \
	getpgrp.o \
	getpid.o \
	getppid.o \
	getuid.o \
	ioctl.o \
	kill.o \
	link.o \
	lseek.o \
	mkdir.o \
	mknod.o \
	mount.o \
	nanosleep.o \
	nice.o \
	open.o \
	pause.o \
	pipe.o \
	ptrace.o \
	raise.o \
	read.o \
	rename.o \
	opendir.o \
	closedir.o \
	rewinddir.o \
	readdir.o \
	reboot.o \
	rmdir.o \
	sbrk.o \
	setegid.o \
	seteuid.o \
	setgid.o \
	setpgid.o \
	setpgrp.o \
	setregid.o \
	setreuid.o \
	setsid.o \
	setuid.o \
	sigaction.o \
	sigaddset.o \
	sigdelset.o \
	sigemptyset.o \
	sigfillset.o \
	sigismember.o \
	signal.o \
	sigpending.o \
	sigprocmask.o \
	sigreturn.o \
	sigsuspend.o \
	sleep.o \
	stat.o \
	lstat.o \
	stime.o \
	sync.o \
	time.o \
	times.o \
	umask.o \
	umount.o \
	uname.o \
	unlink.o \
	ustat.o \
	utime.o \
	utimes.o \
	wait.o \
	waitpid.o \
	write.o \
	mktime.o

.PHONY: all clean
all:   $(OBJS)
clean:
	rm -f core $(OBJS) $(OBJS:.o=.d)

### Dependencies:
-include $(OBJS:.o=.d)
