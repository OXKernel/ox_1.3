/*

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/
/********************************************************
 * Copyright (C)  Roger George Doss. All Rights Reserved.
 ********************************************************
 *
 *  signal.c
 *   signal C routine
 *
 ********************************************************/
#include <unistd.h>
#include <signal.h>
#include <platform/call.h>
#include <platform/syscall.h>

/*
 * void  (*signal(int signum, void (*handler)(int)))(int);
 */
_syscall_2(int,signal,int,signum,sighandler_t,handler);