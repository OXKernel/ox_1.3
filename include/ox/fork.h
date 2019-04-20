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
#ifndef _FORK_H
#define _FORK_H
/********************************************************
 * Copyright (C)  Roger George Doss. All Rights Reserved.
 ********************************************************
 *
 *  @module   
 *	fork.h
 *      contains fork system call.
 *
 ********************************************************/
#include <platform/cpu_ctx.h>

void create_init_task();
int kthread(const char *name, void* ip, void *arg, int stacksize, int prio);
int kfork(struct cpu_ctx ctx);

#endif
