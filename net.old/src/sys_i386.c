/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include <fork.h>
#include "lwip/sys.h"
#include "lwip/def.h"

static unsigned long long ticks=0;
unsigned char dummy_msg[]={0xff,0xff,0xff,0xff,
		0xff,0xff,0xff,0xff,
		0xff,0xff,0xff,0xff,
		0xff,0xff,0xff,0xff,
		0xff,0xff,0xff,0xff,
		0xff,0xff,0xff,0xff};

typedef void (*CB_FUNC)(void *);
#define CB_FUNC_ENTRIES (16)
static struct CB_FUNC_ARRAY
{
	CB_FUNC func;
	void *arg;
}cb_func_array[CB_FUNC_ENTRIES]={0};

struct sys_timeouts
{
	long long v;
	struct sys_timeouts *pNext;
}timeouts;

struct sys_semaphore
{
	unsigned int v;
};

struct sys_message_node
{
	void * msg;
	struct sys_message_node * pNext;
};

void socket_timer()
{
	ticks++;
	unsigned int i;
    //printk("line %d file %s\n",__LINE__,__FILE__);
	for(i=0;i<CB_FUNC_ENTRIES;i++)
	{
        //printk("line %d file %s i=%d\n",__LINE__,__FILE__,i);
		if(cb_func_array[i].func) cb_func_array[i].func(cb_func_array[i].arg);
	}
    //printk("line %d file %s\n",__LINE__,__FILE__);
    //ne2k_isr(5); // RGD: Must pass in our IRQ : This is supposed to be
    //                     called on interrupt, not polled.
	// FIXME This should be called from the loop -- Ismael
	int ne_handler();
	ne_handler(); // RGD
}

static void schedule(){ return;}
unsigned long long get_current_tick()
{
	return ticks;
}


/*-----------------------------------------------------------------------------------*/
void
sys_arch_block(u32_t time)
{
  long long t0,t1;
  t0=get_current_tick();
  
  do{
	  schedule();
	  t1=get_current_tick();
  }while((t1-t0)<time);
}
/*-----------------------------------------------------------------------------------*/
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	struct sys_message_node *pHead;
	pHead=(struct sys_message_node *)kmalloc(sizeof(struct sys_message_node));
	if(pHead==0)  return ERR_VAL;
	*mbox=pHead;
	pHead->pNext=0;
	return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
void sys_mbox_free(sys_mbox_t *mbox)
{
	struct sys_message_node *pHead=*mbox;
	struct sys_message_node *pTemp;
	while(pHead->pNext!=0)
	{
		pTemp=pHead;
		pHead=pHead->pNext;
		free(pTemp);
	}
	free(*pHead);
  return;
}
/*-----------------------------------------------------------------------------------*/
void
sys_mbox_post(sys_mbox_t *mbox, void *data)
{
	struct sys_message_node *pHead=*mbox;
	struct sys_message_node *pTemp;
	pTemp=(struct sys_message_node *)kmalloc(sizeof(struct sys_message_node));
	while(0==pTemp){
		schedule();
		pTemp=(struct sys_message_node *)kmalloc(sizeof(struct sys_message_node));
	}
	pTemp->msg=data;
	pTemp->pNext=0;
	asm_disable_interrupt();
	while(pHead->pNext!=0)pHead=pHead->pNext;
	pHead->pNext=pTemp;
	asm_enable_interrupt();
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	struct sys_message_node *pHead=*mbox;
	struct sys_message_node *pTemp;
	pTemp=(struct sys_message_node *)kmalloc(sizeof(struct sys_message_node));
	if(pTemp==0) return ERR_VAL;
	pTemp->msg=msg;
	pTemp->pNext=0;
	asm_disable_interrupt();
	while(pHead->pNext!=0)pHead=pHead->pNext;
	pHead->pNext=pTemp;
	asm_enable_interrupt();
	return ERR_OK;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	struct sys_message_node *pHead=*mbox;
	struct sys_message_node *pTemp;
	if(0==pHead->pNext) {
		*msg=dummy_msg;
		return SYS_MBOX_EMPTY;
	}
	*msg=pHead->pNext->msg;
	pTemp= pHead->pNext->pNext;
	free(pHead->pNext);
	pHead->pNext=pTemp;
	return 0;
}

/*-----------------------------------------------------------------------------------*/
u32_t
sys_arch_mbox_fetch(sys_mbox_t *mbox, void **data, u32_t timeout)
{
	struct sys_message_node *pHead=*mbox;
	struct sys_message_node *pTemp;
	long long t0,t1;
	int flag=0;
	t0=get_current_tick();

	while(0==pHead->pNext){
		t1=get_current_tick();
		if((timeout !=0)&&((t1-t0)>timeout)){
			flag=1;
			break;
		}
		schedule();
	}
	if(flag) return SYS_ARCH_TIMEOUT;
	*data=pHead->pNext->msg;
	pTemp= pHead->pNext->pNext;
	free(pHead->pNext);
	pHead->pNext=pTemp;
	return get_current_tick()-t0;
}

/*-----------------------------------------------------------------------------------*/
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	struct sys_semaphore * tmp=(struct sys_semaphore *)kmalloc(sizeof(struct sys_semaphore));
	if(tmp==0) return ERR_VAL;
	tmp->v=count;
	*sem = (sys_sem_t *)tmp;
	return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
u32_t
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	struct sys_semaphore * tmp=*(struct sys_semaphore **)sem;
	long long t0,t1;
	int got_sem=0;
	t0=get_current_tick();
	do{
		asm_disable_interrupt();
		if(tmp->v>0){
			tmp->v--;
			got_sem=1;
		}
		asm_enable_interrupt();
		if(got_sem) return 0;
		t1=get_current_tick();
		if(timeout&&((t1-t0)>timeout)) break;
		schedule();
	}while(1);
  return SYS_ARCH_TIMEOUT;
}
/*-----------------------------------------------------------------------------------*/
void
sys_sem_signal(sys_sem_t *sem)
{
	struct sys_semaphore * tmp=*(struct sys_semaphore **)sem;
	tmp->v++;
	return;
}
/*-----------------------------------------------------------------------------------*/
void
sys_sem_free(sys_sem_t *sem)
{
	free(*sem);
  return;
}
/*-----------------------------------------------------------------------------------*/
void
sys_init(void)
{
  return;
}
/*-----------------------------------------------------------------------------------*/
struct sys_timeouts *
sys_arch_timeouts(void)
{
  return &timeouts;
}
/*-----------------------------------------------------------------------------------*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
	unsigned int i;
	for(i=0;i<CB_FUNC_ENTRIES;i++)
	{
		if(0==cb_func_array[i].func){
			cb_func_array[i].func = (CB_FUNC)thread;
			cb_func_array[i].arg=arg;
			return 0;
		}
	}
	while(1);

    return 0;
}
/*-----------------------------------------------------------------------------------*/
