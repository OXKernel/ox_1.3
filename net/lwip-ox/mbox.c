#include "lwipopts.h"
#include "arch/cc.h"
#include "arch/sys_arch.h"
#include "lwip/err.h"
#include "lwip/sys.h"

err_t
sys_mbox_new(sys_mbox_t * mbox, int size)
{
	return ERR_MEM;
}

void
sys_mbox_free(sys_mbox_t * mbox)
{
}

void
sys_mbox_post(sys_mbox_t * mbox, void *msg)
{
}

err_t
sys_mbox_trypost(sys_mbox_t * mbox, void *msg)
{
	return ERR_MEM;
}

u32_t
sys_arch_mbox_fetch(sys_mbox_t * mbox, void **msg, u32_t timeout)
{
	return SYS_ARCH_TIMEOUT;
}

u32_t
sys_arch_mbox_tryfetch(sys_mbox_t * mbox, void **msg)
{
	return SYS_MBOX_EMPTY;
}

int
sys_mbox_valid(sys_mbox_t * mbox)
{
	return *mbox == SYS_MBOX_NULL;
}

void
sys_mbox_set_invalid(sys_mbox_t * mbox)
{
	*mbox = SYS_MBOX_NULL;
}
