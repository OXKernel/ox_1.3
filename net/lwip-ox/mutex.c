#include "arch/cc.h"
#include "arch/sys_arch.h"

void
sys_mutex_new(sys_mutex_t * mutex)
{
    sys_sem_new(mutex,1);
}

void
sys_mutex_free(sys_mutex_t * mutex)
{
    sys_sem_free(mutex);
}

void
sys_mutex_lock(sys_mutex_t * mutex)
{
    sys_arch_sem_wait(mutex,5); // RGD
}

void
sys_mutex_unlock(sys_mutex_t * mutex)
{
    sys_sem_signal(mutex);
}

int
sys_mutex_valid(sys_mutex_t * mutex)
{
	return !!*mutex;
}

void
sys_mutex_set_invalid(sys_mutex_t * mutex)
{
	*mutex = 0;
}
