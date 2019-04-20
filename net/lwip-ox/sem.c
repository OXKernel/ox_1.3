/* Simple binary semaphores */

#include "arch/cc.h"
#include "arch/sys_arch.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* FIXME not SMP-safe, reimplement */

static struct {
	u32_t map;
	u32_t sem;
} sempool;

#define SEMPOOL_SIZE (sizeof(sempool.sem) * 8);

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
	/* Find next free semaphore */
	s8_t i = sizeof(sempool.sem) * 8;
	while (i > 0 && (sempool.map & (1 << --i)));
	if (i < 0)
		return ERR_MEM;
	*sem = i;

	u32_t v = 1 << i;
	sempool.map |= v;
	if (count)
		sempool.sem |= v;
	else
		sempool.sem &= ~v;
	return ERR_OK;
}

void
sys_sem_free(sys_sem_t *sem)
{
	sempool.map &= ~(1 << *sem);
}

void
sys_sem_signal(sys_sem_t *sem)
{
	sempool.sem |= (1 << *sem);
}

u32_t
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	if (sempool.sem & (1 << *sem))
		return 0;
	// FIXME
	return SYS_ARCH_TIMEOUT;
}

int
sys_sem_valid(sys_sem_t *sem)
{
	return *sem >= 0 && *sem < SEMPOOL_SIZE;
}

void
sys_sem_set_invalid(sys_sem_t * sem)
{
	*sem = -1;
}
