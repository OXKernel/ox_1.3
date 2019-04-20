#include <ox/ktime.h>

#include "arch/cc.h"
#include "arch/sys_arch.h"

#if 0
void
sys_init(void)
{
}

sys_thread_t
sys_thread_new(char *name, void (*thread) (void *arg), void *arg, int stacksize,
	       int prio)
{
}
#endif

u32_t
LWIP_RAND(void)
{
    u32_t t = ktime(NULL);
    u32_t x = ktime(NULL);
    if(t > x) {
        return (t - x) ^ 11789;
    } else {
        return (x - t) ^ 19734;
    }
}

u32_t
sys_now(void)
{
        return ktime(NULL);
}

/* called from ox_main */
void
network_init(void)
{
	ne2k_enable(0); // RGD: Test code.
    asm_enable_interrupt();
    socket_test();
    ne2k_disable();
    asm_disable_interrupt();
}
