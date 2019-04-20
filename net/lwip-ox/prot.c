#include <asm_core/util.h>
#include "arch/cc.h"
#include "arch/sys_arch.h"

/* FIXME not SMP-safe, reimplement */

static sys_prot_t prot = 0;

void
sys_arch_unprotect(sys_prot_t pval)
{
	if (!pval)
		asm_enable_interrupt();
	prot = pval;
}

sys_prot_t
sys_arch_protect(void)
{
	asm_disable_interrupt();
	return prot++;
}
