/* Memory management for LwIP
 */

#include <ox/mm/malloc.h>
#include "lwip/mem.h"

#if 0 // Defined in mem.c using kmalloc from CLIB (ox/page.c)
void *
mem_trim(void *mem, mem_size_t size)
{
	return mem;
}

void *
mem_malloc(mem_size_t size)
{
	return kmalloc(size);
}

void *
mem_calloc(mem_size_t count, mem_size_t size)
{
	return kzalloc(count * size);
}

void
mem_free(void *mem)
{
	kfree(mem);
}
#endif
