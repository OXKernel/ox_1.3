#include <sys/types.h>

#define SYS_MBOX_NULL NULL
#define SYS_SEM_NULL -1

struct sys_sem; // Defined in sys_i386.c
struct sys_message_node;
typedef struct sys_sem * sys_sem_t;
typedef struct sys_sem * sys_mutex_t;
typedef struct sys_message_node * sys_mbox_t;
typedef void * sys_thread_t;
typedef unsigned int sys_prot_t;
