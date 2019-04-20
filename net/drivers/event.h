#include <string.h>

struct event {
	int signaled;
};

static inline
void
init_event(struct event *e, int manual_reset, int initial_state)
{
	e->signaled = initial_state;
}

static inline
void
set_event(struct event *e)
{
	e->signaled = 1;
}

static inline
void
reset_event(struct event *e)
{
	e->signaled = 0;
}

static inline
int
wait_for_object(struct event *e, unsigned int timeout)
{
	/* FIXME basically we spinlock, and that's bad */
	do {
        enable_irq(0); // PIT
        enable_irq(5); // NET RGD:: TODO We may need to poll ne_handler
		__asm__ __volatile__ ("pause":::"memory");
	} while (!e->signaled);
	return 0;
}
