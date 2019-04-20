static inline
void
usleep(unsigned long us)
{
	while (us--) {
		/* 1µs delay */
		__asm__ __volatile__ (
			"outb %%al,$0x80"
			:::"memory");
	}
}

static inline
void
msleep(unsigned long ms)
{
	usleep(1000 * ms);
}

