__attribute__((noreturn))
void halt(void)
{
	for (;;)
		__asm__ __volatile__ ("hlt");
}
