#include<stdio.h>

#include<comp421/hardware.h>

void KernelStart(ExceptionStackFrame *frame, Unsigned int pmem_size, void *org_brk, char **cmd_args)
{
	int i;
	void (*vector_table[TRAP_VEACTOR_SIZE])(ExceptionStackFrame *);
	for ( i=0;i<TRAP_VECTOR_SIZE;i++)
		vector_table[i]=NULL;
	WriteRegister(REG_VECTOR_BASE,(RCS421RegVal)vector_table);

}
