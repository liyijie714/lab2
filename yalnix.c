#include<stdlib.h>

#include "hardware.h"
#include "yalnix.h"

void *mod_brk;
int head;
static void (*vector_table[TRAP_VECTOR_SIZE])(ExceptionStackFrame *);
void KernelStart(ExceptionStackFrame *frame, unsigned int pmem_size, void *org_brk, char **cmd_args)
{
	int i,addr;
	struct pte *pt1,*pt0;
	mod_brk=org_brk;
	WriteRegister(REG_VECTOR_BASE,(RCS421RegVal)vector_table);
	addr=0;
	for(i=1;i<PMEM_BASE+pmem_size;i=i+PAGESIZE)
	{
		if((i*PAGESIZE < KERNEL_STACK_BASE && ((i+1)*PAGESIZE)-1 < KERNEL_STACK_BASE)||(i*PAGESIZE > (long)mod_brk && ((i+1)*PAGESIZE)-1 < PMEM_BASE+pmem_size))
		{	*(int *)(long)addr=(i*PAGESIZE);
			addr=(i*PAGESIZE);
		}
			
	}
	pt0=(struct pte *)malloc(PAGE_TABLE_LEN * sizeof(struct pte));
	pt1=(struct pte *)malloc(PAGE_TABLE_LEN * sizeof(struct pte));

	WriteRegister(REG_PTR0,(RCS421RegVal)pt1);
	WriteRegister(REG_PTR1,(RCS421RegVal)pt0);
	
	addr=VMEM_1_BASE;
	for(i=0;i< PAGE_TABLE_LEN;i++)
	{	
		if(addr>=VMEM_1_BASE && addr < (long)&_etext)
		{
			pt1[i].pfn = i;
			pt1[i].uprot=0;
			pt1[i].kprot=PROT_READ|PROT_EXEC;
			pt1[i].valid=1;
		}
		else if (addr>= (long)&_etext && addr < (long)mod_brk)
		{
			pt1[i].pfn = i;
			pt1[i].uprot=0;
			pt1[i].kprot=PROT_READ|PROT_WRITE;
			pt1[i].valid=1;

		}
		else
			pt1[i].valid=0;
		pt0[i].valid=0;
		addr=addr+PAGESIZE;
	}

	WriteRegister(REG_VM_ENABLE, 1);
}

int SetKernelBrk(void *addr)
{
	if((long)addr>=VMEM_1_LIMIT)
		return -1;

	mod_brk=addr;
	return 0;
}
