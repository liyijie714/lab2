#include<stdlib.h>

#include<hardware.h>
#include<yalnix.h>

void *brk;
int head;
void KernelStart(ExceptionStackFrame *frame, Unsigned int pmem_size, void *org_brk, char **cmd_args)
{
	int i,addr;
	
	void (*vector_table[TRAP_VEACTOR_SIZE])(ExceptionStackFrame *);
	struct pte *pt1,*pt2;
	brk=org_brk;
	for ( i=0;i<TRAP_VECTOR_SIZE;i++)
		vector_table[i]=NULL;
	WriteRegister(REG_VECTOR_BASE,(RCS421RegVal)vector_table);
	WriteRegister(VMEM_BASE,(RCS421RegVal)PMEM_BASE);
	WriteRegister(VMEM_0_BASE,(RCS421RegVal)PMEM_BASE);
	WriteRegister(VMEM_SIZE,(RCS421RegVal)pmem_size);
	WriteRegister(VMEM_0_LIMI,(RCS421RegVal)(PMEM_BASE+pmem_size)/2);
	WriteRegister(VMEM_1_BASE,(RCS421RegVal)(PMEM_BASE+pmem_size)/2);
	WriteRegister(VMEM_1_LIMI,(RCS421RegVal)(PMEM_BASE+pmem_size));
	addr=0;
	pt1=(struct pte *)malloc(PAGE_TABLE_LEN * sizeof(struct pte));
	pt2=(struct pte *)malloc(PAGE_TABLE_LEN * sizeof(struct pte));
	WriteRegister(REG_PTR0,(RCS421RegVal)pt1);
	WriteRegister(REG_PTR1,(RCS421RegVal)pt2);
	for(i=1;i<PMEM_BASE+pmem_size;i=i+PAGESIZE)
	{
		if(((i*PAGESIZE) > KERNEL_STACK_BASE && (i*PAGESIZE <brk))||(((i+1)*PAGESIZE-1 )> KERNEL_STACK_BASE && ((i+1)*PAGESIZE -1 <brk)))
			continue;
		*(int *)addr=(i*PAGESIZE);
		addr=(i*PAGE_SIZE);
			
	}






}

int SetKernelBrk(void *addr)
{
	brk=addr;
}
