#include<stdlib.h>
#include<string.h>
#include <comp421/hardware.h>
#include "yalnix.h"


struct PCB{
	int pid;
	struct PCB *next;
	int RG_PTR;
};
int pid_counter;
struct pte *pt1,*pt0;
int virtual_mem_enabled;
struct PCB *pcb_head;
void *mod_brk;
int pfn_head;
static void (*vector_table[TRAP_VECTOR_SIZE])(ExceptionStackFrame *);
void KernelStart(ExceptionStackFrame *frame, unsigned int pmem_size, void *org_brk, char **cmd_args)
{
	int i,j,addr,index;
	struct PCB *new_pcb;

	mod_brk=org_brk;
	WriteRegister(REG_VECTOR_BASE,(RCS421RegVal)vector_table);
	addr=0;

	pt0=(struct pte *)malloc(PAGE_TABLE_SIZE);
	pt1=(struct pte *)malloc(PAGE_TABLE_SIZE);
	if(pt1==NULL || pt0==NULL)
		TracePrintf(0,"Not enough memory");
	else 
		TracePrintf(0,"Enough memory with %p",pt1);

	WriteRegister(REG_PTR1,(RCS421RegVal)pt1);
	WriteRegister(REG_PTR0,(RCS421RegVal)pt0);

	addr=VMEM_1_BASE;
	j=VMEM_1_BASE/PAGESIZE;
	for(i=j;i< PAGE_TABLE_LEN+j;i++)
	{	
		if(addr>=VMEM_1_BASE && addr < (long)&_etext)
		{
			pt1[i-j].pfn = i;
			pt1[i-j].uprot=0;
			pt1[i-j].kprot=PROT_READ|PROT_EXEC;
			pt1[i-j].valid=1;
		}
		else if (addr>= (long)&_etext && addr < (long)mod_brk)
		{
			pt1[i-j].pfn = i;
			pt1[i-j].uprot=0;
			pt1[i-j].kprot=PROT_READ|PROT_WRITE;
			pt1[i-j].valid=1;

		}
		else
			pt1[i-j].valid=0;

		pt0[i-j].valid=0;
		addr=addr+PAGESIZE;
	}
	addr=KERNEL_STACK_BASE;
	while(addr<KERNEL_STACK_LIMIT)
	{i=(long)addr/PAGESIZE;
	pt0[i].pfn = i;
	pt0[i].uprot=0;//PROT_READ|PROT_WRITE;
	pt0[i].kprot=PROT_READ|PROT_WRITE;
	pt0[i].valid=1;
	addr+=PAGESIZE;
	}	
	index=0;
	while(pt1[index++].valid==1);

	pt1[index].pfn = 0;
	pt1[index].uprot=0;
	pt1[index].kprot=PROT_READ|PROT_WRITE;
	pt1[index].valid=1;


	addr=index;
	for(i=1;i<pmem_size/PAGESIZE;i++)
	{	
		if((i*PAGESIZE < KERNEL_STACK_BASE && ((i+1)*PAGESIZE)-1 < KERNEL_STACK_BASE)||(i*PAGESIZE > (long)mod_brk && ((i+1)*PAGESIZE)-1 < PMEM_BASE+pmem_size))
		{	//WriteRegister(REG_TLB_FLUSH,index*PAGESIZE);
			*(int *)(long)(index*PAGESIZE)=(i*PAGESIZE);
			pt1[index].pfn = i;
		}
	}
	*(int *)(long)(index*PAGESIZE)=-1;
	pt1[index].valid=0;

	virtual_mem_enabled=1;
	WriteRegister(REG_VM_ENABLE, 1);
	
	//new_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	//new_pcb->pid=pid_counter++;
	//pcb_head=new_pcb;
	//LoadProgram("idle",pcb_head,frame);
	//new_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	//new_pcb->pid=pid_counter++;
	//new_pcb->next=pcb_head;
	//pcb_head->next=new_pcb;
	//LoadProgram("init",pcb_head->next,frame);
	return ;

}

int SetKernelBrk(void *addr)
{
	int index;
	if(!virtual_mem_enabled)
	{	if((long)addr>=VMEM_1_LIMIT)
			return -1;
	}
	else
	{
		while((long)addr>UP_TO_PAGE(mod_brk))
		{	
			mod_brk=(void *)UP_TO_PAGE(mod_brk);
			index=(long)mod_brk/PAGESIZE;
			if(index >=PAGE_TABLE_SIZE||pfn_head==-1)
				return -1;
			pt1[index].pfn = (long)pfn_head;
			pt1[index].uprot=0;
			pt1[index].kprot=PROT_READ|PROT_WRITE;
			pt1[index].valid=1;
			pfn_head=*(int *)(long)(mod_brk);
		}

	}	
		mod_brk=addr;
		return 0;
}
