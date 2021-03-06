 #include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <comp421/hardware.h>
#include <comp421/loadinfo.h>
#include "yalnix.h"


#define WAITING_READ 0
#define WAITING_WRITE 1
#define NO_STATUS -1

struct PCB{
	int pid;
	struct PCB *next,*prev;
	long pt0_virt;
	long pt0_phy;
	long brk;
	SavedContext ctxp;
	int clockcount;
	int exit_status;
	int tid;
	int ticks;
	int io_status;
	struct PCB *parent;
	struct PCB *child;
	struct PCB *childQ;
};
struct Queue {
	struct PCB *front;
	struct PCB *rear;
};
struct Queue readyQ;
struct Queue blockedQ;
struct Queue exitedQ;

/* Kernel Calls*/
int kfork(ExceptionStackFrame *e);
int kexec(char *filename, char **argvec);
void kexit(int status);
int kwait(int *status_ptr);
int kgetpid(ExceptionStackFrame *e);
int kbrk(void *addr);
int kdelay(int clock_ticks);
int kttyread(int tty_id, void *buf, int len);
int kttywrite(int tty_id, void *buf, int len);

/* handlers */
void trap_kernel(ExceptionStackFrame *e);
void trap_clock(ExceptionStackFrame *e);
void trap_illegal(ExceptionStackFrame *e);
void trap_memory(ExceptionStackFrame *e);
void trap_math(ExceptionStackFrame *e);
void trap_tty_receive(ExceptionStackFrame *e);
void trap_tty_transmit(ExceptionStackFrame *e);
void trap_disk(ExceptionStackFrame *e);

/* utils */
long GetPage();
void ReturnPage(int,long);
long Get_Page_table(long *);
struct PCB * getQ(struct Queue *Q);
void addQ(struct Queue *Q,struct PCB *new_pcb);
SavedContext *MySwitchFunc(SavedContext *ctxp, void *p1, void *p2);

char * tbuffer[NUM_TERMINALS];
int tlines[NUM_TERMINALS]; //initialize it to 1
long one_pgt_pages;
int pid_counter;
struct pte *pt1;
int virtual_mem_enabled;
struct PCB *idle_pcb,*active_pcb,*init_pcb;
void *mod_brk;
long pfn_head;
static void (*vector_table[TRAP_VECTOR_SIZE])(ExceptionStackFrame *);
void initialize_pcb(struct PCB *npcb)
{
	if (npcb==NULL)
		return;
	npcb->pid=-1;
	npcb->next=NULL;
	npcb->prev=NULL;
	npcb->pt0_virt=-1;
	npcb->pt0_phy=-1;
	npcb->brk=-1;
	npcb->clockcount=-1;
	npcb->exit_status=-1;
	npcb->tid=-1;
	npcb->ticks=-1;
	npcb->io_status=-1;
	npcb->parent=NULL;
	npcb->child=NULL;
	npcb->childQ=NULL;
}




void KernelStart(ExceptionStackFrame *frame, unsigned int pmem_size, void *org_brk, char **cmd_args)
{
	int i,j,addr,index,res,no_of_pages=0;
	struct PCB *this_pcb;
	int p;
	struct pte *pt0;
	
	TracePrintf(0,"Entering KernelStart\n");
	mod_brk=org_brk;
	WriteRegister(REG_VECTOR_BASE,(RCS421RegVal)vector_table);
		//Initialize vector table
	vector_table[TRAP_KERNEL] = &trap_kernel;
	vector_table[TRAP_CLOCK] = &trap_clock;
	vector_table[TRAP_ILLEGAL] = &trap_illegal;
	vector_table[TRAP_MEMORY] = &trap_memory;
	vector_table[TRAP_MATH] = &trap_math;
	vector_table[TRAP_TTY_RECEIVE] = &trap_tty_receive;
	vector_table[TRAP_TTY_TRANSMIT] = &trap_tty_transmit;
	vector_table[TRAP_DISK] = &trap_disk;

	addr=0;
	pid_counter=0;

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
	{
		i=(long)(addr/PAGESIZE);
		pt0[i].pfn = i;
		pt0[i].uprot=0;//PROT_READ|PROT_WRITE;
		pt0[i].kprot=PROT_READ|PROT_WRITE;
		pt0[i].valid=1;
		addr+=PAGESIZE;
	}	
	index=0;
	pfn_head=MEM_INVALID_SIZE;
	addr=pfn_head;
	for(i=MEM_INVALID_PAGES;i<pmem_size/PAGESIZE;i++)
	{	
		if((i*PAGESIZE < KERNEL_STACK_BASE && ((i+1)*PAGESIZE)-1 < KERNEL_STACK_BASE)||(i*PAGESIZE > (long)mod_brk && ((i+1)*PAGESIZE)-1 < PMEM_BASE+pmem_size))
		{	
			*(int *)(long)(addr)=(i*PAGESIZE);
			addr=i*PAGESIZE;
			no_of_pages++;
		}
	}
	TracePrintf(0,"Number of pages= %d",no_of_pages);
	*(int *)(long)(addr)=-1;
	
	virtual_mem_enabled=1;
	WriteRegister(REG_VM_ENABLE, 1);
	TracePrintf(0,"Reached after initializing VM\n");

	


	idle_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	initialize_pcb(idle_pcb);
	idle_pcb->pid=pid_counter++;
	idle_pcb->pt0_virt=pt0;
	idle_pcb->pt0_phy=pt0;
	idle_pcb->parent=NULL;
	active_pcb=idle_pcb;
	LoadProgram(frame,(struct PCB *)idle_pcb,"idle",&cmd_args[0]);
	WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);

	init_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	initialize_pcb(init_pcb);
	init_pcb->pid=pid_counter++;
	init_pcb->pt0_virt=(struct pte *)Get_Page_table(&(init_pcb->pt0_phy));
	init_pcb->parent=NULL;

	
	
	active_pcb=init_pcb;
	if((res=ContextSwitch(MySwitchFunc, &idle_pcb->ctxp, (void *)idle_pcb, (void *)init_pcb))==-1)
		TracePrintf(0,"\n Error while context switching");
	
	if(active_pcb==idle_pcb)
	{	TracePrintf(0,"Entering idle process");
		
		TracePrintf(0,"Exiting KernelStart\n");
		return;
	}
	LoadProgram(frame,(struct PCB *)init_pcb,"init",&cmd_args[0]);
	WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);


	TracePrintf(0,"Exiting KernelStart\n");
	return ;
}

int SetKernelBrk(void *addr)
{
	
	static int index;
	TracePrintf(0,"Entering setKernelBrk\n");
	if(!virtual_mem_enabled)
	{	if((long)addr>=VMEM_1_LIMIT)
			return -1;
	}
	else if((long)addr < (DOWN_TO_PAGE(mod_brk)))
	{	while((long)addr < (DOWN_TO_PAGE(mod_brk)))
		{	mod_brk=(void *)(DOWN_TO_PAGE(mod_brk));
			index=(long)mod_brk/PAGESIZE-PAGE_TABLE_LEN;
			mod_brk-=PAGESIZE;

			ReturnPage(pt1[index].pfn,(index+PAGE_TABLE_LEN)*PAGESIZE);
			pt1[index].valid=0;
			WriteRegister(REG_TLB_FLUSH,(index+PAGE_TABLE_LEN)*PAGESIZE);

		}
	}
	else
	{
		while((long)addr>UP_TO_PAGE(mod_brk))
		{	
			mod_brk=(void *)(UP_TO_PAGE(mod_brk)+1);
			index=(long)mod_brk/PAGESIZE-PAGE_TABLE_LEN;
			TracePrintf(0,"SetKernelBrk : value of mod_brk =%x and val of index=%x",mod_brk,index);
			if(index >=PAGE_TABLE_LEN||pfn_head==-1)
				return -1;
			pt1[index].pfn = GetPage();
			pt1[index].uprot=0;
			pt1[index].kprot=PROT_READ|PROT_WRITE;
			pt1[index].valid=1;
			WriteRegister(REG_TLB_FLUSH,(index+PAGE_TABLE_LEN)*PAGESIZE);
		}

	}	
		mod_brk=addr;
		TracePrintf(0,"Exiting SetKernelBrk\n");
		return 0;
}

SavedContext *MySwitchFunc(SavedContext *ctxp, void *p1, void *p2)
{	
		struct pte *pt1,*pt2;
		int i,j,k=0,index[KERNEL_STACK_PAGES];
		TracePrintf(0,"Entering MySwitchFunc\n");
		
		pt2=((struct PCB*)p2)->pt0_virt;
		TracePrintf(0,"Reached in first line\n");
		
		if((pt1!=pt2)&&(pt2[KERNEL_STACK_BASE/PAGESIZE].valid!=1))
		{	if(pt1==NULL)
				return ctxp;
			memcpy(&(((struct PCB*)p2)->ctxp),ctxp,sizeof(SavedContext));

			pt1=((struct PCB*)p1)->pt0_virt;
		
			i=MEM_INVALID_PAGES;
			while(pt1[i].valid==1)
					i++;
			

			for(j=KERNEL_STACK_BASE/PAGESIZE;j<KERNEL_STACK_LIMIT/PAGESIZE;j++)
			{	
				WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);
				TracePrintf(0,"before assigning: pfn of pt1=%d is %d\n",pt1,pt1[i].pfn);

				pt1[i].pfn = pt2[j].pfn;
				pt2[j].valid=1;
				pt2[j].uprot=0;
				pt2[j].kprot=PROT_READ|PROT_WRITE;
				pt1[i].uprot=0;
				pt1[i].kprot=PROT_READ|PROT_WRITE;
				pt1[i].valid=1;
				memcpy((void *)(i*PAGESIZE),(void *)(j*PAGESIZE),PAGESIZE);
				TracePrintf(0,"copied %d pages with i= %d\n",j,i);

				
			}
		
			pt1[i].valid=0;
			
			
		}

		TracePrintf(0,"MySwitchFunc: Reached after memcpy\n");

		TracePrintf(0,"PID value of new active_pcb=%d\n",((struct PCB*)p2)->pid);
	
		WriteRegister(REG_PTR0,(RCS421RegVal)((struct PCB*)p2)->pt0_phy);	
	
		WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);
	
		TracePrintf(0,"Exiting MySwitchFunc\n");
	
	      return (&((struct PCB*)p2)->ctxp);

}



/*
 *  Load a program into the current process's address space.  The
 *  program comes from the Unix file identified by "name", and its
 *  arguments come from the array at "args", which is in standard
 *  argv format.
 *
 *  Returns:
 *      0 on success
 *     -1 on any error for which the current process is still runnable
 *     -2 on any error for which the current process is no longer runnable
 *
 *  This function, after a series of initial checks, deletes the
 *  contents of Region 0, thus making the current process no longer
 *  runnable.  Before this point, it is possible to return ERROR
 *  to an Exec() call that has called LoadProgram, and this function
 *  returns -1 for errors up to this point.  After this point, the
 *  contents of Region 0 no longer exist, so the calling user process
 *  is no longer runnable, and this function returns -2 for errors
 *  in this case.
 */
int
LoadProgram(ExceptionStackFrame *exptr, struct PCB *proc_pcb, char *name,char **args)
{
	int fd;
    	int status;
    	struct loadinfo li;
    	char *cp;
    	char *cp2;
    	char **cpp;
    	char *argbuf;
    	int i,j;
    	unsigned long argcount;
    	int size;
    	int text_npg;
    	int data_bss_npg;
    	int stack_npg;
	int addr;
	int index=0;
	int no_of_pages=0;
	struct pte  *pt0;
	pt0=(struct pte *)proc_pcb->pt0_virt;
	long temp_var;
	
	TracePrintf(0,"Entering LoadProgram\n");
	TracePrintf(0, "LoadProgram '%s', args %p\n", name, args);

    	if ((fd = open(name, O_RDONLY)) < 0) {
		TracePrintf(0, "LoadProgram: can't open file '%s'\n", name);
		return (-1);
    	}

    	status = LoadInfo(fd, &li);
    	TracePrintf(0, "LoadProgram: LoadInfo status %d\n", status);
    	switch (status) {
		case LI_SUCCESS:
	    		break;
		case LI_FORMAT_ERROR:
	    		TracePrintf(0,
			"LoadProgram: '%s' not in Yalnix format\n", name);
	    		close(fd);
	    		return (-1);
		case LI_OTHER_ERROR:
	    		TracePrintf(0, "LoadProgram: '%s' other error\n", name);
	    		close(fd);
	    		return (-1);
		default:
	    		TracePrintf(0, "LoadProgram: '%s' unknown error\n", name);
	    		close(fd);
	   		 return (-1);
    	}
	TracePrintf(0, "text_size 0x%lx, data_size 0x%lx, bss_size 0x%lx\n",li.text_size, li.data_size, li.bss_size);
    	TracePrintf(0, "entry 0x%lx\n", li.entry);

	size = 0;
    	for (i = 0; args[i] != NULL; i++) {
		size += strlen(args[i]) + 1;
    	}
    	argcount = i;
    	TracePrintf(0, "LoadProgram: size %d, argcount %d\n", size, argcount);
	
	cp = argbuf = (char *)malloc(size);
    	for (i = 0; args[i] != NULL; i++) {
		strcpy(cp, args[i]);
		cp += strlen(cp) + 1;
    	}
  
    /*
     *  The arguments will get copied starting at "cp" as set below,
     *  and the argv pointers to the arguments (and the argc value)
     *  will get built starting at "cpp" as set below.  The value for
     *  "cpp" is computed by subtracting off space for the number of
     *  arguments plus 4 (for the argc value, a 0 (AT_NULL) to
     *  terminate the auxiliary vector, a NULL pointer terminating
     *  the argv pointers, and a NULL pointer terminating the envp
     *  pointers) times the size of each (sizeof(void *)).  The
     *  value must also be aligned down to a multiple of 8 boundary.
     */
    	cp = ((char *)USER_STACK_LIMIT) - size;
   	cpp = (char **)((unsigned long)cp & (-1 << 4));	/* align cpp */
    	cpp = (char **)((unsigned long)cpp - ((argcount + 4) * sizeof(void *)));

    	text_npg = li.text_size >> PAGESHIFT;
	proc_pcb->brk=li.data_size + li.bss_size+li.text_size;
    	data_bss_npg = UP_TO_PAGE(li.data_size + li.bss_size) >> PAGESHIFT;
    	stack_npg = (USER_STACK_LIMIT - DOWN_TO_PAGE(cpp)) >> PAGESHIFT;

    	TracePrintf(0, "LoadProgram: text_npg %d, data_bss_npg %d, stack_npg %d\n",
		text_npg, data_bss_npg, stack_npg);

    /*
     *  Make sure we will leave at least one page between heap and stack
     */
    	if (MEM_INVALID_PAGES + text_npg + data_bss_npg + stack_npg +
		1 + KERNEL_STACK_PAGES >= PAGE_TABLE_LEN) {
		TracePrintf(0, "LoadProgram: program '%s' size too large for VM\n",name);
		free(argbuf);
		close(fd);
		TracePrintf(0,"Exiting LoadProgram\n");
		return (-1);
    	}

    /*
     *  And make sure there will be enough physical memory to
     *  load the new program.
     *	  The new program will require text_npg pages of text,
     *  data_bss_npg pages of data/bss, and stack_npg pages of
     *  stack.  In checking that there is enough free physical
     *  memory for this, BE SURE TO ALLOW FOR THE PHYSICAL MEMORY
     *  PAGES ALREADY ALLOCATED TO THIS PROCESS THAT WILL BE
     *  FREED BELOW BEFORE WE ALLOCATE THE NEEDED PAGES FOR
     *  THE NEW PROGRAM BEING LOADED.
	*/
	
	TracePrintf(0,"LoadProgram : reached 1\n");
	
	index=MEM_INVALID_PAGES+1;
	addr=pfn_head;
		
	while(pt0[index++].valid!=0);
	
	pt0[index].pfn = addr/PAGESIZE;
	pt0[index].uprot=0;
	pt0[index].kprot=PROT_READ|PROT_WRITE;
	pt0[index].valid=1;
	
	WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)(index*PAGESIZE));
	
	no_of_pages=text_npg + data_bss_npg + stack_npg+size/PAGESIZE;
	i=0;
	
	TracePrintf(0,"LoadProgram : reached 2\n");
	
	while(*(int *)(long)((index)*PAGESIZE)!=-1)
	{	
		WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)(index*PAGESIZE));
		temp_var=(*(int *)(long)((index)*PAGESIZE))/PAGESIZE;
		pt0[index].pfn=temp_var;	
		i++;
		if(i>no_of_pages)
			break;
	}
	
	pt0[index].valid=0;
	
	TracePrintf(0,"LoadProgram : reached 3\n");
	
    	if (no_of_pages>i) {
		TracePrintf(0,"LoadProgram: program '%s' size too large for physical memory\n",
	    	name);
		free(argbuf);
		close(fd);
		return (-1);
    		}

    /* Initialize sp for the current process to (char *)cpp.
     The value of cpp was initialized above.
	*/
	TracePrintf(0,"LoadProgram : reached 5\n");
	exptr->sp=(void  *)cpp;
	TracePrintf(0,"value of sp= %d",(void *)cpp);
    /*
     *  Free all the old physical memory belonging to this process,
     *  but be sure to leave the kernel stack for this process (which
     *  is also in Region 0) alone.
     */
	TracePrintf(0,"LoadProgram : reached 6\n");

	
	 for(i=MEM_INVALID_PAGES;i < KERNEL_STACK_BASE/PAGESIZE;i++)
	 {
		if (pt0[i].valid ==1)
		{	
			ReturnPage(pt0[i].pfn,i*PAGESIZE);
			pt0[i].valid=0;
		}

	 }

	TracePrintf(0,"LoadProgram : reached 7\n");
	WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);

    /* Loop over all PTEs for the current process's Region 0,
     except for those corresponding to the kernel stack (between
     address KERNEL_STACK_BASE and KERNEL_STACK_LIMIT).  For
     any of these PTEs that are valid, free the physical memory
     memory page indicated by that PTE's pfn field.  Set all
     of these PTEs to be no longer valid.
	*/
    /*
     *  Fill in the page table with the right number of text,
     *  data+bss, and stack pages.  We set all the text pages
     *  here to be read/write, just like the data+bss and
     *  stack pages, so that we can read the text into them
     *  from the file.  We then change them read/execute.
     */
	
    
	 
	for(i=0;i<MEM_INVALID_PAGES;i++)
		pt0[i].valid = 0;
	TracePrintf(0,"LoadProgram : reached 7.5\n");

    /* First, the text pages */

    	for(i=MEM_INVALID_PAGES;i<text_npg+MEM_INVALID_PAGES;i++)
	{   
        	pt0[i].pfn   = GetPage();
		pt0[i].valid = 1;
		pt0[i].uprot = PROT_READ | PROT_EXEC;
      		pt0[i].kprot = PROT_READ | PROT_WRITE;
	}
	TracePrintf(0,"LoadProgram : reached 8\n");

    /* Then the data and bss pages */
    // For the next data_bss_npg number of PTEs in the Region 0
    // page table, initialize each PTE:
    	j=0;
	while(j<data_bss_npg)
	{   
	 	pt0[i].valid = 1;
      		pt0[i].kprot = PROT_READ | PROT_WRITE;
            	pt0[i].uprot = PROT_READ | PROT_WRITE;
            	pt0[i].pfn   =GetPage();
		i++;j++;
		
	}
	
	TracePrintf(0,"LoadProgram : reached 9\n");
    
	/* And finally the user stack pages */
    // For stack_npg number of PTEs in the Region 0 page table
    // corresponding to the user stack (the last page of the
    // user stack *ends* at virtual address USER_STACK_LMIT),
    
	i=(USER_STACK_LIMIT-PAGESIZE)/PAGESIZE;
	j=0;
	while(j<stack_npg)
	{   
		pt0[i].valid = 1;
           	pt0[i].kprot = PROT_READ | PROT_WRITE;
           	pt0[i].uprot = PROT_READ | PROT_WRITE;
            	pt0[i].pfn   = GetPage();
		j++;i--;
	}
	
	TracePrintf(0,"LoadProgram : reached 10\n");
    
	/*
     *  All pages for the new address space are now in place.  Flush
     *  the TLB to get rid of all the old PTEs from this process, so
     *  we'll be able to do the read() into the new pages below.
     */

   	
	TracePrintf(0,"LoadProgram : reached 10.0\n");

    /*
     *  Read the text and data from the file into memory.
     */
	
	
   	if (read(fd, (void *)MEM_INVALID_SIZE, li.text_size+li.data_size)
	!= li.text_size+li.data_size) {
	TracePrintf(0, "LoadProgram: couldn't read for '%s'\n", name);
	free(argbuf);
	close(fd);


	/*>>>> Since we are returning -2 here, this should mean to
	>>>> the rest of the kernel that the current process should
	>>>> be terminated with an exit status of ERROR reported
	>>>> to its parent process.
	*/
	
	return (-2);
    }
	TracePrintf(0,"LoadProgram : reached 11\n");
	close(fd);	
			/* we've read it all now */
	


    /*
     *  Now set the page table entries for the program text to be readable
     *  and executable, but not writable.
     */
	TracePrintf(0,"LoadProgram : reached 11.5\n");

    //>>>> For text_npg number of PTEs corresponding to the user text
    //>>>> pages, set each PTE's kprot to PROT_READ | PROT_EXEC.
	
	
	for(i=MEM_INVALID_PAGES;i<text_npg+MEM_INVALID_PAGES;i++)
	{  
		pt0[i].kprot = PROT_READ | PROT_EXEC;
	}

	TracePrintf(0,"LoadProgram : reached 12\n");
    /*
     *  Zero out the bss
     */
    memset((void *)(MEM_INVALID_SIZE + li.text_size + li.data_size),
	'\0', li.bss_size);
	
	

	
    /*
     *  Set the entry point in the exception frame.
     */
    //>>>> Initialize pc for the current process to (void *)li.entry
	
	exptr->pc=(void *)li.entry;
	
    /*
     *  Now, finally, build the argument list on the new stack.
     */
    	*cpp++ = (char *)argcount;		/* the first value at cpp is argc */
    	cp2 = argbuf;
    	for (i = 0; i < argcount; i++) {      /* copy each argument and set argv */
		*cpp++ = cp;
		strcpy(cp, cp2);
		cp += strlen(cp) + 1;
		cp2 += strlen(cp2) + 1;
    	}
    	free(argbuf);
    	*cpp++ = NULL;	/* the last argv is a NULL pointer */
    	*cpp++ = NULL;	/* a NULL pointer for an empty envp */
    	*cpp++ = 0;		/* and terminate the auxiliary vector */

    /*
     *  Initialize all regs[] registers for the current process to 0,
     *  initialize the PSR for the current process also to 0.  This
     *  value for the PSR will make the process run in user mode,
     *  since this PSR value of 0 does not have the PSR_MODE bit set.
     */
    //>>>> Initialize regs[0] through regs[NUM_REGS-1] for the
    //>>>> current process to 0.
    //>>>> Initialize psr for the current process to 0.
	
	exptr->psr=0;
	
	while(i<NUM_REGS)
		exptr->regs[i++]=0;
	
	TracePrintf(0,"Exiting LoadProgram\n");
    	return (0);
}

long Get_Page_table(long *pt_phy)
{
	static long max_allocated=VMEM_1_LIMIT-1;
	long temp,res;
	int i, j,index;
	struct pte *pt_new;

	TracePrintf(0,"Entering Get_page_table\n");
	//if(one_pgt_pages==-1){
		one_pgt_pages=DOWN_TO_PAGE(max_allocated);
		max_allocated=DOWN_TO_PAGE(max_allocated)-1;
		index=(one_pgt_pages/PAGESIZE)-PAGE_TABLE_LEN;

		TracePrintf(0,"Get_Page_table : reached 1\n");
		if(pfn_head==-1||(one_pgt_pages==DOWN_TO_PAGE(mod_brk)))
		{	TracePrintf(0,"Not enought memory space");
			return -1;
		}
		TracePrintf(0,"Get_Page_table : reached 2\n");

		pt1[index].pfn = GetPage();
		pt1[index].uprot=0;
		pt1[index].kprot=PROT_READ|PROT_WRITE;
		pt1[index].valid=1;
		TracePrintf(0,"value of phy address is %x and virt_addr= %x\n",pt1[index].pfn*PAGESIZE,index*PAGESIZE);

		*pt_phy=pt1[index].pfn*PAGESIZE;
		TracePrintf(0,"value of phy address is %d\n",*pt_phy);
		TracePrintf(0,"Get_Page_table : reached 3\n");


		res=(index+PAGE_TABLE_LEN)*PAGESIZE;
		pt_new=(struct pte *)res;
		for(i=MEM_INVALID_PAGES;i<PAGE_TABLE_LEN-KERNEL_STACK_PAGES;i++)
		{	pt_new[i].pfn = -1;
			pt_new[i].uprot=0;//PROT_READ|PROT_WRITE;
			pt_new[i].kprot=0;
			pt_new[i].valid=0;

		}
		for(i=KERNEL_STACK_BASE/PAGESIZE;i<KERNEL_STACK_LIMIT/PAGESIZE;i++)
		{	
		pt_new[i].pfn = GetPage();
		pt_new[i].valid=0;
		pt_new[i].uprot=0;
		pt_new[i].kprot=PROT_READ|PROT_WRITE;

		}

		temp=one_pgt_pages+(long)PAGE_TABLE_SIZE;
		
		TracePrintf(0,"value of temp is %d\n",temp);

		
		TracePrintf(0,"Exiting Get_page_table\n");
		return res;
	//}
	/*index=one_pgt_pages/PAGESIZE;
	temp=	(long)pt1[index].pfn*PAGESIZE;
	if(((struct pte *)index)->valid==1)
	{	one_pgt_pages=((struct pte *)index)->pfn*PAGESIZE;
		TracePrintf(0,"Exiting Get_page_table\n");
		return temp;
	}
	else
	{	index=index+PAGE_TABLE_SIZE;
		one_pgt_pages=((struct pte *)index)->pfn*PAGESIZE;
		TracePrintf(0,"Exiting Get_page_table\n");
		return (temp+PAGE_TABLE_SIZE);
	}*/




}

long GetPage()
{
	int index=MEM_INVALID_PAGES;
	long res;
	struct pte *pt0=(struct pte *)active_pcb->pt0_virt;
	TracePrintf(0,"Entering GetPage\n");
	while(pt0[index++].valid==1);
	WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)index*PAGESIZE);
	TracePrintf(0,"value of pfn_head is %d\n",pfn_head);
	if (pfn_head==-1)
		return -1;
	TracePrintf(0,"value of pfn is %d\n",pt0[index].pfn);
	pt0[index].pfn = pfn_head/PAGESIZE;
	TracePrintf(0,"value of pfn is %d\n",pt0[index].pfn);

	pt0[index].uprot=0;
	pt0[index].kprot=PROT_READ|PROT_WRITE;
	pt0[index].valid=1;
	res=pt0[index].pfn;
	
	
	TracePrintf(0,"value of pfn is %d and reached before assignment\n",pt0[index].pfn);

	pfn_head=*(int *)(long)(index*PAGESIZE);

	TracePrintf(0,"value of pfn_head is %d\n",pfn_head);
	pt0[index].valid=0;
	
	TracePrintf(0,"Exiting GetPage\n");
	return res; 
}
void ReturnPage(int pfn,long vaddr)
{
	TracePrintf(0,"Entering Return_page\n");
	TracePrintf(0,"value of initial pfn_head is %x\n",pfn_head);
	TracePrintf(0,"value of passed vadder is %x\n",vaddr);
	TracePrintf(0,"value of passed pfn is %x\n",pfn);


	*(int *)(long)(vaddr)=pfn_head;
	TracePrintf(0,"value at mem location is %x\n",*(int *)(long)(vaddr));

	pfn_head = (long)pfn*PAGESIZE;
	TracePrintf(0,"value of new pfn *PAGESIZE= is %x\n",pfn*PAGESIZE);
	TracePrintf(0,"value of new pfn_head is %x\n",pfn_head);
	TracePrintf(0,"Exiting Return Page\n");

}

/*
* Interrupt handlers
*/
void trap_kernel(ExceptionStackFrame *e){
    
	switch(e->code)
    {
        case YALNIX_FORK:
		kfork(e);
		break;
        case YALNIX_EXEC:
		
            break;
        case YALNIX_EXIT:
		kexit(e->regs[1]);
            break;
        case YALNIX_WAIT:
            break;
        case YALNIX_GETPID:
            kgetpid(e);
            break;
        case YALNIX_BRK:
            break;
        case YALNIX_DELAY:
            break;
 	  case YALNIX_TTY_READ:
            e->regs[0] = kttyread(e->regs[1], e->regs[2], e->regs[3]);
            break;
        case YALNIX_TTY_WRITE:
        	e->regs[0] = kttyWrite(e->regs[1], e->regs[2], e->regs[3]);
            break;
    }
}

void trap_clock(ExceptionStackFrame *e){
	if(current_process->clock > 2)
	{
		/* store the head of blocked queue */
		struct PCB * blocked_head = blockedQ;

		/* go through blockedQ, move available processes to readyQ, and remove these process from blockedQ */
		if(blockedQ->next != NULL)
		{
			while(blockedQ->next != NULL)
			{
				blockedQ->next->ticks--;
				/* if the tick of this process is 0, add this process t*/
				if(blockedQ->next->ticks == 0)
				{
					addQ(readyQ, blockedQ->next);
					remove_from_Q(blockedQ, blockedQ->next);
				}
			}
		}

	    //TODO
	    //context switch to next available process.
	}
}

void trap_illegal(ExceptionStackFrame *e){
	//call kexit to stop the current one and switch to next available process
	printf("The current process '%d' is executing an illegal instruction. The kernel forcequites it.", current_process->pid);
	kexit(ERROR);
}

void trap_memory(ExceptionStackFrame *e){

	long curr,addr=(long)e->addr;
	int i,insuf_mem=0;
	struct pte *pt0=active_pcb->pt0_virt;
	TracePrintf(0,"Entering Trap_Memory with address as %ld and code = %d \n",addr,e->code);
	i=(USER_STACK_LIMIT-PAGESIZE)/PAGESIZE;
	while(i>active_pcb->brk)
	{
		if (pt0[i].valid==0)
			break;
		else
			i--;
	}	
	if((addr<active_pcb->brk)||( addr<USER_STACK_LIMIT &&addr >=i*PAGESIZE))
	{	printf("Invalid Memory Access by process with id %d\n",active_pcb->pid);
		kexit(ERROR);
	}
	
	if(addr>KERNEL_STACK_BASE||addr < UP_TO_PAGE(active_pcb->brk)+PAGESIZE)
	{	printf("Invalid Memory Access by process with id %d\n",active_pcb->pid);
		kexit(ERROR);
	}
	curr=DOWN_TO_PAGE(USER_STACK_LIMIT);
	i=curr/PAGESIZE;
	while(addr<curr)
	{	if(pt0[i].valid==0)
		{
			pt0[i].valid = 1;
           		pt0[i].kprot = PROT_READ | PROT_WRITE;
           		pt0[i].uprot = PROT_READ | PROT_WRITE;
            		pt0[i].pfn   = GetPage();
			if(pt0[i].pfn==-1)
			{	printf("Not sufficient Memory available to process with id %d\n",active_pcb->pid);
				insuf_mem=1;
				break;
			}				
		
		}
		curr=DOWN_TO_PAGE(curr-1);
		i=curr/PAGESIZE;
	}
	if(insuf_mem)
		kexit(ERROR);
	return;	
			

	
}

void trap_math(ExceptionStackFrame *e){
	//call kexit to stop the current one and switch to next available process
	printf("The current process '%d' results in some arithmetic error. The kernel forcequites it.", current_process->pid);
	kexit(ERROR);
}

void trap_tty_receive(ExceptionStackFrame *e){
	if(tlines[e->code] == 1)
		tbuffer[e->code] = (char*)malloc(TERMINAL_MAX_LINE * sizeof(char)*tlines[e->code]);
	else
		tbuffer[e->code] = (char*)realloc(tbuffer[e->code], TERMINAL_MAX_LINE * sizeof(char) * tlines[e->code]);

	tlines[e->code]++;

	int len = TtyReceive(e->code, tbuffer[e->code]+(TERMINAL_MAX_LINE*tlines[e->code]-1)), TERMINAL_MAX_LINE*sizeof(char));

    /* unblock the processes whose */
	if(blockedQ->front == NULL) return;
	/* store the readyQ head */
	struct PCB* tempblockedQ = blockedQ;
	while(tempblockedQ->front != NULL)
	{
		if(tempblockedQ->front->tid == e->code && tempblockedQ->front->io_status == WAITING_READ)
		{
			tempblockedQ->front->io_status = NO_STATUS;
			remove_from_Q(blockedQ,tempreadyQ->frond);
			addQ(readyQ, tempreadyQ->frond);
			break; // everytime unblock one process
		}
		tempblockedQ->front = tempblockedQ->front->next;
	}

}

void trap_tty_transmit(ExceptionStackFrame *e){
	if(blockedQ->front == NULL) return;
	else
	{
		struct *Queue tempblockedQ = blockedQ;
		while(tempblockedQ->front != NULL)
		{	
			/* ublock some process */
			if(tempblockedQ->front->tid == e->code && tempblockedQ->front->io_status == WAITING_WRITE)
			{
				tempblockedQ->front->io_status = NO_STATUS;
				remove_from_Q(blockedQ,tempreadyQ->frond);
				addQ(readyQ, tempreadyQ->frond);
				break; // everytime unblock one process
			}
			tempblockedQ->front = tempblockedQ->front->next;
		}
	}
}

void trap_disk(ExceptionStackFrame *e){

}

/* Kernel Calls*/
int kfork(ExceptionStackFrame *e){
	struct PCB *new_pcb,*old_pcb,*child_pcb;
	struct pte *new_pt0, *old_pt0;
	int i,j;
	
	TracePrintf(0,"Entering fork()\n");
	
	new_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	initialize_pcb(new_pcb);
	new_pcb->pid=pid_counter++;
	new_pcb->pt0_virt=(struct pte *)Get_Page_table(&(new_pcb->pt0_phy));
	new_pt0=(struct pte *)new_pcb->pt0_virt;
	old_pcb=active_pcb;
	old_pt0=(struct pte *)old_pcb->pt0_virt;
	i=MEM_INVALID_PAGES;
	while(pt1[i].valid==1)
          i++;
      for(j=MEM_INVALID_PAGES;j<KERNEL_STACK_BASE/PAGESIZE;j++)
	{   new_pt0[j].valid=old_pt0[j].valid;
	    if(new_pt0[j].valid==1)
	    {
          	WriteRegister(REG_TLB_FLUSH,(i+PAGE_TABLE_LEN)*PAGESIZE);
          	
	    	new_pt0[j].pfn=GetPage();
	    	new_pt0[j].uprot=old_pt0[j].uprot;
	    	new_pt0[j].kprot=old_pt0[j].kprot;
          	pt1[i].pfn = new_pt0[j].pfn;
          	
          	pt1[i].uprot=0;
          	pt1[i].kprot=PROT_READ|PROT_WRITE;
          	pt1[i].valid=1;

		memcpy((void *)((i+PAGE_TABLE_LEN)*PAGESIZE),(void *)(j*PAGESIZE),PAGESIZE);
          	
		}
      }
	pt1[i].valid=0;
	child_pcb=old_pcb->child;
	if(child_pcb==NULL)
		old_pcb->child=new_pcb;
	else
	{	while(child_pcb->childQ!=NULL)
			child_pcb=old_pcb->child;
		child_pcb->childQ=new_pcb;
	}
	new_pcb->parent=old_pcb;

	e->regs[0]=new_pcb->pid;
	addQ(&readyQ,old_pcb);
	active_pcb=new_pcb;
	if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)new_pcb))==-1)
        	TracePrintf(0,"\n Error while context switching");
      if(active_pcb->pid > old_pcb->pid)
	{
		TracePrintf(0,"New Pcb Returning");
		e->regs[0]=0;
		TracePrintf(0,"Exiting fork()\n");
		return 0;
	}
	TracePrintf(0,"old Pcb Returning");

	TracePrintf(0,"Exiting fork()\n");
	return (int)active_pcb->pid;

}

int kexec(char *filename, char **argvec){
}

void kexit(int status)
{
	int i;
	struct pte *pt0;
	struct PCB *old_pcb,*child_pcb;
	pt0=active_pcb->pt0_virt;
	old_pcb=active_pcb;
	TracePrintf(0,"Entering kexit()\n");
	
	for (i=MEM_INVALID_PAGES+1;i< KERNEL_STACK_LIMIT/PAGESIZE;i++)
	{
		if(pt0[i].valid==1)
		{	
			TracePrintf(0,"In kexit():value of index is %d\n",i);
			WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);
			pt0[i].kprot=PROT_READ|PROT_WRITE;
			TracePrintf(0,"value of mapped permission kprot=%d and uprot= is %d\n",pt0[i].kprot,pt0[i].uprot);
			ReturnPage(pt0[i].pfn,i*PAGESIZE);
			WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);

			
		}
	}	

	

	TracePrintf(0,"In kexit() reached 1");
	old_pcb->exit_status=status;
	if(old_pcb->parent==NULL)
		old_pcb->parent=NULL;
	else if(old_pcb->parent->child!=old_pcb)
	{	child_pcb=old_pcb->parent->child;
		while(child_pcb->childQ!=old_pcb)
			child_pcb=child_pcb->childQ;
		child_pcb->childQ=old_pcb->childQ;
	}
	else
		old_pcb->parent->child=NULL;
		
	
	old_pcb->parent=NULL;
	child_pcb=old_pcb->child;
	while(child_pcb)
	{	child_pcb->parent=NULL;
		child_pcb=child_pcb->childQ;
	}
	TracePrintf(0,"In kexit() reached 2");


	active_pcb=getQ(&readyQ);

	if(active_pcb==NULL)
		active_pcb=idle_pcb;
	
	
	TracePrintf(0,"In kexit() reached 3 val of phy add is %d",active_pcb->pt0_virt);
	if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)NULL, (void *)active_pcb))==-1)
      			TracePrintf(0,"\n Error while context switching\n");

	TracePrintf(0,"\nReached 5");
	free((void *)old_pcb->pt0_virt);
	free(old_pcb);
	TracePrintf(0,"Exiting kexit()");


}

int kwait(int *status_ptr){
}

int kgetpid(ExceptionStackFrame *e){
    TracePrintf(0, "in getpid '%d'", active_pcb->pid);
    e->regs[0]=active_pcb->pid;
    return active_pcb->pid;
}

int kbrk(void *addr){



}

int kdelay(int clock_ticks){

struct PCB * next_process;
TracePrintf(0, "enter delay");
    if(clock_ticks == 0)
        return 0;
    if(clock_ticks < 0)
        return ERROR;
    else
    {
        
        TracePrintf(0, "I'm in delay.");
        /* Add this process to blocked queue*/
        addQ(&blockedQ, active_pcb);

        /*set the current process's clock ticks*/
        active_pcb->clockcount = clock_ticks;  

        next_process = getQ(&readyQ);
        if(next_process == NULL)
        {
           // if(ContextSwitch(MySwitchFunc, &idle_pcb->ctxp, (void *)active_pcb, (void *)idle_pcb)==-1)
             //   TracePrintf(0,"\n Error while context switching");

            TracePrintf(0, "prepare for switching to init");
            /* If no process is ready, it would switch to init */
            TracePrintf(0,"value of pt1=%d and pt2 =%d\n",idle_pcb,idle_pcb);
            if(ContextSwitch(MySwitchFunc, &idle_pcb->ctxp, (void *)active_pcb, (void *)idle_pcb)==-1)
                 TracePrintf(0,"\n Error while context switching");

            //TracePrintf(0,"value of pt1=%d and pt2 =%d\n",active_pcb,idle_pcb);
            return 0;
        }
        else
        {
            //if((ContextSwitch(MySwitchFunc, &next_process->ctxp, (void *)active_pcb, (void *)next_process))==-1)
              //  TracePrintf(0,"\n Error while context switching");

            TracePrintf(0, "prepare for switching to next process");
             /* context switch to next available process*/
            if((ContextSwitch(MySwitchFunc, &next_process->ctxp, (void *)active_pcb, (void *)next_process))==-1)
                 TracePrintf(0,"\n Error while context switching");
            return 0;
        }
    }


}

int kttyread(int tty_id, void *buf, int len)
{
	if(tty_id < 0 || tty_id > NUM_TERMINALS)
	{
		printf("Terminal %d does not exist.", tty_id);
		return ERROR;
	}
		
	if(len < 0 || len > TERMINAL_MAX_LINE)
	{
		printf("The length of input line is illegal. Please check it.");
		return ERROR;
	}

	/* if the corresponding buffer is not null, read chars to user's buf immediately and won't block this process*/
	int countchar;
	if(tbuffer[tty_id] != NULL)
	{
		countchar = 0;
		/* copy first len chars or all available chars in this line*/
		while(countchar < len && tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1) != NULL) 
		{
			strncpy(tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1), buf+countchar,1);
			countchar++;
			tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1) += 1; // keep the head pointing the current char
		}
		return countchar;
	}
	else
	{
		/* put this process to blockedQ */
 		current_process->io_status = WAITING_READ;
 		addQ(blockedQ, current_process);

 		/* switch to next available process*/
 		if(readyQ->front == NULL)
 			//TODO: contextswitch to idle
 		else
 		{
 			//TODO: context switch to next available process by using getQ()
 		}

 		/* when coming back, tbuffer is not null, it is available to read*/
 		countchar = 0;
		/* copy first len chars or all available chars in this line*/
		while(countchar < len && tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1) != NULL) 
		{
			strncpy(tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1), buf+countchar,1);
			countchar++;
			tbuffer[tty_id]+TERMINAL_MAX_LINE*(tlines(tty_id)-1) += 1; // keep the head pointing the current char
		}
		return countchar;
	}



}

int kttywrite(int tty_id, void *buf, int len){

	if(tty_id < 0 || tty_id > NUM_TERMINALS)
	{
		printf("Terminal %d does not exist.", tty_id);
		return ERROR;
	}
		
	if(len < 0 || len > TERMINAL_MAX_LINE)
	{
		printf("The length of input line is illegal. Please check it.");
		return ERROR;
	}

	char *wbuf = (char *)malloc(len * sizeof(char));
	TtyTransmit(tty_id, wbuf, len);

	/*block this process and switch to next available process*/
	current_process->io_status = WAITING_WRITE;
	addQ(blockedQ, current_process);

	//TODO
	if(readyQ == NULL)
	{
		//TODO: context switch to idle process
	}
	else
	{
		//TODO: next available process
	}
}




void addQ(struct Queue *Q,struct PCB *new_pcb)
{
	struct PCB *temp;
	if(Q->front ==NULL && Q->rear==NULL)
	{
		Q->rear =new_pcb;
		Q->front=new_pcb;
		new_pcb->next=NULL;
		new_pcb->prev=NULL;
	}
	else
	{
		temp=Q->rear ;
		new_pcb->next=temp;
		new_pcb->prev=NULL;
		temp->prev=new_pcb;
		Q->rear =new_pcb;
	}


}



struct PCB * getQ(struct Queue *Q)
{
	struct PCB *temp;
	if(Q->front==NULL)
		return NULL;
	else if(Q->front==Q->rear)
	{	
		temp=Q->front;
		Q->front=NULL;
		Q->rear=NULL;
	}
	else
	{
		temp=Q->front;
		Q->front=temp->prev;
		(Q->front)->next=NULL;
		temp->prev=NULL;
	}
	return temp;
		


}

void remove_from_Q(struct Queue *Q, struct PCB *pcb)
{
	if(Q->front == NULL)
		return:
	else
	{
		/* store the front*/
		if (Q->front == pcb)
		{
			Q->front = Q->front->next;
			return;
		}
		else
		{
			while(Q->front->next!= NULL)
			{

				if(Q->front->next == pcb)
				{
					Q->front = Q->front->next->next;
				}
				Q->front = Q->front->next;
			}
		}
	}
}

