#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <comp421/hardware.h>
#include <comp421/loadinfo.h>
#include "yalnix.h"


struct PCB{
	int pid;
	struct PCB *next;
	int REG_PTR;
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
	
	new_pcb=(struct PCB *)malloc (sizeof(struct PCB));
	new_pcb->pid=pid_counter++;
	pcb_head=new_pcb;
	LoadProgram("init",pcb_head,frame);
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
LoadProgram(char *name, char **args)
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
	//extern int pfn_head;
	struct pte *pt0;
	struct PCB *proc_pcb;
	ExceptionStackFrame *eptr;
	
	
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
    TracePrintf(0, "text_size 0x%lx, data_size 0x%lx, bss_size 0x%lx\n",
	li.text_size, li.data_size, li.bss_size);
    TracePrintf(0, "entry 0x%lx\n", li.entry);

    /*
     *  Figure out how many bytes are needed to hold the arguments on
     *  the new stack that we are building.  Also count the number of
     *  arguments, to become the argc that the new "main" gets called with.
     */
    size = 0;
    for (i = 0; args[i] != NULL; i++) {
	size += strlen(args[i]) + 1;
    }
    argcount = i;
    TracePrintf(0, "LoadProgram: size %d, argcount %d\n", size, argcount);

    /*
     *  Now save the arguments in a separate buffer in Region 1, since
     *  we are about to delete all of Region 0.
     */
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
    data_bss_npg = UP_TO_PAGE(li.data_size + li.bss_size) >> PAGESHIFT;
    stack_npg = (USER_STACK_LIMIT - DOWN_TO_PAGE(cpp)) >> PAGESHIFT;

    TracePrintf(0, "LoadProgram: text_npg %d, data_bss_npg %d, stack_npg %d\n",
	text_npg, data_bss_npg, stack_npg);

    /*
     *  Make sure we will leave at least one page between heap and stack
     */
    if (MEM_INVALID_PAGES + text_npg + data_bss_npg + stack_npg +
	1 + KERNEL_STACK_PAGES >= PAGE_TABLE_LEN) {
	TracePrintf(0, "LoadProgram: program '%s' size too large for VM\n",
	    name);
	free(argbuf);
	close(fd);
	return (-1);
    }

    /*
     *  And make sure there will be enough physical memory to
     *  load the new program.
     */
    /*The new program will require text_npg pages of text,
     data_bss_npg pages of data/bss, and stack_npg pages of
     stack.  In checking that there is enough free physical
     memory for this, BE SURE TO ALLOW FOR THE PHYSICAL MEMORY
    PAGES ALREADY ALLOCATED TO THIS PROCESS THAT WILL BE
     FREED BELOW BEFORE WE ALLOCATE THE NEEDED PAGES FOR
     THE NEW PROGRAM BEING LOADED.
	*/
	 addr=pfn_head;
	while(pt1[index++].valid!=0);
	pt1[index].pfn = (long)pfn_head;
	pt1[index].uprot=0;
	pt1[index].kprot=PROT_READ|PROT_WRITE;
	pt1[index].valid=1;
	

	while(pt1[index].pfn!=-1)
	{	pt1[index].pfn=(*(int *)(long)index*PAGESIZE)/PAGESIZE;
		no_of_pages++;
	}
    if (no_of_pages<text_npg + data_bss_npg + stack_npg+size/PAGESIZE) {
	TracePrintf(0,
	    "LoadProgram: program '%s' size too large for physical memory\n",
	    name);
	free(argbuf);
	close(fd);
	return (-1);
    }
	
    /* Initialize sp for the current process to (char *)cpp.
     The value of cpp was initialized above.
	*/
	eptr=(ExceptionStackFrame *)cp[1];
	proc_pcb=(struct PCB *)cp[0];
	eptr->sp=(char  *)cpp;
    /*
     *  Free all the old physical memory belonging to this process,
     *  but be sure to leave the kernel stack for this process (which
     *  is also in Region 0) alone.
     */
	
//	pt0=(struct pte *)ReadRegister(REG_PTR0);
	pt0=(struct pte *)malloc(PAGE_TABLE_SIZE);
	 proc_pcb->REG_PTR=pt0;
	 for(i=0;i < PAGE_TABLE_LEN;i++)
	 {	if((i*PAGESIZE >= USER_STACK_LIMIT) && (i*PAGESIZE <= KERNEL_STACK_LIMIT))
			continue;
		if (pt0[i].valid ==1)
		{	*(int *)(long)(i*PAGESIZE)=pfn_head;
			pfn_head=pt0[i].pfn*PAGESIZE;
			pt0[i].valid=0;
		}
	 
	 }
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

    //Leave the first MEM_INVALID_PAGES number of PTEs in the
    // Region 0 page table unused (and thus invalid)

    /* First, the text pages */
	WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);
    // For the next text_npg number of PTEs in the Region 0
    //page table, initialize each PTE:
		for(i=MEM_INVALID_PAGES+1;i<text_npg+MEM_INVALID_PAGES;i++)
		{   pt0[i].valid = 1;
            pt0[i].kprot = PROT_READ | PROT_WRITE;
            pt0[i].uprot = PROT_READ | PROT_EXEC;
            pt0[i].pfn   = pfn_head;
			
			pfn_head=*(int *)(long)(i*PAGESIZE);
		}

    /* Then the data and bss pages */
    // For the next data_bss_npg number of PTEs in the Region 0
    // page table, initialize each PTE:
    j=0;
		while(j<data_bss_npg)
		{   pt0[i].valid = 1;
            pt0[i].kprot = PROT_READ | PROT_WRITE;
            pt0[i].uprot = PROT_READ | PROT_WRITE;
            pt0[i].pfn   = pfn_head;
			i++;j++;
			pfn_head=*(int *)(long)(i*PAGESIZE);
		}

    /* And finally the user stack pages */
    // For stack_npg number of PTEs in the Region 0 page table
    // corresponding to the user stack (the last page of the
    // user stack *ends* at virtual address USER_STACK_LMIT),
    
	i=(USER_STACK_LIMIT-PAGESIZE)/PAGESIZE;
	j=0;
	while(j<stack_npg)
		{   pt0[i].valid = 1;
            pt0[i].kprot = PROT_READ | PROT_WRITE;
            pt0[i].uprot = PROT_READ | PROT_WRITE;
            pt0[i].pfn   = pfn_head;
			j++;i--;
			pfn_head=*(int *)(long)(i*PAGESIZE);
		}

    /*
     *  All pages for the new address space are now in place.  Flush
     *  the TLB to get rid of all the old PTEs from this process, so
     *  we'll be able to do the read() into the new pages below.
     */
    WriteRegister(REG_TLB_FLUSH, TLB_FLUSH_0);

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

    close(fd);			/* we've read it all now */

    /*
     *  Now set the page table entries for the program text to be readable
     *  and executable, but not writable.
     */
    //>>>> For text_npg number of PTEs corresponding to the user text
    //>>>> pages, set each PTE's kprot to PROT_READ | PROT_EXEC.
	for(i=MEM_INVALID_PAGES+1;i<text_npg+MEM_INVALID_PAGES;i++)
		{  
			pt0[i].kprot = PROT_READ | PROT_EXEC;
		}

    WriteRegister(REG_TLB_FLUSH, TLB_FLUSH_0);

    /*
     *  Zero out the bss
     */
    memset((void *)(MEM_INVALID_SIZE + li.text_size + li.data_size),
	'\0', li.bss_size);

    /*
     *  Set the entry point in the exception frame.
     */
    //>>>> Initialize pc for the current process to (void *)li.entry
	eptr->pc=(void *)li.entry;
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
	eptr->psr=0;
	while(i<NUM_REGS)
		eptr->regs[i++]=0;

    return (0);
}
