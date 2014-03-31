#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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
struct Queue wait_blockedQ;

/* Kernel Calls*/
int kfork(ExceptionStackFrame *e);
int kexec(ExceptionStackFrame *frame);
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

/* utils */
long GetPage();
int checkNpages(int n);             //To check if n free physical pages are available-- returns 0 on failure, 1 on success
void ReturnPage(int,long);
long Get_Page_table(long *);
struct PCB * getQ(struct Queue *Q);
void addQ(struct Queue *Q,struct PCB *new_pcb);
int remove_from_Q(struct Queue *Q, struct PCB *pcb);
SavedContext *MySwitchFunc(SavedContext *ctxp, void *p1, void *p2);
int LoadProgram(ExceptionStackFrame *exptr, struct PCB *proc_pcb, char *name,char **args);
void printpid(struct Queue Q);

char * tbuffer[NUM_TERMINALS];
int tlines[NUM_TERMINALS]; //initialize it to 1
long one_pgt_pages;
int pid_counter;
struct pte *pt1;
int virtual_mem_enabled;
struct PCB *idle_pcb,*active_pcb,*init_pcb;
void *mod_brk;
long pfn_head;
int rest_char_in_line = TERMINAL_MAX_LINE;
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
    struct pte *pt0;

    TracePrintf(10,"Entering KernelStart\n");
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

    /* initialize tlines*/
    int a;
    for(a = 0; a<NUM_TERMINALS;a++)
    {
        tlines[a] = 1;
    }

    addr=0;
    pid_counter=0;

    pt0=(struct pte *)malloc(PAGE_TABLE_SIZE);
    pt1=(struct pte *)malloc(PAGE_TABLE_SIZE);

    if(pt0==NULL || pt1==NULL)
		Halt();
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
    *(int *)(long)(addr)=-1;

    virtual_mem_enabled=1;
    WriteRegister(REG_VM_ENABLE, 1);
    TracePrintf(10,"Reached after initializing VM\n");




    idle_pcb=(struct PCB *)malloc (sizeof(struct PCB));
    initialize_pcb(idle_pcb);
    idle_pcb->pid=pid_counter++;
    idle_pcb->pt0_virt=(long)pt0;
    idle_pcb->pt0_phy=(long)pt0;
    idle_pcb->parent=NULL;

    active_pcb=idle_pcb;

    LoadProgram(frame,(struct PCB *)idle_pcb,"idle",&cmd_args[1]);
    //WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);

    init_pcb=(struct PCB *)malloc (sizeof(struct PCB));
    initialize_pcb(init_pcb);
    init_pcb->pid=pid_counter++;
    init_pcb->pt0_virt=(long)Get_Page_table(&(init_pcb->pt0_phy));
    init_pcb->parent=NULL;



    active_pcb=init_pcb;

    if((res=ContextSwitch(MySwitchFunc, &idle_pcb->ctxp, (void *)idle_pcb, (void *)init_pcb))==-1)
        TracePrintf(0,"\n Error while context switching");

    if(active_pcb==idle_pcb)
    {  
        TracePrintf(10,"Exiting KernelStart\n");
        return;
    }
    res=LoadProgram(frame,init_pcb,cmd_args[0],&cmd_args[1]);
    if(res!=0)
	kexit(ERROR);
    WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);



    TracePrintf(10,"Exiting KernelStart\n");
    return ;
}

int SetKernelBrk(void *addr)
{

    static int index;
    TracePrintf(10,"Entering setKernelBrk\n");
    if(!virtual_mem_enabled)
    {   if((long)addr>=VMEM_1_LIMIT)
            return -1;
    }
    else if((long)addr < (DOWN_TO_PAGE(mod_brk)))
    {   while((long)addr < (DOWN_TO_PAGE(mod_brk)))
        {   mod_brk=(void *)(DOWN_TO_PAGE(mod_brk));
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
        TracePrintf(10,"Exiting SetKernelBrk\n");
        return 0;
}

SavedContext *MySwitchFunc(SavedContext *ctxp, void *p1, void *p2)
{   
        struct pte *pt1,*pt2;
        int i,j;
        TracePrintf(10,"Entering MySwitchFunc\n");

        pt2=(struct pte *)((struct PCB*)p2)->pt0_virt;


        if((p1!=p2)&&(pt2[KERNEL_STACK_BASE/PAGESIZE].valid!=1))
        {   memcpy(&(((struct PCB*)p2)->ctxp),ctxp,sizeof(SavedContext));
            pt1=(struct pte *)((struct PCB*)p1)->pt0_virt;
            if(pt1==NULL)
                return ctxp;




            i=MEM_INVALID_PAGES;
            while(pt1[i].valid==1)
                    i++;


            for(j=KERNEL_STACK_BASE/PAGESIZE;j<KERNEL_STACK_LIMIT/PAGESIZE;j++)
            {   
                WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);
                 pt1[i].pfn = pt2[j].pfn;
                pt2[j].valid=1;
                pt2[j].uprot=0;
                pt2[j].kprot=PROT_READ|PROT_WRITE;
                pt1[i].uprot=0;
                pt1[i].kprot=PROT_READ|PROT_WRITE;
                pt1[i].valid=1;
                memcpy((void *)(i*PAGESIZE),(void *)(j*PAGESIZE),PAGESIZE);
  


            }

            pt1[i].valid=0;


        }

         WriteRegister(REG_PTR0,(RCS421RegVal)((struct PCB*)p2)->pt0_phy);   

        WriteRegister(REG_TLB_FLUSH,TLB_FLUSH_0);

        TracePrintf(10,"Exiting MySwitchFunc\n");

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
int LoadProgram(ExceptionStackFrame *exptr, struct PCB *proc_pcb, char *name,char **args)
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
    int no_of_pages=0;
    struct pte  *pt0;
    pt0=(struct pte *)proc_pcb->pt0_virt;


    TracePrintf(10,"Entering LoadProgram\n");
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
    cpp = (char **)((unsigned long)cp & (-1 << 4)); /* align cpp */
        cpp = (char **)((unsigned long)cpp - ((argcount + 4) * sizeof(void *)));

        text_npg = li.text_size >> PAGESHIFT;
    proc_pcb->brk=li.data_size + li.bss_size+li.text_size;
        data_bss_npg = UP_TO_PAGE(li.data_size + li.bss_size) >> PAGESHIFT;
        stack_npg = (USER_STACK_LIMIT - DOWN_TO_PAGE(cpp)) >> PAGESHIFT;

        TracePrintf(0, "LoadProgram: text_npg %d, data_bss_npg %d, stack_npg %d\n",
        text_npg, data_bss_npg, stack_npg);
    size=(li.data_size + li.bss_size);
    proc_pcb->brk=(text_npg+stack_npg+MEM_INVALID_PAGES)*PAGESIZE +(size/PAGESIZE)+(size%PAGESIZE);

    /*
     *  Make sure we will leave at least one page between heap and stack
     */
        if (MEM_INVALID_PAGES + text_npg + data_bss_npg + stack_npg +
        1 + KERNEL_STACK_PAGES >= PAGE_TABLE_LEN) {
        TracePrintf(0, "LoadProgram: program '%s' size too large for VM\n",name);
        free(argbuf);
        close(fd);
        TracePrintf(10,"Exiting LoadProgram\n");
        return (-1);
        }

   
    while(pt0[i++].valid==1);

    no_of_pages=text_npg + data_bss_npg + stack_npg-i;
    status=checkNpages(no_of_pages);




        if (status==0) {
        TracePrintf(0,"LoadProgram: program '%s' size too large for physical memory\n",
            name);
        free(argbuf);
        close(fd);
        return (-1);
            }

 
    exptr->sp=(void  *)cpp;



     for(i=MEM_INVALID_PAGES;i < KERNEL_STACK_BASE/PAGESIZE;i++)
     {
        if (pt0[i].valid ==1)
        {   pt0[i].kprot=PROT_ALL;
            WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);

            ReturnPage(pt0[i].pfn,i*PAGESIZE);
            pt0[i].valid=0;
        }

     }


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


    /* First, the text pages */

        for(i=MEM_INVALID_PAGES;i<text_npg+MEM_INVALID_PAGES;i++)
    {   
            pt0[i].pfn   = GetPage();
        pt0[i].valid = 1;
        pt0[i].uprot = PROT_READ | PROT_EXEC;
        pt0[i].kprot = PROT_READ | PROT_WRITE;
    }

        j=0;
    while(j<data_bss_npg)
    {   
        pt0[i].valid = 1;
            pt0[i].kprot = PROT_READ | PROT_WRITE;
                pt0[i].uprot = PROT_READ | PROT_WRITE;
                pt0[i].pfn   =GetPage();
        i++;j++;

    }

    
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


    /*
     *  All pages for the new address space are now in place.  Flush
     *  the TLB to get rid of all the old PTEs from this process, so
     *  we'll be able to do the read() into the new pages below.
     */

 

    /*
     *  Read the text and data from the file into memory.
     */
        WriteRegister(REG_TLB_FLUSH, TLB_FLUSH_0);

    if (read(fd, (void *)MEM_INVALID_SIZE, li.text_size+li.data_size)
    != li.text_size+li.data_size) {
    TracePrintf(0, "LoadProgram: couldn't read for '%s'\n", name);
    free(argbuf);
    close(fd);
    kexit(ERROR);
    return (-2);
    }

    close(fd);  
            /* we've read it all now */



    /*
     *  Now set the page table entries for the program text to be readable
     *  and executable, but not writable.
     */
    for(i=MEM_INVALID_PAGES;i<text_npg+MEM_INVALID_PAGES;i++)
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

    exptr->pc=(void *)li.entry;

    /*
     *  Now, finally, build the argument list on the new stack.
     */
        *cpp++ = (char *)argcount;      /* the first value at cpp is argc */
        cp2 = argbuf;
        for (i = 0; i < argcount; i++) {      /* copy each argument and set argv */
        *cpp++ = cp;
        strcpy(cp, cp2);
        cp += strlen(cp) + 1;
        cp2 += strlen(cp2) + 1;
        }
        free(argbuf);
        *cpp++ = NULL;  /* the last argv is a NULL pointer */
        *cpp++ = NULL;  /* a NULL pointer for an empty envp */
        *cpp++ = 0;     /* and terminate the auxiliary vector */

    /*
     *  Initialize all regs[] registers for the current process to 0,
     *  initialize the PSR for the current process also to 0.  This
     *  value for the PSR will make the process run in user mode,
     *  since this PSR value of 0 does not have the PSR_MODE bit set.
     */
  
    exptr->psr=0;

    while(i<NUM_REGS)
        exptr->regs[i++]=0;

    TracePrintf(10,"Exiting LoadProgram\n");
        return (0);
}

long Get_Page_table(long *pt_phy)
{
    static long max_allocated=VMEM_1_LIMIT-1;
    long temp,res;
    int i,index;
    struct pte *pt_new;

    TracePrintf(10,"Entering Get_page_table\n");
    //if(one_pgt_pages==-1){
        one_pgt_pages=DOWN_TO_PAGE(max_allocated);
        max_allocated=DOWN_TO_PAGE(max_allocated)-1;
        index=(one_pgt_pages/PAGESIZE)-PAGE_TABLE_LEN;


        if(pfn_head==-1||(one_pgt_pages==DOWN_TO_PAGE(mod_brk)))
        {   TracePrintf(0,"Not enought memory space");
            return -1;
        }
 

        pt1[index].pfn = GetPage();
        pt1[index].uprot=0;
        pt1[index].kprot=PROT_READ|PROT_WRITE;
        pt1[index].valid=1;
 
        *pt_phy=pt1[index].pfn*PAGESIZE;
        res=(index+PAGE_TABLE_LEN)*PAGESIZE;
        pt_new=(struct pte *)res;
        for(i=MEM_INVALID_PAGES;i<PAGE_TABLE_LEN-KERNEL_STACK_PAGES;i++)
        {   pt_new[i].pfn = -1;
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

        TracePrintf(10,"Exiting Get_page_table\n");
        return res;
    //}
    /*index=one_pgt_pages/PAGESIZE;
    temp=   (long)pt1[index].pfn*PAGESIZE;
    if(((struct pte *)index)->valid==1)
    {   one_pgt_pages=((struct pte *)index)->pfn*PAGESIZE;
        TracePrintf(10,"Exiting Get_page_table\n");
        return temp;
    }
    else
    {   index=index+PAGE_TABLE_SIZE;
        one_pgt_pages=((struct pte *)index)->pfn*PAGESIZE;
        TracePrintf(10,"Exiting Get_page_table\n");
        return (temp+PAGE_TABLE_SIZE);
    }*/




}

long GetPage()
{
    int index=MEM_INVALID_PAGES;
    long res;
    struct pte *pt0=(struct pte *)active_pcb->pt0_virt;
    TracePrintf(10,"Entering GetPage\n");
    while(pt0[index++].valid==1);
    WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)index*PAGESIZE);
   
    if (pfn_head==-1)
        return -1;
    
    pt0[index].pfn = pfn_head/PAGESIZE;
  
    pt0[index].uprot=0;
    pt0[index].kprot=PROT_READ|PROT_WRITE;
    pt0[index].valid=1;
    res=pt0[index].pfn;

    pfn_head=*(int *)(long)(index*PAGESIZE);

   
    pt0[index].valid=0;
    WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)index*PAGESIZE);

    TracePrintf(10,"Exiting GetPage\n");
    return res; 
}
void ReturnPage(int pfn,long vaddr)
{
    TracePrintf(10,"Entering Return_page\n");
    *(int *)(long)(vaddr)=pfn_head;
    

    pfn_head = (long)pfn*PAGESIZE;
 
    TracePrintf(10,"Exiting Return Page\n");

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
            e->regs[0]=kexec(e);
            break;
        case YALNIX_EXIT:
            kexit(e->regs[1]);
            break;
        case YALNIX_WAIT:
            e->regs[0]=kwait((int *)e->regs[1]);
            break;
        case YALNIX_GETPID:
            kgetpid(e);
            break;
        case YALNIX_BRK:
            e->regs[0]=kbrk((void *)e->regs[1]);
            break;
        case YALNIX_DELAY:
            e->regs[0] = kdelay(e->regs[1]);
            break;
      case YALNIX_TTY_READ:
            e->regs[0] = kttyread(e->regs[1], e->regs[2], e->regs[3]);
            break;
        case YALNIX_TTY_WRITE:
            e->regs[0] = kttywrite(e->regs[1], e->regs[2], e->regs[3]);
            break;
    }
}

void trap_clock(ExceptionStackFrame *e){
    struct PCB *temppcb= blockedQ.front;
    struct PCB * old_pcb;
    TracePrintf(0, "Enter trap clock.\n");

	while(temppcb != NULL)
            {
                if(temppcb->ticks == 0)
                {
				
                    remove_from_Q(&blockedQ, temppcb);

                    addQ(&readyQ, temppcb);
 
                }
                else
                {
                    temppcb->ticks--;
                }
                             
                temppcb = temppcb->next;
            }
   if(active_pcb->clockcount ==2)
   {
      active_pcb->clockcount = 0;
	old_pcb = active_pcb;

      active_pcb = getQ(&readyQ);
        if(active_pcb != NULL)
         {  addQ(&readyQ, old_pcb);           
        	if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)active_pcb))==-1)
              TracePrintf(0,"\n Error while context switching");
	   }
	   else
	    	active_pcb=old_pcb;	
    }
    else
    {
         active_pcb->clockcount++;
         TracePrintf(10, "in trap clock else %d \n", active_pcb->clockcount);
    }

}

void trap_illegal(ExceptionStackFrame *e){
    int code=e->code;
    char buf[128];
        char *msg = buf;
    if (code == ILL_BADSTK)
            msg = "Bad stack";
    else if (code == ILL_ILLOPC || code == ILL_ILLOPN || code == ILL_ILLADR)
            msg = "Illegal instruction";
    else if (code == ILL_PRVOPC || code == ILL_PRVREG)
            msg = "Privileged instruction";
    else if (code == ILL_COPROC)
            msg = "Coprocessor error";
    else if (code == ILL_ILLTRP)
            msg = "Illegal software trap";
    else if (code == BUS_ADRALN + 20)
            sprintf(msg, "Invalid address alignment %p", e->addr);
    else if (code == SI_KERNEL)
            msg = "Linux kernel SIGILL";
    else if (code == SI_USER)
            msg = "Received SIGILL from user";
    else
            sprintf(msg, "Unknown code 0x%x", code);    

    printf("The process '%d' is terminated forcefully due to %s ", active_pcb->pid,msg);
    kexit(ERROR);

}

void trap_memory(ExceptionStackFrame *e){

    long addr=(long)e->addr;
    int i,j,no_of_pages;
    struct pte *pt0=(struct pte *)active_pcb->pt0_virt;

    TracePrintf(0,"Entering Trap_Memory \n");
    if(addr>KERNEL_STACK_BASE)
    {   printf("\nERROR : Invalid Memory Access by process with id %d due to protection violation\n",active_pcb->pid);
        kexit(ERROR);
    }
    j=(DOWN_TO_PAGE(addr))/PAGESIZE ;
    if(j<=(DOWN_TO_PAGE(active_pcb->brk))/PAGESIZE +1)
    {   printf("\nERROR : Not Enough Memory Available for process %d\n",active_pcb->pid);
        kexit(ERROR);
    }

    i=(USER_STACK_LIMIT-PAGESIZE)/PAGESIZE;
    while(i>(DOWN_TO_PAGE(active_pcb->brk))/PAGESIZE +1)
    {
        if (pt0[i].valid==0)
            break;
        i--;
    }   
   i++;
   no_of_pages=i-j;
    if(!checkNpages(no_of_pages))
    {   printf("\nERROR : Not Enough Memory Available for process %d\n",active_pcb->pid);
        kexit(ERROR);
    }


    i--;
    while(no_of_pages>0)
    {   
        pt0[i].valid = 1;
        pt0[i].kprot = PROT_READ | PROT_WRITE;
        pt0[i].uprot = PROT_READ | PROT_WRITE;
        pt0[i].pfn   = GetPage();
      
        i--;
	  no_of_pages--;
    }
    TracePrintf(0,"Exiting Trap_memory");
    return; 



}

void trap_math(ExceptionStackFrame *e){
    char buf[128];
        char *msg = buf;    
    int code=e->code;

    switch (code) {
        case FPE_INTOVF:
                msg = "Integer overflow";
                break;
        case FPE_INTDIV:
                msg = "Integer divide by zero";
                break;
        case FPE_FLTRES:
                msg = "Floating inexact result";
                break;
        case FPE_FLTDIV:
                msg = "Floating divide by zero";
                break;
        case FPE_FLTUND:
                msg = "Floating underflow";
                break;
        case FPE_FLTINV:
                msg = "Invalid floating operation";
                break;
        case FPE_FLTSUB:
                msg = "FP subscript out of range";
                break;
        case FPE_FLTOVF:
                msg = "Floating overflow";
                break;
        case SI_KERNEL:
                msg = "Linux kernel SIGFPE";
                break;
        case SI_USER:
                msg = "Received SIGFPE from user";
                break;
        default:
                sprintf(msg, "Unknown code %d", code);
    }
    printf("The current process '%d' is terminated due to %s", active_pcb->pid,msg);

    kexit(ERROR);

}

void trap_tty_receive(ExceptionStackFrame *e){
    TracePrintf(0, "enter trap_tty_receive\n");
    TracePrintf(0, "print out readyQ:\n");
    printpid(readyQ);
    TracePrintf(0, "tlines[e->code]: %d\n", tlines[e->code]);
    if(tlines[e->code] == 1)
    {
        TracePrintf(0, " in if before malloc\n");
        tbuffer[e->code] = (char*)malloc(TERMINAL_MAX_LINE * sizeof(char) * tlines[e->code]);
        TracePrintf(0, "in if after malloc \n");
    }    
    else
    {
        TracePrintf(0, "in else before malloc\n");
        tbuffer[e->code] = (char*)realloc(tbuffer[e->code], TERMINAL_MAX_LINE * sizeof(char) * tlines[e->code]);
        TracePrintf(0, "in else after malloc\n");
    }
        

    tlines[e->code]++;



    int len = TtyReceive(e->code, tbuffer[e->code]+(TERMINAL_MAX_LINE*(tlines[e->code]-1)), TERMINAL_MAX_LINE*sizeof(char));

    /* unblock one process from blocked queue*/
    if(blockedQ.front == NULL) return;
    struct PCB *temppcb= blockedQ.front;
    while(temppcb != NULL)
    {
        if(temppcb->tid == e->code && temppcb->io_status == WAITING_READ)
        {
            temppcb->io_status = NO_STATUS;
            remove_from_Q(&blockedQ, temppcb);
            addQ(&readyQ, temppcb);
            break;
        }
        temppcb = temppcb->next;
    }

}

void trap_tty_transmit(ExceptionStackFrame *e){
    if(blockedQ.front == NULL) return;
    else
    {
         struct PCB *temppcb= blockedQ.front;
         while(temppcb != NULL)
         {
            /* unblock some process*/
            if(temppcb->tid == e->code && temppcb->io_status == WAITING_WRITE)
            {
                temppcb->io_status = NO_STATUS;
                remove_from_Q(&blockedQ, temppcb);
                addQ(&readyQ, temppcb);
                break;
            }
            temppcb = temppcb->next;
         }
    }
}

/* Kernel Calls*/
int kfork(ExceptionStackFrame *e){
    struct PCB *new_pcb,*old_pcb,*child_pcb;
    struct pte *new_pt0, *old_pt0,*pt0;
    int i,index,j,no_of_pages=0;

    TracePrintf(10,"Entering fork()\n");
    pt0=(struct pte *)active_pcb->pt0_virt;
    for(index=MEM_INVALID_PAGES;index<KERNEL_STACK_LIMIT/PAGESIZE;index++)
        if(pt0[index].valid==1)
            no_of_pages++;
    if(!checkNpages(no_of_pages+1))
    {
        printf("\nNot enough physical memory available for creating child process");
        return 0;
    } 
    new_pcb=(struct PCB *)malloc (sizeof(struct PCB));
    initialize_pcb(new_pcb);
    new_pcb->pid=pid_counter++;
    new_pcb->pt0_virt=(long)Get_Page_table(&(new_pcb->pt0_phy));

    new_pt0=(struct pte *)new_pcb->pt0_virt;

    old_pcb=active_pcb;
    old_pt0=(struct pte *)old_pcb->pt0_virt;

    i=MEM_INVALID_PAGES;
    while(pt1[i].valid==1)
          i++;
      for(j=0;j<KERNEL_STACK_BASE/PAGESIZE;j++)
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
    {   while(child_pcb->childQ!=NULL)
            child_pcb=child_pcb->childQ;
        child_pcb->childQ=new_pcb;
    }
    new_pcb->parent=old_pcb;

    e->regs[0]=new_pcb->pid;



    active_pcb=new_pcb;

    if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)new_pcb))==-1)
            TracePrintf(0,"\n Error while context switching");
      if(active_pcb->pid > old_pcb->pid)
    {   addQ(&readyQ,old_pcb);
        
        e->regs[0]=0;
        TracePrintf(10,"Exiting fork()\n");
        return 0;
    }
    TracePrintf(10,"Exiting fork()\n");
    return (int)active_pcb->pid;

}

int kexec(ExceptionStackFrame *frame)
{   
    int status;

    status=LoadProgram(frame,active_pcb,(char *)frame -> regs[1], (char **)frame -> regs[2]);
    if(status)
    {   frame->regs[0]=ERROR;
        return ERROR;

    }
    if((ContextSwitch(MySwitchFunc, &active_pcb->ctxp, (void *)NULL, (void *)active_pcb))==-1)
            TracePrintf(0,"\n Error while context switching");

    return 0;


}

void kexit(int status)
{
    int i;
    struct pte *pt0;
    struct PCB *old_pcb,*child_pcb,*ptr=NULL;
    pt0=(struct pte *)active_pcb->pt0_virt;
    old_pcb=active_pcb;
    TracePrintf(10,"Entering kexit()\n");

    for (i=MEM_INVALID_PAGES+1;i< KERNEL_STACK_LIMIT/PAGESIZE;i++)
    {
        if(pt0[i].valid==1)
        {   

            WriteRegister(REG_TLB_FLUSH,i*PAGESIZE);
            pt0[i].kprot=PROT_READ|PROT_WRITE;
            ReturnPage(pt0[i].pfn,i*PAGESIZE);



        }
    }   

   old_pcb->exit_status=status;
    if(old_pcb->parent==NULL)
        old_pcb->parent=NULL;
    else if(old_pcb->parent->child!=old_pcb)
    {   child_pcb=old_pcb->parent->child;
        while(child_pcb->childQ!=old_pcb)
            child_pcb=child_pcb->childQ;
        child_pcb->childQ=old_pcb->childQ;
    }
    else
        old_pcb->parent->child=NULL;


    child_pcb=old_pcb->child;
    while(child_pcb)
    {   child_pcb->parent=NULL;
        child_pcb=child_pcb->childQ;
    }
    if(old_pcb->parent!=NULL)
    {
        addQ(&exitedQ,old_pcb);
        ptr=old_pcb->parent;
        status=remove_from_Q(&wait_blockedQ,ptr);   
        active_pcb=ptr;             
    }

    if((ptr==NULL)||(status==-1))
    {   active_pcb=getQ(&readyQ);
       
        if(active_pcb==NULL)
              active_pcb=idle_pcb;
    }


    if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)NULL, (void *)active_pcb))==-1)
                TracePrintf(0,"\n Error while context switching\n");

    free((void *)old_pcb->pt0_virt);
    if(old_pcb->parent==NULL)
        free(old_pcb);

    TracePrintf(10,"Exiting kexit()\n");


}

int kwait(int *status_ptr){
    struct PCB *ptr;
    int temp;
    ptr=exitedQ.front;
    while(ptr)
    {   if(ptr->parent==active_pcb)
        {   *status_ptr=ptr->exit_status;
            remove_from_Q(&exitedQ,ptr);
            temp=ptr->pid;
            *status_ptr=ptr->exit_status;
            free(ptr);
            return temp;
        }
        ptr=ptr->next;

    }
    if(active_pcb->child!=NULL)
        addQ(&wait_blockedQ,active_pcb);
    else
        return ERROR;
    ptr=active_pcb;
    active_pcb=getQ(&readyQ);
    if(active_pcb==NULL)
        active_pcb=idle_pcb;
    if((ContextSwitch(MySwitchFunc, &ptr->ctxp, (void *)ptr, (void *)active_pcb))==-1)
                TracePrintf(0,"\n Error while context switching\n");
    if(active_pcb==ptr)
    {   ptr=exitedQ.front;
        while(ptr)
        {   if(ptr->parent==active_pcb)
            {   *status_ptr=ptr->exit_status;
                remove_from_Q(&exitedQ,ptr);
                temp=ptr->pid;
                *status_ptr=ptr->exit_status;
                free(ptr);
                return temp;
            }
            ptr=ptr->next;
        }
    }
    return 0;



}

int kgetpid(ExceptionStackFrame *e){
  
    e->regs[0]=active_pcb->pid;
    return active_pcb->pid;
}

int kbrk(void *a){
    struct pte *pt0=(struct pte *)active_pcb->pt0_virt;
    int i,j;
    long addr=(long)a;
    TracePrintf(10,"Entering Brk()\n");
    i=(USER_STACK_LIMIT-PAGESIZE)/PAGESIZE;
    while(pt0[i].valid==1)
        i--;
    if (addr <MEM_INVALID_PAGES ||((DOWN_TO_PAGE(addr))/PAGESIZE) >= i)
        return ERROR;
    i=DOWN_TO_PAGE(active_pcb->brk)/PAGESIZE;
    j=DOWN_TO_PAGE(addr)/PAGESIZE;
    
    if(j>i)
    {   if(checkNpages(j-i)==0)
        {   printf("\n Not enough memory available");
            return ERROR;
        }
        while(i<=j)
        {
            pt0[i].valid = 1;
            pt0[i].kprot = PROT_READ | PROT_WRITE;
                pt0[i].uprot = PROT_READ | PROT_WRITE;
                pt0[i].pfn   =GetPage();
            i++;
        }
    }
    else if(j<i)
    {
        while(j<i)
        {   
            ReturnPage(pt0[i].pfn,i*PAGESIZE);
            pt0[i].valid = 1;
            i--;
        }
    }
    active_pcb->brk=addr;
    TracePrintf(10,"Exiting Brk()\n");
    return 0;

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
        
        TracePrintf(0, "I'm in delay.\n");
        /* Add this process to blocked queue*/
        addQ(&blockedQ, active_pcb);

        /*set the current process's clock ticks*/
        active_pcb->ticks = clock_ticks;  

        next_process = getQ(&readyQ);
        struct PCB * old_pcb = active_pcb;
        // TracePrintf(0, "\n 1 in delay Check blockedQ: ");
        //     printpid(blockedQ);
        if(next_process == NULL)
        {
            TracePrintf(0, "Context Switch in kdelay");
            active_pcb = idle_pcb;
            /* If no process is ready, it would switch to init */
            if(ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)active_pcb)==-1)
                 TracePrintf(0,"\n Error while context switching");
            //TracePrintf(0, "\n 2 in delay Check blockedQ: ");
            //printpid(blockedQ);
            return 0;
        }
        else
        {
            active_pcb = next_process;
            //TracePrintf(0, "prepare for switching to next process");
             /* context switch to next available process*/
            if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)active_pcb))==-1)
                 TracePrintf(0,"\n Error while context switching");
            return 0;
        }
    }
}

int kttyread(int tty_id, void *buf, int len)
{
    TracePrintf(0, "///////////////////enter ttyread");

    if(tty_id < 0 || tty_id >= NUM_TERMINALS)
    {
        printf("Terminal %d does not exist.", tty_id);
        return ERROR;
    }

    if(len<0 || len > TERMINAL_MAX_LINE)
    {
        printf("The length of input line is out of range. Please check it.");
        return ERROR;
    }

    /* if the corresponding buffer is not null, read chars to user's buf immediately and won't block this process*/
    int countchar;
    if(tbuffer[tty_id] != NULL)
    {
        countchar = 0;
        /* copy first len chars or all available chars in this line*/
        char *this_line = tbuffer[tty_id] + TERMINAL_MAX_LINE*(tlines[tty_id]-1);
        while(countchar < len && this_line != NULL)
        {
            strncpy(buf+sizeof(char)*countchar, this_line, 1);
            countchar++;
            TracePrintf(0, "current char%s \n",this_line);
            TracePrintf(0, "track countchar: %d", countchar);
            this_line += 1; // keep the head pointing the current char
            rest_char_in_line--;
        }
        if(this_line == NULL)
        {
            tbuffer[tty_id] = tbuffer[tty_id] + rest_char_in_line; // this line is read up. point to next line.
            rest_char_in_line = TERMINAL_MAX_LINE;
            tlines[tty_id]--;// num lines in this buffer decreasing by 1
        }
        return countchar;
    }
    else
    {
        /* put this process to blockedQ */
        active_pcb->io_status = WAITING_READ;
        active_pcb->tid = tty_id;
        addQ(&blockedQ, active_pcb);

        struct PCB * old_pcb = active_pcb;
        /* switch to next available process or idle process*/
        if(readyQ.front == NULL)
            active_pcb = idle_pcb;
        else
            active_pcb = getQ(&readyQ);

        /* context switch to next process*/
        if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)active_pcb))==-1)
            TracePrintf(0,"\n Error while context switching");

        /* when coming back, tbuffer is not null, it is available to read*/
        countchar = 0;
        /* copy first len chars or all available chars in this line*/
        char *this_line = tbuffer[tty_id] + TERMINAL_MAX_LINE*(tlines[tty_id]-1);
        while(countchar < len && *this_line != NULL)
        {
            strncpy(buf+sizeof(char)*countchar, this_line, 1);
            countchar++;
            TracePrintf(0, "current char%s \n",this_line);
            TracePrintf(0, "track countchar: %d", countchar);
            this_line += 1; // keep the head pointing the current char
            rest_char_in_line--;
        }
        if(*this_line == NULL)
        {
            tbuffer[tty_id] = tbuffer[tty_id] + rest_char_in_line; // this line is read up. point to next line.
            rest_char_in_line = TERMINAL_MAX_LINE;
            tlines[tty_id]--;// num lines in this buffer decreasing by 1
        }
        return countchar;
    }

}

int kttywrite(int tty_id, void *buf, int len){
    TracePrintf(0, "enter ttywrite\n");

    if(tty_id < 0 || tty_id >= NUM_TERMINALS)
    {
        printf("Terminal %d does not exists.", tty_id);
        return ERROR;
    }

    if(len < 0 || len > TERMINAL_MAX_LINE)
    {
        printf("The length of input line is out of range. Please check it.\n");
        return ERROR;
    }

    char *wbuf = (char *)malloc(len * sizeof(char));
    memcpy(wbuf, buf, len);
    TtyTransmit(tty_id, wbuf, len);

    /* block this process and switch to next available process.*/
    active_pcb->io_status = WAITING_WRITE;
    active_pcb->tid = tty_id;
    addQ(&blockedQ, active_pcb);

    struct PCB* old_pcb = active_pcb;
    if(readyQ.front == NULL)
        active_pcb = idle_pcb;
    else
        active_pcb = getQ(&readyQ);

    if((ContextSwitch(MySwitchFunc, &old_pcb->ctxp, (void *)old_pcb, (void *)active_pcb))==-1)
            TracePrintf(0,"\n Error while context switching");

    return len;

}


void addQ(struct Queue *Q,struct PCB *new_pcb)
{
    struct PCB *temp;
    TracePrintf(10,"Entering addQ");
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
    TracePrintf(10,"Exiting addQ");

}



struct PCB * getQ(struct Queue *Q)
{
    struct PCB *temp;
    TracePrintf(10,"getQ");

    if(Q->front==NULL)
   {
   	  TracePrintf(10,"Exiting getQ");
        return NULL;
    }
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
    TracePrintf(10,"Exiting getQ");

    return temp;



}
/* returns -1 if pcb does not exist in Q ,
   returns 0 if it exists and removes it from the queue
*/
int remove_from_Q(struct Queue *Q, struct PCB *pcb)
{
    struct PCB *temp=NULL;
    TracePrintf(10,"Entering return_from_Q");
    if(Q->front == NULL)
        return -1;
    else if (Q->front == pcb)/* store the front*/
    {   if(Q->rear==pcb)
        {   Q->rear=NULL;
            Q->front=NULL;
        }
        else
        {
            Q->front = Q->front->next;
            Q->front->prev=NULL;
        }
        pcb->next=NULL;
	  TracePrintf(10,"Exiting return from Q");
        return 0;
    }
    else
    {   temp=Q->front;
        while(temp->next!= NULL)
        {
            if(temp->next == pcb)
            {
                temp->next = temp->next->next;
                temp->next->prev=temp;
                pcb->next=NULL;
                pcb->prev=NULL;
	  	    TracePrintf(10,"Exiting return from Q");
                return 0;
            }
            temp = temp->next;
        }
	  TracePrintf(10,"Exiting return from Q");
        return -1;

    }
}

int checkNpages(int n)
{   int index,i;
    long addr;
    struct pte *pt0=(struct pte *)active_pcb->pt0_virt;
    TracePrintf(10,"Entering checkNpages");
    index=MEM_INVALID_PAGES;
    addr=pfn_head;
    i=0;
    while(pt0[index++].valid!=0);
    pt0[index].pfn = addr/PAGESIZE;
    pt0[index].uprot=0;
    pt0[index].kprot=PROT_READ|PROT_WRITE;
    pt0[index].valid=1;

    WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)(index*PAGESIZE));

    while(*(int *)(long)((index)*PAGESIZE)!=-1)
    {   
        WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)(index*PAGESIZE));
        pt0[index].pfn=(*(int *)(long)((index)*PAGESIZE))/PAGESIZE;
        i++;
        if(i>n)
            break;
    }

    pt0[index].valid=0;
    WriteRegister(REG_TLB_FLUSH,(RCS421RegVal)(index*PAGESIZE));
    TracePrintf(0,"Exiting checkNpages with i>n as %d",i>n);
    if(i>n)
        return 1;
    return 0;

}

void printpid(struct Queue Q)
{
    if(Q.front == NULL)
    {
        TracePrintf(10, "this queue is null.\n");
    }
    else
    {
        while(Q.front != NULL)
        {
            TracePrintf(10, "pid: %d ", Q.front->pid);
            Q.front = Q.front->next;
        }
    }
}
