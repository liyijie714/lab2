void addQ(struct Queue *Q,struct PCB *new_pcb)
{
	struct PCB *temp;
	if(Q->first ==NULL && Q->rear==NULL)
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

