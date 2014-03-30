/* returns -1 if pcb does not exist in Q ,
   returns 0 if it exists and removes it from the queue
*/
int remove_from_Q(struct Queue *Q, struct PCB *pcb)
{
	struct PCB *temp=NULL;
	if(Q->front == NULL)
		return -1;
	else if (Q->front == pcb)/* store the front*/
	{	if(Q->rear==pcb)
		{	Q->rear=NULL;
			Q->front=NLL;
		}
		else
		{
			Q->front = Q->front->next;
			Q->front->prev=NULL;
		}
		pcb->next=NULL;
		return 0;
	}
	else
	{	temp=Q->front;
		while(temp->next!= NULL)
		{
			if(temp->next == pcb)
			{
				temp->next = temp->next->next;
				temp->next->prev=temp;
				pcb->next=NULL;
				pcb->prev=NULL;
				return 0;
			}
			temp = temp->next;
		}
		return -1;
		
	}
}
