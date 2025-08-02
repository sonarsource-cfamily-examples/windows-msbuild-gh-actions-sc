void readData(unsigned short int src[],unsigned short int dest[],unsigned int datalen)
{
	for(int i = 0;i<datalen;i++)
	{
		dest[i] = src[i];
	}
}
int testA()
{
	unsigned short int time[6];
	unsigned short int src[6]={1,2,3,4,5,6};
	for(int i = 0;i<5;i++)
	{
		time[i] = src[i];
	}
	readData(src,time,6);
}