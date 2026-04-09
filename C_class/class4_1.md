```c
#include<stdio.h>
int main()
{
    int num[10];
    int test;
    for(int i=0;i<10;i++)
    {
        scanf("%d",&test);
        if(test>100||test<0)
        {
            printf("输入错误");
        }else
        {
            num[i]=test;
        }
    }
    for(int i=0;i<10;i++)
    {
        if(num[i]>num[i+1])
        {
            int temp=num[i];
            num[i]=num[i+1];
            num[i+1]=temp;
        }
    }
    for(int i=0;i<10;i++)
    {
        printf("%d ",num[i]);
    }
    return 0;

}