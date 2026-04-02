#include <stdio.h>
#include <windows.h>
int main()
{
    SetConsoleOutputCP(CP_UTF8);
    int num=0;
    int prime=1;
    begin:
    scanf("%d",&num);
    if(num<3){
        goto begin;
    }
    for(int i=2;i<num;i++){
        if(num%i==0){
            prime=0;
            break;
        }else{
            prime=1;
        }
    }
    if(prime==1){
        printf("%d是素数\n",num);
    }else if(prime==0){
        printf("%d不是素数\n",num);
    }
    return 0;
 }