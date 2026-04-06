```c
#include <stdio.h>
#include <windows.h>
int main()
{
    SetConsoleOutputCP(CP_UTF8);
    
    for(int i=200;i<=300;i++){
        for(int j=2;j<i;j++){
            if(i%j==0){
                break;
                
            }else if(j==i-1){
                printf("%d ",i);
            }
                
        }
   
 }return 0;
}