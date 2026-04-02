#include <stdio.h>
#include <windows.h>
int main()
{
    SetConsoleOutputCP(CP_UTF8);
    for(int i=1;i<10;i++){
        for(int j=1;j<=i;j++){
            printf("%d*%d=%d ",i,j,i*j);
            if(i==j){
               printf("\n");     
            }
        }
        printf("\n");
    }
   return 0; 
 }