```c
#include<stdio.h>
int main(){
    int count=0,i=0;
    char str[16];
    gets(str);
    while(str[i]!='\0'){
        while(str[i]==' '){
            
            i++;
            if(str[i]!='\0'){
                count++;
                while(str[i]!=' '&&str[i]!='\0'){
                    i++;
                }
            }
        }
    
}
    printf("%d",count);
    return 0;
}


