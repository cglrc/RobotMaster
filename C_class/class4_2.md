```c
#include<stdio.h>
int main(){
    int num[3][4];
    
    
    int row=0;
    int col=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            scanf("%d",&num[i][j]);
    }
}
    
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            if(num[i][j]>num[row][col]){
                row=i;
                col=j;
            }
        }
    }
    printf("%d",num[row][col]);
    printf("%d",row);
    printf("%d",col);
    return 0;   
}