#include <iostream>
#include <iomanip>
using namespace std;
int main(){
    double price,discount;;
    int num;
    cin>>price>>num>>discount;
    double trice=price*num*discount;
    if(trice>=100)trice-=10;
    cout<<fixed<<setprecision(2)<<trice;
    return 0;
}