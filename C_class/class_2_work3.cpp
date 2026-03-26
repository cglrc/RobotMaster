#include <iostream>
#include <windows.h>
using namespace std;
int main() {
    SetConsoleOutputCP(CP_UTF8);
    int num=-1,digit; 
    while(num<0||num>999){
        cin >> num;
        if(num<0||num>999){
            cout << "重输：" ;
        }
    }
    if (num >= 100) {
        digit = 3;
    } else if (num >= 10) {
        digit = 2;
    } else {
        digit = 1;
    }
    cout << "位数：" << digit << endl;
    int a = num / 100;
    int b = (num / 10) % 10;
    int c = num % 10;
    cout << "各位数字：";
    if (digit == 3) {
        cout << a << " " << b << " " << c;
    } else if (digit == 2) {
        cout << b << " " << c;
    } else {
        cout << c;
    }
    cout << endl;
    cout << "逆序数字：";
    if (digit == 3) {
        cout << c << b << a;
    } else if (digit == 2) {
        cout << c << b;
    } else {
        cout << c;
    }
     return 0;
}