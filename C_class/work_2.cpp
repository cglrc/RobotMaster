#include <iostream>
#include <iomanip>
#include <windows.h>//不加会报错
using namespace std;

int main() {
    SetConsoleOutputCP(CP_UTF8); //
    const double PI = 3.14159;
    double r, h;
    cout << " r: ";
    cin >> r;
    cout << " h: ";
    cin >> h;
    cout << fixed << setprecision(2)
         << "圆周长=" << 2 * PI * r << '\n'
         << "圆面积=" << PI * r * r << '\n'
         << "圆球表面积=" << 4 * PI * r * r << '\n'
         << "圆球体积=" << 4.0 / 3 * PI * r * r * r << '\n'
         << "圆柱体积=" << PI * r * r * h << '\n';
    return 0;
}