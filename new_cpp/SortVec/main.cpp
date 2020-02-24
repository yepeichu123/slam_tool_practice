#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

struct point2d {
    
    point2d(const int &x, const int &y) : x_(x), y_(y) {}
    
    friend ostream& operator<< (ostream &out, const point2d &p) {
        out << "[" << p.x_ << ", " << p.y_ << "]" << endl;
        return out;
    }

    int x_;
    int y_;
};

/*
* 旧c++标准包括：
* 容器的初始化;
* for循环;
* 标准库函数绑定的函数
*/
void oldSortVec();

bool cmp(const point2d &p1, const point2d &p2) {
    if (p1.x_ < p2.x_) {
        return true;
    }
    else if (p1.x_ == p2.x_) {
        if (p1.y_ <= p2.y_) {
            return true;
        }
    }
    return false;
}

/*
* 新c++标准：
* 容器的列表初始化
* for循环新方式;
* lambda表达式
*/
void newSortVec();

int main(int argc, char** argv) {

    oldSortVec();

    newSortVec();

    return 0;
}


void oldSortVec() {
    cout << "Old cpp standar!" << endl;

    vector<point2d> vec;
    vec.push_back(point2d(1, 2));
    vec.push_back(point2d(3, 3));
    vec.push_back(point2d(3, 2));
    vec.push_back(point2d(2, 3));
    vec.push_back(point2d(3, 1));
    vec.push_back(point2d(1, 3));
    vec.push_back(point2d(1, 1));
    vec.push_back(point2d(2, 2));
    vec.push_back(point2d(2, 1));

    cout << "Before sort!" << endl;
    for (int i = 0; i < vec.size(); ++i) {
        cout << vec[i];
    }

    sort(vec.begin(), vec.end(), cmp);
    cout << "After sort!" << endl;
    for (int i = 0; i < vec.size(); ++i) {
        cout << vec[i];
    }
    cout << endl;
}

void newSortVec() {
    cout << "new cpp standar!" << endl;

    vector<point2d> vec = { {1, 2}, {3, 3}, {3, 2}, {2, 3},
                        {3, 1}, {1, 3}, {1, 1}, {2, 2}, {2, 1} };
    cout << "Before sort!" << endl;
    for (int i = 0; i < vec.size(); ++i) {
        cout << vec[i];
    }

    sort(vec.begin(), vec.end(), 
            [](point2d &p1, point2d &p2) {
                if (p1.x_ < p2.x_) {
                    return true;
                }
                else if (p1.x_ == p2.x_) {
                    if (p1.y_ <= p2.y_) {
                        return true;
                    }
                }
                return false;
            });
    cout << "After sort!" << endl;
    for (int i = 0; i < vec.size(); ++i) {
        cout << vec[i];
    }
    cout << endl;
}