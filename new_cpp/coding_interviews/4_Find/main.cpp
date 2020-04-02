/*
 * 在二维数组中找到某个数值
*/

#include <iostream>
using namespace std;

bool CheckValueInArray(int* number, const int& row, const int& col, const int& value, int& count) {
    // 第一步检查指针是否有效
    if (number == nullptr || row <= 0 || col <= 0) {
        cout << "empty number array!" << endl;
        return false;
    } 

    // first method
    bool flag = true;
    count = 1;
    int r_id = 0, c_id = 0;
    int max_iterate = row * col;
    while (flag) {
        int num = number[r_id*col + c_id];
        if (num == value) {
            return true;
        }
        else if (num < value) {
            if (r_id < row-1 && c_id < col-1) {
                ++r_id;
                ++c_id;
            }
            else if (c_id == col-1 && r_id < row-1) {
                ++r_id;
            }
            else if (r_id == row-1 && c_id < col-1) {
                ++c_id;
            }
            else {
                return false;
            }
        }
        else {
            for (int i = c_id - 1; i >= 0; --i) {
                ++count;
                if (number[r_id*col + i] == value) {
                    return true;
                }
                else if (number[r_id*col + i] < value) {
                    break;
                }
            }
            for (int i = r_id - 1; i >= 0; --i) {
                ++count;
                if (number[i*col + c_id] == value) {
                    return true;
                }
                else if (number[i*col + c_id] < value) {
                    break;
                }
            }
        }
        ++count;
        if (count > max_iterate) {
            cout << "We iterate " << count << " times." << endl;
            flag = false;
        }
    }
    return false;
}

// coding interviews 
bool Find(int* number, const int& row, const int& col, const int& value, int& count) {
    bool found = false;
    
    // 检查指针
    if (number != nullptr && row > 0 && col > 0) {
        int r = 0;
        int c = col - 1;
        while (r < row && c >= 0) {
            ++count;
            if (number[r*col + c] == value) {
                found = true;
                break;
            }
            else if (number[r*col + c] > value) {
                --c;
            }
            else {
                ++r;
            }
        }
    }
    return found;
}

int main(int argc, char** argv) {

    int row = 4;
    int col = 5;
    int number[row*col] = { 1, 2, 8, 9, 12,
                            2, 4, 9, 12, 18,
                            4, 7, 10, 13, 20,
                            6, 8, 11, 15, 70};
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            cout << number[col*i + j] << ", ";
        }
        cout << endl;
    }

    // method 1 designed by myself
    int value = 14;
    int count = 0;
    bool flag = CheckValueInArray(number, row, col, value, count);
    cout << "We loop " << count << " times to find the value!" << endl;
    if (flag) {
        cout << "Ok, fine, we find the value " << value << " in array!" << endl;
    }
    else {
        cout << "Sorry, we cannot find the value " << value << " in array!" << endl;
    }

    // method 2 designed by coding interviews
    bool flag2 = Find(number, row, col, value, count);
    cout << "We loop " << count << " times to find the value!" << endl;
    if (flag2) {
        cout << "Ok, fine, we find the value " << value << " in array!" << endl;
    }
    else {
        cout << "Sorry, we cannot find the value " << value << " in array!" << endl;
    }

    return 0;
}