/*
 * 找出数组中任意一个重复的数字
 * 解法：重排数组，并构建哈希表
 * 每一个下标i对应的数值x1应该等于下标i，若不等，则将该数值作为下标x1查找另一个值x2，若x1==x2，则我们找到了重复的数字。
 * 若还是不等，则交换数值。此时下标i的数值为x2，利用x2作为下标继续查找，知道找到重复的数字为止。
*/

#include <iostream>
#include <cstring>
using namespace std;

int GetSize(int data[]) {
    return sizeof(data);
}

// part 2
bool duplicate(int number[], int length, int& duplication) {

    cout << "Enter function duplicate!" << endl;

    // 第一步需要确保输入数组和长度有效
    if (number == nullptr || length <= 0) {
        cout << "Sorry, you input the empty array!" << endl;
        return false;
    }

    // 第二步需要确保输入的数字必须在0~n-1之间
    for (int i = 0; i < length; ++i) {
        if (number[i] < 0 || number[i] > length-1) {
            cout << "Sorry, you input the value in number array is out of range!" << endl;
            return false;
        }
    }

    // 第三步才是我们真正需要实现的内容
    for (int i = 0; i < length; ) {
        int num = number[i];
        if (num == i) {
            ++i;
            continue;
        }
        else {
            int next = number[num];
            if (next == num) {
                duplication = number[num];
                cout << "Oh yeah, we find the repeated number : "<< duplication << endl;
                return true;
            }
            else {
                number[num] = num;
                number[i] = next;
            }
        }
    }

    return false;
}

// part 3
bool duplicateWithoudChangeArray(const int number[], int length, int& duplication) {

    cout << "Enter function duplicateWithoudChangeArray!" << endl;

    // 第一步检查数组和长度是否有效
    if (number == nullptr || length <= 0) {
        cout << "empty number array or invalid length!" << endl;
        return false;
    }

    // 第二步检查数组是否超过长度
    for (int i = 0; i < length; ++i) {
        if (number[i] <= 0 || number[i] > length-1) {
            cout << "number array is out of range!" << endl;
            return false;
        }
    }

    // 第三步常规查找重复数值
    /*
    for (int i = 0; i < length; ++i) {
        for (int j = i+1; j < length; ++j) {
            if (number[i] == number[j]) {
                duplication = number[i];
                cout << "We find the repeated number : " << duplication << endl;
                return true;
            }
        }
    }*/

    int start = 1;
    int end = length - 1;
    while (start < end) {
        int middle = (length + start) / 2;
        int count = 0;
        for (int i = start; i <= end; ++i) {
            if (number[i] <= middle) {
                ++count;
            }
            if (count > middle) {
                end = middle; 
            }
            else {
                start = middle+1;
            }
        }
    }
    duplication = start;
    cout << "We find the repeated number : " << duplication << endl;
    return true;
}

int main() {

    // part 1
    // sizeof
    int data[] = {1, 2, 3, 4, 5};
    int size1 = sizeof(data);
    int *data2 = data;
    int size2 = sizeof(data2);
    int size3 = GetSize(data);

    cout << size1 << ", " << size2 << ", " << size3 << endl;

    // part 2
    int repeat_num;
    int number1[] = {5, 2, 3, 0, 1, 4, 6, 3};
    int leng = sizeof(number1) / sizeof(number1[0]);
    if (duplicate(number1, leng, repeat_num)) {
        cout << "We found the repeated number : " << repeat_num << endl;
    }
    
    int repeat_num2;
    int number2[] = {};
    int leng2 = sizeof(number2) / sizeof(number2[0]);
    if (duplicate(number2, leng2, repeat_num)) {
        cout << "We found the repeated number : " << repeat_num2 << endl;
    }

    int repeat_num3;
    int number3[] = {5, 2, 3, 0};
    int leng3 = sizeof(number3) / sizeof(number3[0]);
    if (duplicate(number3, leng3, repeat_num)) {
        cout << "We found the repeated number : " << repeat_num3 << endl;
    }

    // part 3
    int repeat_num4;
    int number4[] = {5, 2, 3, 7, 1, 4, 6, 3};
    int leng4 = sizeof(number4) / sizeof(number4[0]);
    if (duplicateWithoudChangeArray(number4, leng4, repeat_num4)) {
        cout << "We found the repeated number : " << repeat_num4 << endl;
    }

    return 0;
}