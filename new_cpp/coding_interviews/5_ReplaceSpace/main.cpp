#include <iostream>

using namespace std;

// part 1
void CompareArray() {
    char str1[] = "hello world";
    char str2[] = "hello world";

    if (str1 == str2) {
        cout << "str1 is the same as str2." << endl;
    }
    else {
        cout << "str1 is differet from str2." << endl;
    }

    char* str3 = "hello world";
    char* str4 = "hello world";

    if (str3 == str4) {
        cout << "str3 is the same as str4." << endl;
    }
    else {
        cout << "str3 is differet from str4." << endl;
    }
}

// part 2
bool ReplaceSpace() {

}

int main(int argc, char** argv) {

    CompareArray();

    return 0;
}