#include <iostream>
#include <string>
using namespace std;

// 无输出型，输入和输出都在输入列表里
void FuckLiuboshi(const string& input, string& output);

// 有输出型
string FuckLiuboshiAgain(const string& input);


int main(int argc, char** argv) {

    string input = "XiaoZeng";
    string input_2 = "Subo";
    string output, output_2;
    FuckLiuboshi(input, output);
    cout << "output = " << output << endl;

    output_2 = FuckLiuboshiAgain(input_2);
    cout << "output_2 = " << output_2 << endl;

    return 0;
}

// 无输出型，输入和输出都在输入列表里
void FuckLiuboshi(const string& input, string& output) {
    cout << "Input is : " << input << endl;
    output = input + " is Niubi.";
}

// 有输出型
string FuckLiuboshiAgain(const string& input) {
    cout << "Input is : " << input << endl;
    string output;
    output = input + " is NiubiCai.";
    return output;
}