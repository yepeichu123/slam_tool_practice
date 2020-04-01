#include <iostream>

using namespace std;

class A {

};

class B {
    virtual void foo() {

    }
};

class C {
    public:
        C() {

        }

        ~C() {

        }
};

int main(int argc, char** argv) {

    // 8
    A* a;
    cout << "size of A* = " << sizeof(a) << endl;

    // 1
    A b;
    cout << "size of A = " << sizeof(b) << endl;

    // 8
    B c;
    cout << "size of B with virtual function = " << sizeof(c) << endl;

    // 8
    B* d;
    cout << "size of B* with virtual function = " << sizeof (d) << endl;

    // 1
    C e;
    cout << "size of C with constructor and deconstructor = " << sizeof(e) << endl;

    // 8
    C* f;
    cout << "size of C* with constrcutor and deconstructor = " << sizeof(f) << endl;

    // 4
    cout << "size of float = " << sizeof(float) << endl;

    // 4
    cout << "size of int = " << sizeof(int) << endl;

    // 8
    cout << "size of double = " << sizeof(double) << endl;

    // 8
    cout << "size of long = " << sizeof(long) << endl;

    // 2    
    cout << "size of short = " << sizeof(short) << endl;

    // 1
    cout << "size of char = " << sizeof(char) << endl;

    return 0;
}