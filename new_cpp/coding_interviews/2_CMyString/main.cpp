#include <iostream>
#include <cstring>
using namespace std;

class CMyString {
    public:
        CMyString(char* pData = nullptr) {
            cout << "Default constructor!" << endl;
            this->m_pData = pData;
        }

        CMyString(const CMyString& str) {
            cout << "copy constructor!" << endl;
            this->m_pData = str.m_pData;
        }
        
        CMyString& operator=(const CMyString& str) {
            cout << "Assignment constructor!" << endl;

            if (this == &str) {
                cout << "self assigned." << endl;
            }
            else {
                cout << "Not self assigned." << endl;
                CMyString strTemp(str);

                char* pTemp = strTemp.m_pData;
                strTemp.m_pData = this->m_pData;
                this->m_pData = pTemp;
            }
            return *this;
        }

        ~CMyString() {
            cout << "Deconstructor!" << endl;
            if (m_pData != nullptr) {
                delete m_pData;
                m_pData = nullptr;
            }
        }
        
        char GetData() {
            if (m_pData != nullptr) 
                return *m_pData;
            cout << "empty m_pData!" << endl;
            return 's';
        }
    
    private:
        char* m_pData;
};


int main(int argc, char** argv) {

    // CMyString str;

    char* name = new char('a');
    CMyString* str_2 = new CMyString(name);
    cout << "str_2.GetData() = " << str_2->GetData() << endl;
    *str_2 = *str_2;
    cout << "str_2.GetData() = " << str_2->GetData() << endl;

    CMyString str_3;
    str_3 = *str_2;
    cout << "str_3.GetData() = " << str_3.GetData() << endl;

    if (str_2 != nullptr) {
        delete str_2;
        str_2 = nullptr;
    }

    return 0;
}