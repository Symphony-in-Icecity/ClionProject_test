#include <iostream>
#include <vector>
#include<ctime>
using namespace std;


vector<vector<int>> transpose02(vector<vector<int>> &A) {
    int leny = A[0].size();
    int lenx = A.size();
    vector<vector<int>> v(leny, vector<int>());
//    if (A.empty())return vector<vector<int>>();
    for (int i = 0; i < lenx; i++)
        for (int j = 0; j < leny; j++) {
            v[j].push_back(A[i][j]);
        }
    return v;
}

vector<vector<int> > transpose(vector<vector<int> > &matrix) {
    vector<vector<int>> v(matrix[0].size(), vector<int>());
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix[0].size(); j++) {
            v[j].push_back(matrix[i][j]);
        }
    }
    return v;
}

int main() {
    vector<vector<int> > vec(10000, vector<int>(10000, 0));
    int n = vec.size();
    clock_t startTime,endTime;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < vec[0].size(); j++) {
            vec[i][j] = (i + 1) * (j + 2);
        }
    }
    cout << "before transpose" << endl;
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < vec[0].size(); j++) {
//            cout << vec[i][j] << " ";
//        }
//        cout << "\n";
//    }
    startTime = clock();
//    vector<vector<int> > res = transpose(vec);
    vector<vector<int> > res = transpose02(vec);
    endTime = clock();
    cout << "after transpose" << endl;
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < res[0].size(); j++) {
//            cout << res[i][j] << " ";
//        }
//        cout << "\n";
//    }
    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}