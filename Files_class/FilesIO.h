//
// Created by dqn on 19-3-28.
//

#ifndef FILES_CLASS_FILESIO_H
#define FILES_CLASS_FILESIO_H

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;

class FilesIO {
private:
    void SplitString(const string &s, vector <string> &v, const string &c);

public:
    vector <vector<int>> read_txt(string filePath);
    void write_txt(string filePath, vector <vector<int>> answer_path);

};


#endif //FILES_CLASS_FILESIO_H
