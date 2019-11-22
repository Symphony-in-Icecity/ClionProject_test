//
// Created by dqn on 19-3-28.
//

#include "FilesIO.h"

vector <vector<int>> FilesIO::read_txt(string filePath) {
    ifstream file(filePath);
    string file_str;
    getline(file, file_str);
    int num = count(file_str.begin(), file_str.end(), ',');
    vector<int> file_info(num + 1);
    vector <vector<int>> file_info_all;
    while (getline(file, file_str)) {
        vector <string> v;
        file_str.erase(file_str.begin());
        file_str.erase(file_str.end() - 1);
        SplitString(file_str, v, ", ");
        for (int i = 0; i < (int)file_info.size(); i++) {
            file_info[i] = stoi(v[i]);
        }
        file_info_all.push_back(file_info);
    }
    return file_info_all;
}

void FilesIO::SplitString(const string &s, vector<string> &v, const string &c) {
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2) {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}
void FilesIO::write_txt(string filePath, vector<vector<int>> answer_path) {
    ofstream answer(filePath);
    if (answer.is_open()) {
        for (int i = 0; i < (int)answer_path.size(); ++i) {
            answer << "(";
            for (int j = 0; j < (int)answer_path[i].size(); ++j) {
                answer << to_string(answer_path[i][j]);
                if (j != (int)answer_path[i].size() - 1) {
                    answer << ", ";
                }
            }

            answer << ")" << endl;
        }
        answer.close();
    }
}