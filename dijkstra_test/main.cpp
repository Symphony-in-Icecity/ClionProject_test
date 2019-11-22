#include<iostream>
#include <vector>
#include <fstream>
#include <algorithm>

using namespace std;

void SplitString(const string &s, vector<string> &v, const string &c) {
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

vector<vector<int>> read_txt(string filePath) {
    ifstream file(filePath);
    string file_str;
    getline(file, file_str);
    int num = count(file_str.begin(), file_str.end(), ',');
    vector<int> file_info(num + 1);
    vector<vector<int>> file_info_all;
    while (getline(file, file_str)) {
        vector<string> v;
        file_str.erase(file_str.begin());
        file_str.erase(file_str.end() - 1);
        SplitString(file_str, v, ", ");
        for (int i = 0; i < file_info.size(); i++) {
            file_info[i] = stoi(v[i]);
        }
        file_info_all.push_back(file_info);
    }
    return file_info_all;
}

void creat_map_array(vector<vector<int>> cross, vector<vector<int>> road, vector<vector<int>> &dis_array,
                     vector<vector<int>> &road_array, vector<vector<float>> &loss_array) {
    int a = (int) cross.size();
    int n = (int) road.size();
    int road_id;
    int road_length;
    int start_node;
    int end_node;
    int isDuplex;
    vector<int> dis_vector(a);
    vector<float> loss_vector(a);
    vector<int> road_vector;
    for (int i = 0; i < a; ++i) {
        for (int j = 0; j < a; ++j) {
            if (i != j) {
                dis_vector[j] = 100000;
                loss_vector[j] = 100000.0;
            } else {
                dis_vector[j] = 0;
                loss_vector[j] = 0.0;
            }
        }
        dis_array.push_back(dis_vector);
        loss_array.push_back(loss_vector);
    }
    for (int i = 0; i < a; ++i) {
        for (int j = 0; j < a; ++j) {
            road_vector.push_back(0);
        }
        road_array.push_back(road_vector);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < road[i].size(); ++j) {
            road_id = road[i][0];
            road_length = road[i][1];
            start_node = road[i][4] - 1;
            end_node = road[i][5] - 1;
            isDuplex = road[i][6];
            if (isDuplex == 1) {
                dis_array[start_node][end_node] = road_length;
                dis_array[end_node][start_node] = road_length;
                loss_array[start_node][end_node] = (float) road_length;
                loss_array[end_node][start_node] = (float) road_length;
                road_array[start_node][end_node] = road_id;
                road_array[end_node][start_node] = road_id;
            } else {
                dis_array[start_node][end_node] = road_length;
                loss_array[start_node][end_node] = (float) road_length;
                road_array[start_node][end_node] = road_id;
            }
        }
    }
}


vector<int> DijkstraMinPath(int start, int end, vector<vector<float>> L) {
    int n = (int) L.size();
    float dis[n];        //存储源点到各个顶点的最短路径
    vector<int> path[n];
    for (int i = 0; i < n; i++) //initial
    {
        dis[i] = L[start][i];
        path[i].push_back(start);//初始化起点位置?
        path[i].push_back(i);
    }
    for (int i = 1; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (dis[i] > dis[j] + L[j][i]) {
                dis[i] = dis[j] + L[j][i];
                path[i].clear();
                path[i].insert(path[i].end(), path[j].begin(), path[j].end());
                path[i].push_back(i);
            }
        }
        for (int m = 0; m < i; m++) {
            for (int k = 0; k < n; k++) {
                if (dis[m] > dis[k] + L[k][m]) {
                    dis[m] = dis[k] + L[k][m];
                    path[m].clear();                     //保存并更新路径
                    path[m].insert(path[m].end(), path[k].begin(), path[k].end());
                    path[m].push_back(m);
                }
            }
        }
    }
    return path[end];
}
int main()
{
    vector<vector<int>> cross_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/cross.txt");
    vector<vector<int>> car_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/car.txt");
    vector<vector<int>> road_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/road.txt");

    vector<vector<int>> dis_array;
    vector<vector<int>> road_array;
    vector<vector<float>> loss_array;
    vector<int> path;
    creat_map_array(cross_info_all, road_info_all, dis_array, road_array, loss_array);

    path = DijkstraMinPath(98, 1, loss_array);
    for (int i = 0; i < path.size(); ++i) {
        cout << path[i] << " ";
    }
    return 0;
}