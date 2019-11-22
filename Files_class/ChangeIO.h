//
// Created by dqn on 19-3-30.
//

#ifndef FILES_CLASS_CHANGEIO_H
#define FILES_CLASS_CHANGEIO_H

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <unordered_map>

using namespace std;

class ChangeIO {
public:
    void change_id(vector<vector<int>> &cross_info_all, vector<vector<int>> &car_info_all,
                   vector<vector<int>> &road_info_all, vector<vector<int>> &new_cross_info_all,
                   vector<vector<int>> &new_car_info_all, vector<vector<int>> &new_road_info_all,
                   unordered_map<int, int> &car_map, unordered_map<int, int> &road_map);
    void AnswerIO(vector<vector<int>>& planning_answer,unordered_map<int, int>& car_map, unordered_map<int, int>& road_map);
};


#endif //FILES_CLASS_CHANGEIO_H
