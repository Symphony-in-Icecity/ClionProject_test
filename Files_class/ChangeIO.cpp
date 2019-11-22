//
// Created by dqn on 19-3-30.
//

#include "ChangeIO.h"

void ChangeIO::change_id(vector<vector<int>> &cross_info_all, vector<vector<int>> &car_info_all,
               vector<vector<int>> &road_info_all, vector<vector<int>> &new_cross_info_all,
               vector<vector<int>> &new_car_info_all, vector<vector<int>> &new_road_info_all,
               unordered_map<int, int>& car_map, unordered_map<int, int>& road_map) {
//    vector<unordered_map<int, vector<int>>> cross_map_vec;
//    vector<unordered_map<int, vector<int>>> road_map_vec;
//    vector<unordered_map<int, vector<int>>> car_map_vec;
    unordered_map<int, int> cross_id_map;
    unordered_map<int, int> road_id_map;
    unordered_map<int, int> car_id_map;
//    unordered_map<int, int> cross_map;
//    unordered_map<int, int> road_map;
//    unordered_map<int, int> car_map;
    int cross_1 = 1;
    int car_1 = 10000;
    int road_1 = 5000;

    for (int i = 0; i < cross_info_all.size(); ++i) {
        cross_id_map[cross_info_all[i][0]] = i + cross_1;//反向字典
        new_cross_info_all[i][0] = i + cross_1;
    }
    for (int i = 0; i < road_info_all.size(); ++i) {
        road_map[i + road_1] = road_info_all[i][0];
        road_id_map[road_info_all[i][0]] = i + road_1;
        new_road_info_all[i][0] = i + road_1;
    }
    for (int i = 0; i < car_info_all.size(); ++i) {
        car_map[i + car_1] = car_info_all[i][0];
        car_id_map[car_info_all[i][0]] = i + car_1;
        new_car_info_all[i][0] = i + car_1;
    }

    for (auto &car : new_car_info_all) {
        car[1] = cross_id_map[car[1]];
        car[2] = cross_id_map[car[2]];
    }
    for (auto &road : new_road_info_all) {
        road[4] = cross_id_map[road[4]];
        road[5] = cross_id_map[road[5]];
    }
    for (auto &cross : new_cross_info_all) {
        for (int i = 1; i <= 4; ++i) {
            if (cross[i] == -1) continue;
            else cross[i] = road_id_map[cross[i]];
        }
    }
}

void ChangeIO::AnswerIO(vector<vector<int>>& planning_answer,unordered_map<int, int>& car_map, unordered_map<int, int>& road_map)
{
    for (auto& answer : planning_answer)
    {
        answer[0] = car_map[answer[0]];
        for (int i = 2; i < answer.size(); ++i) {
            answer[i] = road_map[answer[i]];
        }
    }
}