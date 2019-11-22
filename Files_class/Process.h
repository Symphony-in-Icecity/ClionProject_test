//
// Created by dqn on 19-3-28.
//

#ifndef FILES_CLASS_PROCESS_H
#define FILES_CLASS_PROCESS_H
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;

class Process {
private:
    vector<vector<float>> transpose(vector<vector<float>> A);
    vector<int> DijkstraMinPath(int start, int end, vector<vector<float>> L);
public:
    void update_loss(vector <vector<float>> &loss_array, vector <vector<int>> dis_array, vector <vector<int>> road_inf,
                     vector<float> road_percent_list, int road_id_bias, int speed, vector<float> cross_loss);
    void creat_map_array(vector <vector<int>> cross, vector <vector<int>> road, vector <vector<int>> &dis_array,
                         vector <vector<int>> &road_array, vector <vector<float>> &loss_array);
    void record_road(vector <vector<int>> batch, vector<float> &road_use_list, vector<float> &road_percent_list,
                     int road_id_bias);
    vector <vector<int>> speed_split(vector <vector<int>> car_inf);
    vector<vector<int>>
    cal_car_path(vector<vector<float>> map_loss_array, const vector<vector<int>> map_road_array,vector<vector<int>> car_inf,
                 vector<int> batch, int car_id_bias, int road_id_bias, vector<vector<int>> road_inf, int speed,int planTime,
                 vector<int> &final_time, int &all_time);
    vector <vector<int>> time_split(vector<int> group, int car_per_sec, int interval_time, int speed);
    vector<float> cal_cross_loss(vector <vector<int>> cross_inf, vector <vector<int>> road_inf);
    vector<int> update_car(vector<int> final_time, int time);
};


#endif //FILES_CLASS_PROCESS_H
