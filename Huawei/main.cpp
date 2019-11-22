#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <unordered_map>

using namespace std;

int car_per_sec = 26;
int interval_time = 10;
int car_in_map = 2000;

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

void write_txt(string filePath, vector<vector<int>> answer_path) {
    ofstream answer(filePath);
    if (answer.is_open()) {
        for (int i = 0; i < answer_path.size(); ++i) {
            answer << "(";
            for (int j = 0; j < answer_path[i].size(); ++j) {
                answer << to_string(answer_path[i][j]);
                if (j != answer_path[i].size() - 1) {
                    answer << ", ";
                }
            }

            answer << ")" << endl;
        }
        answer.close();
    }
}

vector<vector<float>> transpose(vector<vector<float>> A) {
    int leny = A[0].size();
    int lenx = A.size();
    vector<vector<float>> v(leny, vector<float>());
//    if (A.empty())return vector<vector<int>>();
    for (int i = 0; i < lenx; i++)
        for (int j = 0; j < leny; j++) {
            v[j].push_back(A[i][j]);
        }
    return v;
}

void update_loss(vector<vector<float>> &loss_array, vector<vector<int>> dis_array, vector<vector<int>> road_inf,
                 vector<float> road_percent_list, int road_id_bias, int speed, vector<float> cross_loss) {
    int n = (int) road_inf.size();
    int road_id;
    int road_length;
    int channel;
    int speed_lim;
    int start_node;
    int end_node;
    int isDuplex;
    float use_rate;
    double loss;
    for (int i = 0; i < n; ++i) {
        road_id = road_inf[i][0] - road_id_bias;
        road_length = road_inf[i][1];
        channel = road_inf[i][2];
        speed_lim = road_inf[i][3];
        start_node = road_inf[i][4] - 1;
        end_node = road_inf[i][5] - 1;
        isDuplex = road_inf[i][6];
        use_rate = road_percent_list[road_id];
        loss = road_length * (1.0 / min(speed, speed_lim) + 30 * use_rate / channel) +
               0.1 * max(cross_loss[start_node + 1], cross_loss[end_node + 1]);
        if (isDuplex) {
            loss_array[start_node][end_node] = float(loss);
            loss_array[end_node][start_node] = float(loss);
        } else {
            loss_array[start_node][end_node] = float(loss);
        }
    }
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

void record_road(vector<vector<int>> batch, vector<float> &road_use_list, vector<float> &road_percent_list,
                 int road_id_bias) {
    vector<int> batch_list;
    for (int i = 0; i < batch.size(); ++i) {
        batch_list = batch[i];
        for (int j = 2; j < batch_list.size(); ++j) {
            road_use_list[batch_list[j] - road_id_bias] += 1;
        }
    }
    float sum_use = 0.0;
    for (int i = 0; i < road_use_list.size(); ++i) {
        sum_use += road_use_list[i];
    }
    int n = (int) road_use_list.size();
    for (int i = 0; i < n; ++i) {
        road_percent_list[i] = road_use_list[i] / sum_use;
    }
}

vector<vector<int>> speed_split(vector<vector<int>> car_inf) {
    int max_speed = 0;
    int n = (int) car_inf.size();
    vector<int> car_list;
    vector<int> cur_group;
    for (int i = 0; i < n; ++i) {
        car_list = car_inf[i];
        if (car_list[3] > max_speed) {
            max_speed = car_list[3];
        }
    }
    vector<vector<int>> car_divide_speed(max_speed);
    vector<vector<int>> output_list(max_speed + 1);
    for (int i = 0; i < n; ++i) {
        car_list = car_inf[i];
        car_divide_speed[car_list[3] - 1].push_back(car_list[0]);
    }
    for (int j = 0; j < max_speed; ++j) {
        if (!car_divide_speed[j].empty()) {
            output_list[j + 1] = car_divide_speed[j];
        }

    }
    return output_list;
}

vector<int> DijkstraMinPath(int start, int end, vector<vector<float>> L) {
    int n = (int) L.size();
    float dis[n];//存储源点到各个顶点的最短路径
    int temp = -1;
    vector<int> path[n];
    if (start - end > 96) {
        temp = end;
        end = start;
        start = temp;
        L = transpose(L);
    }
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
    if (temp == -1) {
        return path[end];
    } else {
        reverse(path[end].begin(), path[end].end());
        return path[end];
    }
}

vector<vector<int>>
cal_car_path(vector<vector<float>> map_loss_array, const vector<vector<int>> map_road_array,
             vector<vector<int>> car_inf,
             vector<int> batch, int car_id_bias, int road_id_bias, vector<vector<int>> road_inf, int speed,
             int planTime,
             vector<int> &final_time, int &all_time) {
    srand((unsigned) time(NULL));
    vector<int> path;
    vector<vector<int>> path_road_time;
    int num_batch = (int) batch.size();
    int batch_element;
    int car_index;
    int a;
    vector<int> path_center;
    for (int i = 0; i < num_batch; i++) {
        batch_element = batch[i];
        car_index = batch_element - car_id_bias;
        path = DijkstraMinPath(car_inf[car_index][1] - 1, car_inf[car_index][2] - 1, map_loss_array);
        path_center.clear();//如果清空不好用的话就通过定义来覆盖
        a = (int) path.size();

        path_center.push_back(car_inf[car_index][0]);
        path_center.push_back(max(car_inf[car_index][4],
                                  planTime + (rand() % (interval_time / 2 + interval_time / 2 + 1)) -
                                  interval_time / 2)); // 可以在最后尝试添加随机数 (-interval_time/2, interval_time/2)

        for (int j = 0; j < (a - 1); j++) {
            path_center.push_back(map_road_array[path[j]][path[j + 1]]);
        }

        path_road_time.push_back(path_center);

        int run_time = 0;
        int num_center_index = (int) path_center.size() - 2;
        int index;
        for (int k = 0; k < num_center_index; k++) {
            index = path_center[k + 2];
            run_time += road_inf[index - road_id_bias][1] / min(speed, road_inf[index - road_id_bias][2]);//超索引报错
        }
        final_time.push_back(run_time + 1 + planTime);
        all_time += run_time;
    }
    return path_road_time;
}

//vector<vector<int>>
//car_sort_time(vector<vector<int>> group, int speed, vector<vector<int>> car_inf, vector<vector<int>> road_inf,
//              vector<vector<int>> map_dis_array, vector<vector<int>> map_road_array, int car_id_bias,
//              int road_id_bias) {
//
//}

vector<vector<int>> time_split(vector<int> group, int car_per_sec, int interval_time, int speed) //按照速度决定单位时间发车数量
{
    vector<vector<int>> group_divide_time;
    int group_len = (int) group.size();
    int car_per_batch;
    if (speed == 1 || speed == 2) {
        car_per_batch = car_per_sec * interval_time;
    } else if (speed == 3 || speed == 4) {
        car_per_batch = int(double(car_per_sec * interval_time) * 1.1);
    } else if (speed == 5 || speed == 6) {
        car_per_batch = int(double(car_per_sec * interval_time) * 1.1);
//    } else if (speed == 7 || speed == 8) {
//        car_per_batch = int(double(car_per_sec * interval_time) * 1.1);
    } else {
        car_per_batch = int(double(car_per_sec * interval_time) * 1.01);
    }
    int batch_num = group_len / car_per_batch + 1;
    for (int i = 0; i < batch_num; ++i) {
        vector<int> cur_batch;
        for (int j = 0; j < car_per_batch; ++j) {
            cur_batch.push_back(group.back());
            group.pop_back();
            if (group.empty()) break;
        }
        group_divide_time.push_back(cur_batch);
    }
    return group_divide_time;
}


vector<float> cal_cross_loss(vector<vector<int>> cross_inf, vector<vector<int>> road_inf) {
    int road_num = (int) road_inf.size();
    int road_id_bias = road_inf[0][0];
    vector<float> road_flow(road_num + road_id_bias);
    int cross_num = (int) cross_inf.size();
    vector<float> cross_loss(cross_num + 1);
    vector<int> road;
    vector<int> cross;
    float loss;
    for (int i = 0; i < road_num; ++i) { //road[3]:channel; road[2]:speed;
        road = road_inf[i];
        road_flow[road[0]] = float(road[3]) * float(road[3]) / float(road[2]);
    }
    for (int i = 0; i < cross_num; ++i) {
        loss = 0.0;
        cross = cross_inf[i];
        for (int j = 1; j <= 4; ++j) {
            if (cross[j] == -1) continue;
            for (int k = 1; k <= 4; ++k) {
                if (cross[k] == -1) continue;
                if (road_flow[cross[j]] - road_flow[cross[k]] > 0) {
                    loss += (road_flow[cross[j]] - road_flow[cross[k]]);
                } else {
                    loss += (road_flow[cross[k]] - road_flow[cross[j]]);
                }
            }
        }
        cross_loss[cross[0]] = loss;
    }
    return cross_loss;
}

vector<int> update_car(vector<int> final_time, int time) {
    vector<int> new_final_time;
    int n = (int) final_time.size();
    for (int i = 0; i < n; ++i) {
        if (final_time[i] > time) {
            new_final_time.push_back(final_time[i]);
        }
    }
    return new_final_time;
}

void change_id(vector<vector<int>> &cross_info_all, vector<vector<int>> &car_info_all,
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


// //////////////////验证程序////////////////////////////
//    for (int j = 0; j < new_car_info_all.size(); ++j) {
//        for (int i = 0; i < new_car_info_all[j].size(); ++i) {
//            cout << new_car_info_all[j][i] << "  ";
//        }
//        cout << endl;
//    }
//    for (int j = 0; j < new_cross_info_all.size(); ++j) {
//        for (int i = 0; i < new_cross_info_all[j].size(); ++i) {
//            cout << new_cross_info_all[j][i] << "  ";
//        }
//        cout << endl;
//    }
//    for (int j = 0; j < new_road_info_all.size(); ++j) {
//        for (int i = 0; i < new_road_info_all[j].size(); ++i) {
//            cout << new_road_info_all[j][i] << "  ";
//        }
//        cout << endl;
//    }
}

void AnswerIO(vector<vector<int>>& planning_answer,unordered_map<int, int>& car_map, unordered_map<int, int>& road_map)
{
    for (auto& answer : planning_answer)
    {
        answer[0] = car_map[answer[0]];
        for (int i = 2; i < answer.size(); ++i) {
            answer[i] = road_map[answer[i]];
        }
    }
}


int main() {
    vector<vector<int>> cross_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/cross.txt");
    vector<vector<int>> car_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/car.txt");
    vector<vector<int>> road_info_all = read_txt("/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/road.txt");

    vector<vector<int>> new_cross_info_all = cross_info_all;
    vector<vector<int>> new_road_info_all = road_info_all;
    vector<vector<int>> new_car_info_all = car_info_all;

    unordered_map<int, int> road_map;
    unordered_map<int, int> car_map;

    change_id(cross_info_all, car_info_all, road_info_all, new_cross_info_all, new_car_info_all, new_road_info_all, car_map, road_map);
    vector<vector<int>> dis_array;
    vector<vector<int>> road_array;
    vector<vector<float>> loss_array;

    creat_map_array(new_cross_info_all, new_road_info_all, dis_array, road_array, loss_array);
    vector<float> cross_loss = cal_cross_loss(new_cross_info_all, new_road_info_all);
    vector<vector<int>> car_divide_speed = speed_split(new_car_info_all);

    int time = 1;
    int all_time = 0;
    int car_id_bias = new_car_info_all[0][0];
    int road_id_bias = new_road_info_all[0][0];
    int road_num = new_road_info_all.size();
    vector<int> speed_list;
    vector<int> final_time;//后期检查
    vector<int> cur_group;
    vector<int> batch;
    int batch_len;
    vector<float> road_use_list(road_num + road_id_bias);
    vector<float> road_percent_list(road_num + road_id_bias);
    vector<vector<int>> planning_answer;
    vector<vector<int>> group_divide_time;
    vector<vector<int>> batch_path_time;
    int speed;

    for (int i = (int) car_divide_speed.size() - 1; i > 0; --i) {//输出是87654321，多出来一个9速的车，有可能出问题
        speed_list.push_back(i);
    }

    for (int i = 0; i < speed_list.size(); ++i) {
        speed = speed_list[i];
        cur_group = car_divide_speed[speed];
        if (cur_group.empty()) continue;
        // car_sort_time
        group_divide_time = time_split(cur_group, car_per_sec, interval_time, speed);//warning,不知道为啥是0-8共9个速度挡
        for (int j = 0; j < group_divide_time.size(); ++j) {
            batch = group_divide_time[j];
            batch_len = batch.size();
            while (final_time.size() > (car_in_map - batch_len)) {
                time += 1;
                final_time = update_car(final_time, time);
            }
            batch_path_time = cal_car_path(loss_array, road_array, new_car_info_all, batch, car_id_bias, road_id_bias,
                                           new_road_info_all, speed, time, final_time, all_time); //超索引错误
            time += interval_time;
            for (int k = 0; k < batch_path_time.size(); ++k) {
                planning_answer.push_back(batch_path_time[k]);
//                for (int l = 0; l < batch_path_time[k].size(); ++l) {//测试batch_path_time
//                    cout << batch_path_time[k][l] << " ";
//                }
//                cout << endl;
            }
//            cout << batch_path_time.size();
            record_road(batch_path_time, road_use_list, road_percent_list, road_id_bias);
            update_loss(loss_array, dis_array, new_road_info_all, road_percent_list, road_id_bias, speed, cross_loss);
        }
    }

    AnswerIO(planning_answer, car_map, road_map);
//    write_txt
//    for (int i = 0; i < car_divide_speed.size(); ++i) { //测试car_divide_speed函数
//        for (int j = 0; j < car_divide_speed[i].size(); ++j) {
//            cout << car_divide_speed[i][j] << " ";
//        }
//        cout << endl;
//    }
//    for (int i = 0; i < cross_loss.size(); ++i) { // 测试cal_cross_loss函数
//        cout << cross_loss[i] << endl;
//    }
//    cout << cross_loss.size();
//    cout << planning_answer.size() << endl;
//    for (int i = 0; i < planning_answer.size(); ++i) {//测试最终答案
//        for (int j = 0; j < planning_answer[i].size(); ++j) {
//            cout << planning_answer[i][j] << "  ";
//        }
//        cout << endl;
//    }
//    cout << planning_answer.size() << endl;
    return 0;
}