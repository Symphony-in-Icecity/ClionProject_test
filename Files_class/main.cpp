#include <iostream>
#include "FilesIO.h"
#include "Process.h"

int main(int argc, char *argv[]) {
    cout << "Begin" << endl;

//    if (argc < 5) {
//        cout << "please input args: carPath, roadPath, crossPath, answerPath" << endl;
//        exit(1);
//    }

//    string carPath(argv[1]);
//    string roadPath(argv[2]);
//    string crossPath(argv[3]);
//    string answerPath(argv[4]);
    string carPath = "/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/car.txt";
    string roadPath = "/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/road.txt";
    string crossPath = "/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/cross.txt";
    string answerPath = "/home/dqn/Huawei/SDK_python/CodeCraft-2019/config/answer_class.txt";

    cout << "carPath is " << carPath << endl;
    cout << "roadPath is " << roadPath << endl;
    cout << "crossPath is " << crossPath << endl;
    cout << "answerPath is " << answerPath << endl;

    int car_per_sec = 30;
    int interval_time = 10;
    int car_in_map = 3000;
    clock_t pro_start_time ;
    clock_t pro_end_time;

    pro_start_time = clock();
    // TODO:read input filebuf
    FilesIO read_and_write;
    Process process;
    vector <vector<int>> cross_info_all = read_and_write.read_txt(crossPath);
    vector <vector<int>> car_info_all = read_and_write.read_txt(carPath);
    vector <vector<int>> road_info_all = read_and_write.read_txt(roadPath);

    vector <vector<int>> dis_array;
    vector <vector<int>> road_array;
    vector <vector<float>> loss_array;

    process.creat_map_array(cross_info_all, road_info_all, dis_array, road_array, loss_array);
    vector<float> cross_loss = process.cal_cross_loss(cross_info_all, road_info_all);
    vector <vector<int>> car_divide_speed = process.speed_split(car_info_all);

    int time = 1;
    int all_time = 0;
    int car_id_bias = car_info_all[0][0];
    int road_id_bias = road_info_all[0][0];
    int road_num = road_info_all.size();
    vector<int> speed_list;
    vector<int> final_time;//后期检查
    vector<int> cur_group;
    vector<int> batch;
    int batch_len;
    vector<float> road_use_list(road_num + road_id_bias);
    vector<float> road_percent_list(road_num + road_id_bias);
    vector <vector<int>> planning_answer;
    vector <vector<int>> group_divide_time;
    vector <vector<int>> batch_path_time;
    int speed;

    for (int i = (int) car_divide_speed.size() - 1; i > 0; --i) {
        speed_list.push_back(i);
    }

    for (int i = 0; i < (int) speed_list.size(); ++i) {
        speed = speed_list[i];
        cur_group = car_divide_speed[speed];
        if (cur_group.empty()) continue;
        // car_sort_time日后补充
        group_divide_time = process.time_split(cur_group, car_per_sec, interval_time, speed);//warning,不知道为啥是0-8共9个速度挡
        for (int j = 0; j < (int) group_divide_time.size(); ++j) {
            batch = group_divide_time[j];
            batch_len = batch.size();
            while ((int) final_time.size() > (car_in_map - batch_len)) {
                time += 1;
                final_time = process.update_car(final_time, time);
            }
            batch_path_time = process.cal_car_path(loss_array, road_array, car_info_all, batch, car_id_bias, road_id_bias,
                                           road_info_all, speed, time, final_time, all_time); //超索引错误
            time += interval_time;
            for (int k = 0; k < (int) batch_path_time.size(); ++k) {
                planning_answer.push_back(batch_path_time[k]);
            }
            process.record_road(batch_path_time, road_use_list, road_percent_list, road_id_bias);
            process.update_loss(loss_array, dis_array, road_info_all, road_percent_list, road_id_bias, speed, cross_loss);
        }
    }
    pro_end_time = clock();
    cout << "The run time is: " <<(double)(pro_end_time - pro_start_time) / CLOCKS_PER_SEC << "s" << endl;
    // TODO:write output file
    read_and_write.write_txt(answerPath, planning_answer);

    return 0;
}