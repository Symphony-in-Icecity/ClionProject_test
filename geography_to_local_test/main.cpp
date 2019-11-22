#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <math.h>
using namespace std;

#define M_DEG_TO_RAD 1/57.295
#define CONSTANTS_RADIUS_OF_EARTH 6371000

vector<double> world_to_local(const double ref_lat, const double ref_lon, double lat, double lon) {

    double lat_rad = lat * M_DEG_TO_RAD; // 度 -> 弧度 A/57.295
    double lon_rad = lon * M_DEG_TO_RAD; // GPS数据角度单位为弧度
    double ref_lat_rad = ref_lat * M_DEG_TO_RAD;
    double ref_lon_rad = ref_lon * M_DEG_TO_RAD;
    double sin_lat = sin(lat_rad); //程序中三角运算使用的是弧度
    double cos_lat = cos(lat_rad);
    double sin_ref_lat = sin(ref_lat_rad);
    double cos_ref_lat = cos(ref_lat_rad);
//    double sin_ref_lon = sin(ref_lon_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);
    double arg = sin_ref_lat * sin_lat + cos_ref_lat * cos_lat * cos_d_lon;
    double x;
    double y;

    if (arg > 1.0) {
        arg = 1.0;
    } else if (arg < -1.0) {
        arg = -1.0; //限幅
    }
    double c = acos(arg);
    double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));// c为正数
    x = k * (cos_ref_lat * sin_lat - sin_ref_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    y = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

    return {x, y};
}

void race_track_to_local(vector<sensor_msgs::NavSatFix> race_track, double &yaw, vector<double> &origin_local) {
    if (race_track.size() != 4) {
        return;
    } else {
        vector<double> world_origin = {0.5 * (race_track[0].latitude + race_track[1].latitude),
                                       0.5 * (race_track[0].longitude + race_track[1].longitude)};
        vector<vector<double>> four_base_point_local = {
                world_to_local(world_origin[0], world_origin[1], race_track[0].latitude, race_track[0].longitude),
                world_to_local(world_origin[0], world_origin[1], race_track[1].latitude, race_track[1].longitude),
                world_to_local(world_origin[0], world_origin[1], race_track[2].latitude, race_track[2].longitude),
                world_to_local(world_origin[0], world_origin[1], race_track[3].latitude, race_track[3].longitude)
        };
        yaw = atan2((four_base_point_local[2][1] + four_base_point_local[3][1]) / 2, (four_base_point_local[2][0] + four_base_point_local[3][0]) / 2);
        origin_local = {0.5 * (four_base_point_local[0][0] + four_base_point_local[1][0]),
                        0.5 * (four_base_point_local[0][1] + four_base_point_local[1][1])
        };
    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    double x;
    double y;

    return 0;
}