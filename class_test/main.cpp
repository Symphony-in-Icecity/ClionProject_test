#include <iostream>
#include <iomanip>
#include <cfloat>
#include "Spline.h"
#include "RangeImpl.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"

using namespace CubicSplinePlanner;

double MAX_SPEED = 50.0 / 3.6; // 最大速度[m/s]
double MAX_ACCEL = 5.0; // 最大加速度[m/ss]
double MAX_CURVATURE = 10.0; // 最大曲率[1/m]
double MAX_ROAD_WIDTH = 7.0; // 最大道路宽度[m]
double D_ROAD_W = 1.0; // 道路宽度方向采样长度，即几米一个格[m]
double DT = 0.2; // 时钟刻度，是预测的局部路径的最小时间单位[s]
double MAXT = 5.0; // 最大预测时间[s]
double MINT = 4.0;  // 最小预测时间[s]
double TARGET_SPEED = 50.0 / 3.6; // 目标速度，无人机要维持的速度[m/s]
double D_T_S = 5.0 / 3.6; // 目标速度采样长度[m/s]
int N_S_SAMPLE = 1; // 目标速度取样个数【个】
double ROBOT_RADIUS = 2.0; // 机器人半径[m]

// cost weights
double KJ = 0.01;// 加速度突变
double KT = 0.01;// 制动速度
double KD = 0.01;// 偏离道路中心
double KS = 1.0;// 速度保持
double KLAT = 10.0;// 横向权重
double KLON = 1.0;// 纵向权重

struct Frenet_path {
    vector<double> t;       //time
    vector<double> d;       // 横向距离
    vector<double> d_d;     // 横向速度
    vector<double> d_dd;    // 横向加速度
    vector<double> d_ddd;   //横向加加速度
    vector<double> s;       //纵向距离
    vector<double> s_d;     //纵向速度
    vector<double> s_dd;    //纵向加速度
    vector<double> s_ddd;   //纵向加加速度
    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;
    vector<double> x;
    vector<double> y;
    vector<double> yaw; // yaw
    vector<double> ds; // distance
    vector<double> c;// curvature
};

bool check_collision(Frenet_path fp, const MatrixXd& ob){
    vector<double> d;
    bool collision;
    for (int i = 0; i < ob.rows(); ++i) {
        for (int j = 0; j < fp.x.size(); ++j) {
            d.push_back(pow(fp.x[j] - ob(i, 0), 2) + pow(fp.y[j] - ob(i, 1), 2));
        }
        collision = any_of(d.begin(), d.end(), [](double di){return di <= ROBOT_RADIUS * ROBOT_RADIUS;});
        if (collision)
        {
            return false;
        }
    }
    return true;
}

vector<Frenet_path> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0)
{
    vector<Frenet_path> frenet_paths;

    double Jd;
    double Js;
    double ds;
    vector<double> SquareOfJerks_d_ddd;
    vector<double> SquareOfJerks_s_ddd;
    // 由末端状态的不同生成每一条可能的路径（末端取决于将道路分成了几块）
    for (auto di : Cosmos::Range(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W))
    {
        for (auto Ti : Cosmos::Range(MINT, MAXT, DT)) // Lateral motion planning横向的运动规划，采用五次多项式规划的方式。
        {
            Frenet_path fp;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (auto t : Cosmos::Range(0.0, Ti, DT))
            {
                fp.t.push_back(t);
            }
            for (auto& t : fp.t)
            {
                fp.d.push_back(lat_qp.calc_point(t));
                fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                fp.d_dd.push_back((lat_qp.calc_second_derivative(t)));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
            }

            // 在速度保持模式下，生成纵向运动规划 Longitudinal motion planning (Velocity keeping)纵向运动规划采用的是四次多项式规划。
            for (auto tv : Cosmos::Range(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S))
            {
                Frenet_path tfp;
                tfp = fp;
                QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

                for (auto& t : fp.t)
                {
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                for (int i = 0; i < tfp.d_ddd.size(); i++) {
                    SquareOfJerks_d_ddd.push_back(pow(tfp.d_ddd[i], 2));
                    SquareOfJerks_s_ddd.push_back(pow(tfp.s_ddd[i], 2));
                }

                Jd = accumulate(SquareOfJerks_d_ddd.begin() , SquareOfJerks_d_ddd.end() , 0.00); //横向加速度突变对路径cost的影响
                Js = accumulate(SquareOfJerks_s_ddd.begin() , SquareOfJerks_s_ddd.end() , 0.00); //纵向加速度突变对路径cost的影响
                ds = (TARGET_SPEED - tfp.s_d.back()) * (TARGET_SPEED - tfp.s_d.back());
                tfp.cd = KJ * Jd + KT * Ti + KD * tfp.d.back() * tfp.d.back();
                tfp.cv = KJ * Js + KT * Ti + KS * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;
                SquareOfJerks_d_ddd.clear();
                SquareOfJerks_s_ddd.clear();
                frenet_paths.push_back(tfp);
            }
        }
    }

    return frenet_paths;
}

void calc_global_paths(vector<Frenet_path>& fplist, Spline2D& csp)
{
    vector<double> ixy;
    double iyaw;
    double fx;
    double fy;
    double dx;
    double dy;
    for (auto& fp : fplist)
    {
        // 计算全局路径点calculate global position
        for (int i = 0; i < fp.s.size(); ++i)
        {
            ixy = csp.calc_position(fp.s[i]);
            if (ixy.empty()) break;
            iyaw = csp.calc_yaw(fp.s[i]);
            fx = ixy[0] + fp.d[i] * cos(iyaw + M_PI_2);
            fy = ixy[1] + fp.d[i] * sin(iyaw + M_PI_2);
            fp.x.push_back(fx);
            fp.y.push_back(fy);
        }

        // 计算偏航角和每步走的距离calculate yaw and ds
        for (int i = 0; i < fp.x.size() - 1; ++i)
        {
            dx = fp.x[i + 1] - fp.x[i];
            dy = fp.y[i + 1] - fp.y[i];
            fp.yaw.push_back(atan2(dy, dx));
            fp.ds.push_back(sqrt(dx * dx + dy * dy));
        }
        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        // 计算轨迹曲率calculate curvature
        for (int i = 0; i < fp.yaw.size() - 1; ++i) {
            fp.c.push_back((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i]);
        }
    }
}

vector<Frenet_path> check_paths(vector<Frenet_path> fplist, const MatrixXd& ob)
{
    vector<int> okind;
    vector<Frenet_path> collision_avoid_paths;
    for (int i = 0; i < fplist.size(); ++i) {
        if (any_of(fplist[i].s_d.begin(), fplist[i].s_d.end(), [](double v){return v > MAX_SPEED;})){
            continue;
        } else if (any_of(fplist[i].s_dd.begin(), fplist[i].s_dd.end(), [](double a){return abs(a) > MAX_ACCEL;})){
            continue;
        } else if (any_of(fplist[i].c.begin(), fplist[i].c.end(), [](double c){return abs(c) > MAX_CURVATURE;})){
            continue;
        } else if(!check_collision(fplist[i], ob)){
            continue;
        }
        okind.push_back(i);
    }
    for (auto i : okind)
    {
        collision_avoid_paths.push_back(fplist[i]);
    }
    return collision_avoid_paths;
}

Frenet_path
frenet_optimal_planning(Spline2D& csp, const double s0, const double c_speed, const double c_d, const double c_d_d,
                        const double c_d_dd, const MatrixXd& ob) {
    auto mincost = DBL_MAX;
    Frenet_path best_path; // 没有初始化，如果出现问题了记得查一下
    vector<Frenet_path> collision_avoid_paths;
    vector<Frenet_path> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fplist, csp);
    collision_avoid_paths = check_paths(fplist, ob);

    for (auto& fp : collision_avoid_paths)
    {
        if (mincost >= fp.cf) // 找到cost最小的路径赋值给best_path
        {
            mincost = fp.cf;
            best_path = fp; // C语言中结构体不能直接赋值，但是C++可以
        }
    }
    return best_path;
}

Spline2D
generate_target_course(vector<double> x, vector<double> y, vector<double> &rx, vector<double> &ry, vector<double> &ryaw,
                       vector<double> &rc) {
    Spline2D csp(x, y);
    vector<double> ixy;
    Cosmos::RangeImpl<int> s = Cosmos::Range(0, int(csp.s.back()), 5);
    for (auto i_s : Cosmos::Range(0, int(csp.s.back()), 5)) {
        ixy = csp.calc_position(i_s);
        rx.push_back(ixy[0]);
        ry.push_back(ixy[1]);
        ryaw.push_back(csp.calc_yaw(i_s));
        rc.push_back(csp.calc_curvature(i_s));
    }
    return csp;
}

int main() {
    vector<double> x = {0.0, -212.5};
    vector<double> y = {0.0, -212.5};
    MatrixXd ob(6, 2);
    ob << -10.0, -12.0,
            -50.0, -50.0,
            -100.0, -100.0,
            -155.0, -154.0,
            -175.0, -175.5,
            -200.0, -205.5;
    vector<double> tx;
    vector<double> ty;
    vector<double> tyaw;
    vector<double> tc;
    Frenet_path path;
    Spline2D csp = generate_target_course(x, y, tx, ty, tyaw, tc);

    double c_speed = 0.01 / 3.6;
    double c_d = 2.0;
    double c_d_d = 0.0;
    double c_d_dd = 0.0;
    double s0 = 0.0;

    while (true) {
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);

        s0 = path.s[1];
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];
//        cout << "x = " << path.x[1] << " y = " << path.y[1] << " speed: " << c_speed << " accelerate:" << path.s_dd[1] << endl;
        cout << "s = " << path.s[1] << " d = " << path.d[1] << endl;
        if (hypot(path.x[1] - tx.back(), path.y[1] - ty.back()) <= 2.0)
        {
            cout << "Goal" << endl;
            break;
        }
    }

    // ///////////////////////////////////调试////////////////////////////////////////// //
//    for (auto &ix : tx) {
//        cout << setw(10) << ix;
//    }
//    cout << endl;
//    for (auto &iy : ty) {
//        cout << setw(10) << iy;
//    }
//    cout << endl;
//    for (auto &iyaw : tyaw) {
//        cout << setw(10) << iyaw;
//    }
//    cout << endl;

    return 0;
}
