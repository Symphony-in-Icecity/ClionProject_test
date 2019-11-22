#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;
int main()
{
//    Matrix3d m = Matrix3d::Ones();
//    cout << "Matrix m" << endl;
//    cout << m << endl;
//    cout << m(0, 2) << endl;
//
//    m = (m + Matrix3d::Constant(1.2)) * 5;
//    cout << "m =" << endl << m << endl;
//    Vector3d v(1, 2, 3);
//    cout << "v = " << endl << v << endl;
//    cout << "m * v =" << endl << m * v << endl;
////    Vector3d x = m.qr().solve(v);
//    Vector3d x = m.lu().solve(v);
//    cout << "x = " << endl << x << endl;
////    Vector3d y = m.transpose() * v;
////    cout << "y = " << endl << y << endl;
//
//    VectorXd y = VectorXd::LinSpaced(10,1,9);
//    cout << "y = " << endl << y << endl;
//
//    MatrixXd A_ = MatrixXd::Zero(2, 3);
//
//    MatrixXd ob(6, 2);
//    ob << 20.0, 0.0,
//            70.0, 2.0,
//            100.0, -3.0,
//            150.0, 0.0,
//            210.0, 0.5,
//            250.0, -1.5;
//    cout << ob << endl;
//    Matrix3d xiashishi;
//    Vector3d aaa;
//    aaa <<  1,
//            2,
//            3;
//    cout << aaa << endl;
//    xiashishi << 1, 2, 3,
//            4, 5, 6,
//            7, 8, 9;
//    std::cout << m << endl;

    Vector3d x1(1.2, 0, 0);
    Vector3d x2(-1, 0, 0);
    Vector3d x3 = x1.cross(x2);
    cout << x3 << endl;
    

//    Vector3d x0(0, 0.1, 0);
//    Vector3d x1(1.2, 0.2, 0);
//    Vector3d x2(-0.1, 1.1, 0);
//    Vector3d x3 = (x1 - x0).cross(x2 - x0);
//
//    cout << x3(2) << endl;

    return 0;
}