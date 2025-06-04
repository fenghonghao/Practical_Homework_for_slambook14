/**************************************************QUESTION***********************************************/
//有两个右手系1和2,其中2系的x轴与1系的y轴方向相同，2系的y轴与1系z轴方向相反，2系的z轴与1系的x轴相反,两个坐标系原点重合
//求R12，求1系中(1,1,1)在2系中的坐标。请自己编写一个c++程序实现它，并用Cmake编译，得到能输出答案的可执行文件
//@ author: 全日制学生混(bilibili)
//@ homePage: https://space.bilibili.com/336103007
//@ github: https://github.com/cckaixin/Practical_Homework_for_slambook14
/*********************************************************************************************************/
#include<iostream>
#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#define out(x) cout<<#x<<"=\n"<<x<<"\n";
using namespace std;
using namespace Eigen;

// 内旋欧拉角,按照rpy顺序转为旋转矩阵

Matrix3d EularAngleToRotationMatrix_active_1(double roll, double pitch, double yaw)
{
    AngleAxisd rv_roll(roll, Vector3d::UnitX());// Active Axis
    AngleAxisd rv_pitch(pitch, rv_roll * Vector3d::UnitY());
    AngleAxisd rv_yaw(yaw, rv_pitch * rv_roll * Vector3d::UnitZ());
    // out(rv_roll.toRotationMatrix())
    // out(rv_pitch.toRotationMatrix())
    // out(rv_yaw.toRotationMatrix())
    Matrix3d R = (rv_yaw * rv_pitch * rv_roll).toRotationMatrix();
    return R;
}

Matrix3d EularAngleToRotationMatrix_active_2(double roll, double pitch, double yaw)
{
    AngleAxisd rv_roll(roll, Vector3d::UnitX());// Active Axis
    AngleAxisd rv_pitch(pitch, Vector3d::UnitY());
    AngleAxisd rv_yaw(yaw, Vector3d::UnitZ());
    // out(rv_roll.toRotationMatrix())
    // out(rv_pitch.toRotationMatrix())
    // out(rv_yaw.toRotationMatrix())
    Matrix3d R = (rv_roll * rv_pitch * rv_yaw).toRotationMatrix();
    /*这里乘法的顺序为什么是这样呢？因为左乘对应的是同一坐标系下的，如果要想上面的顺序，就得把计算对应旋转角度参考的那个旋转向量给计算出来（21,22行后面的参数）；
     *如果不想那样算，动轴和定轴旋转顺序是反过来的，参见：https://www.zhihu.com/question/407150749/answer/2354982075，左乘的顺序跟上面刚好反过来*/
    return R;
}

Matrix3d EularAngleToRotationMatrix_active_3(double roll , double pitch , double yaw)
{
	Matrix3d R, R_roll, R_pitch, R_yaw;
	R_roll << 1.0, 0.0, 0.0,
		0.0, cos(roll), -sin(roll),
		0.0, sin(roll), cos(roll);
	R_pitch << cos(pitch), 0.0, sin(pitch),
		0.0, 1.0, 0.0,
		-sin(pitch), 0.0, cos(pitch);
	R_yaw << cos(yaw), -sin(yaw), 0.0,
		sin(yaw), cos(yaw), 0.0,
		0.0, 0.0, 1.0;
    // 跟上面的角轴是一样的，可以输出看看
    // out(R_roll)
    // out(R_pitch)
    // out(R_yaw)
	R = R_roll * R_pitch * R_yaw;
	return R;
}

// 外旋欧拉角,按照rpy顺序转为旋转矩阵
// 可以是上面三种改一下左乘顺序改写过来

Matrix3d EularAngleToRotationMatrix_fixed_1(double roll , double pitch , double yaw)
{
    AngleAxisd rv_roll(roll, Vector3d::UnitX()); // FIXED axis
    AngleAxisd rv_pitch(pitch, Vector3d::UnitY());
    AngleAxisd rv_yaw(yaw, Vector3d::UnitZ());
    // out(rv_roll.toRotationMatrix())
    // out(rv_pitch.toRotationMatrix())
    // out(rv_yaw.toRotationMatrix())
    Matrix3d R = (rv_yaw * rv_pitch * rv_roll).toRotationMatrix();
    return R;
}

int main(int argc, char* argv[])
{
//hint 1: Find the Euler Angle from Frame1 to Frame2
    double roll = -M_PI_2;
    double pitch = -M_PI_2;
    double yaw = 0;
    Matrix3d R12_1 = EularAngleToRotationMatrix_active_1(roll, pitch, yaw);
    cout << "Rotation Matrix R12_1 from Frame1 to Frame2 is: " << endl;
    cout << R12_1 << endl;

    Matrix3d R12_2 = EularAngleToRotationMatrix_active_2(roll, pitch, yaw);
    cout << "Rotation Matrix R12_2 from Frame1 to Frame2 is: " << endl;
    cout << R12_2 << endl;
    
    Matrix3d R12_3 = EularAngleToRotationMatrix_active_3(roll, pitch, yaw);
    cout << "Rotation Matrix R12_3 from Frame1 to Frame2 is: " << endl;
    cout << R12_3 << endl;

    Matrix3d R12_4 = EularAngleToRotationMatrix_fixed_1(-M_PI_2, 0, M_PI_2);
    cout << "Rotation Matrix R12_4 from Frame1 to Frame2 is: " << endl;
    cout << R12_4 << endl;

    //四个rotationmatrix都是一样的

//hint 2: Convert the Euler Angle into the rotation matrix

//hint 3: Try to convert a known point under the Frame1, like (1,1,1), to Frame2 using the rotation matrix, 
//        and then you can check if your answer is correct intuitively. (The answer is (1, -1, -1))
    Eigen::Vector3d point_frame1(1,1,1);
    Eigen::Vector3d point_frame2;
    point_frame2 = R12_1.transpose() * point_frame1;  //p2 = R21 * p1

    std::cout << "***************************************" << std::endl;
    std::cout << "point in frame1: " << point_frame1.transpose() << std::endl;
    std::cout << "point in frame2: " << point_frame2.transpose() << std::endl;
    std::cout << "R12: " << std::endl;
    std::cout << R12_1<< std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}