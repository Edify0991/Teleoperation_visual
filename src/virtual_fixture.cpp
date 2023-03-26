#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <math.h>
#include<initializer_list>
#include <virtual_fixture.h>
/**
* @brief 阶乘
* @note unsigned long long 只能计算22以内的阶乘
* @note int 只能计算13以内的阶乘
* @param [typename T]
* @return [typename T]
*/
template<typename T>
inline constexpr T factorial(T i){
  return i < 2 ? 1 :  i * factorial(i-1);
}

/**
* @brief Point 构造函数
* @param T p_i 任意数量的参数,参数数量代表点的维数
*/
template<typename T>
Point<T>::Point(std::initializer_list<T> p_i)
{
  dimension = p_i.size();
  position.resize(1,dimension);
  int i=0;
  std::for_each(p_i.begin(), p_i.end(), [&](const T data){
    position[i] = data;
    i++;
  });
  std::cout<< "position" << std::endl << position << std::endl;
}

/**
* @brief 设置Point参数,重载
* @note T p_i 任意数量的参数,参数数量代表点的维数
* @return void
*/
template<typename T>
void Point<T>::setPoint(std::initializer_list<T> p_i)
{
  dimension = p_i.size();
  position.resize(1,dimension);
  int i=0;
  std::for_each(p_i.begin(), p_i.end(), [&](const T data){
    position[i] = data;
    i++;
  });
  std::cout<< "reSet position " << std::endl << position << std::endl;
}

/**
* @brief 设置Point参数
* @note T p_i 任意数量的参数,参数数量代表点的维数
* @return void
*/
template<typename T>
void setPoint(const Point<T>& p, const Eigen::DenseBase<T>& p_i)
{
  static_assert(p_i.rows() == 1,"Point维度设置错误\n");
  p.dimension = p_i.cols();
  p.position.resize(1,p.dimension);
  p.position = p_i;
  std::cout<< "reSet position2 " << std::endl << p.position << std::endl;
}


/**
* @brief Point 构造函数
* @param T p_i 任意数量的参数,参数数量代表点的维数
*/
template<typename T>
Vector<T>::Vector(std::initializer_list<T> p_i)
{
  dimension = p_i.size();
  direction.resize(1,dimension);
  int i=0;
  std::for_each(p_i.begin(), p_i.end(), [&](const T data){
    direction[i] = data;
    i++;
  });
  std::cout<< "direction " << std::endl << direction << std::endl;
}

/**
* @brief 设置参数
* @note T p_i 任意数量的参数,参数数量代表点的维数
* @return void
*/
template<typename T>
void Vector<T>::setVector(std::initializer_list<T> p_i)
{
  dimension = p_i.size();
  direction.resize(1,dimension);
  int i=0;
  std::for_each(p_i.begin(), p_i.end(), [&](const T data){
    direction[i] = data;
    i++;
  });
  std::cout<< "reSet direction " << std::endl << direction << std::endl;
}


/**
* @brief 求两直线交点
* @param Line l1,l2
* @return intersectPoint [x,y], 当两直线平行时候返回[DBL_MAX,DBL_MAX],当两直线重合时候，返回l1,l2中点
*/
template<typename T>
Point<T> Line<T>::lineIntersection_2D(Line<T> l2)
{
  Point<T> intersection;
  double A1,B1,C1,A2,B2,C2;


  A1 = this->direction[1];
  B1 = -this->direction[0];
  C1 = -this->position[0] * this->direction[1] + this->direction[0] * this->position[1];

  A2 = l2.vector.direction[1];
  B2 = -l2.vector.direction[0];
  C2 = -l2.point.position[0] * l2.vector.direction[1] + l2.vector.direction[0] * l2.point.position[1];
  double M = A1 * B2 - A2 * B1;

  if(std::abs(M) < 1e-150){
    intersection.setPoint(DBL_MAX,DBL_MAX);
    if(std::abs(C1 - C2) < 1e-150){
      intersection.position << (this->position + l2.point.position) / 2;
    }
  }else{
    intersection.position[0] = (C2 * B1 - C1 * B2) / M;
    intersection.position[1] = (C1 * A2 - C2 * A1) / M;
  }
  return intersection;
}

/**
* @brief 求平面一点是否在三点围城的三角形内
* @param Line l1,l2,l3
* @return intersectPoint [x,y], 当两直线平行时候返回[DBL_MAX,DBL_MAX],当两直线重合时候，返回l1,l2中点
*/
template<typename T>
bool isInTriangle(Line<T> l1, Line<T> l2)
{
}


///**
//* @brief Bezier曲线类的构造函数
//* @param [Dynamic，2]大小的控制点矩阵，
//* @return void
//*/
//template<typename T>
//Bezier::Bezier(const Eigen::MatrixBase<T>& inputCtrlPoints)
//{
//  if(inputCtrlPoints.cols() != 2 && inputCtrlPoints.rows() < 1){
//    printf("Bezier曲线控制点维度错误\n");
//    return;
//  }
//  degree = inputCtrlPoints.rows();

//  //单凸Bezier曲线
//  uInterval.push_back((Eigen::MatrixXd(1,2) << 0, 1.0).finished());

//  ctrlPoints = inputCtrlPoints;
//  std::vector<unsigned long long> tmpFactorial;
//  for (unsigned long long i=0 ; i<degree; i++){
//    tmpFactorial.push_back(factorial(i));
//  }
//  M.resize(degree,degree);
//  for (unsigned int i=0; i<degree; i++){
//    for (unsigned int j=0; j<degree; j++){
//      if(i<j){
//        M(i,j) = 0;
//      }else {
//        M(i,j) = std::pow(-1,i-j) * tmpFactorial[degree-1]/
//            (tmpFactorial[j]*tmpFactorial[degree-1-i]*tmpFactorial[i-j]);
//      }
//    }
//  }
//}

///**
//* @brief 求Bezier曲线参数u处的坐标
//* @param 参数u 取值范围 [0,1]
//* @return Bezier曲线上的点[x,y]
//*/
//Eigen::Matrix<double, 1, 2> Bezier::point(double u)
//{
//  Eigen::Matrix<double, 1, Eigen::Dynamic> U;
//  U.resize(1,degree);
//  for(unsigned int i=0; i<degree; i++)
//  {
//    U[i] = std::pow(u,i);
//  }
//  return U * M * ctrlPoints;
//}

///**
//* @brief 求Bezier曲线参数u处的导数,矢量方向沿着u增大的方向
//* @param 参数u 取值范围 [0,1]
//* @param 参数n 导数阶数 [0,degree-1]
//* @return 没有归一化导数,[x,y]
//*/
//Eigen::Matrix<double, 1, 2> Bezier::derivative(double u, int n)
//{
//  Eigen::Matrix<double, 1, 2> derivativeDirection;
//  Eigen::Matrix<double, 1, Eigen::Dynamic> uDerivative;

//  if(n > static_cast<int>(degree-1))
//  {
//    printf("导数阶数阶数过大\n");
//  }
//  uDerivative.resize(1, degree);
//  for (int i = 0; i < static_cast<int>(degree); i++){
//    if(i<n){
//      uDerivative[i] = 0;
//    }else{
//      uDerivative[i] = std::pow(u,i-n) * factorial(i)/factorial(i-n);
//    }
//  }
//  //  std::cout  <<  uDerivative << std::endl;
//  //  std::cout << uDerivative << std::endl;
//  //  derivativeDirection << uDerivative * M * ctrlPoints;
//  //  return derivativeDirection/derivativeDirection.norm();
//  return uDerivative * M * ctrlPoints;
//}

///**
//* @brief 求Bezier曲线参数u处的法向
//* @param 参数u取值范围 [0,1]
//* @return 归一化的法线方法，规定法线方向从凹侧向凸侧,[x,y]
//*/

//Eigen::Matrix<double, 1, 2> Bezier::normal(double u)
//{
//  Eigen::Matrix<double, 1, 2> tangent;
//  Eigen::Matrix<double, 1, 2> normal;
//  tangent = derivative(u, 1);
//  normal << tangent[1],-tangent[0];

//  if(u == 0.0)
//  {
//    if((point(1.0) - point(u)).dot(normal) > 0.0)
//    {
//      normal = -normal;
//    }
//  }
//  if((point(0.0) - point(u)).dot(normal) > 0.0)
//  {
//    normal = -normal;
//  }
//  return normal / normal.norm();
//}


///**
//* @brief 得到Bezier曲线在参数u处的曲率
//* @param 参数u 取值范围 [0,1]
//* @return 曲率
//*/
//double Bezier::curvature(double u)
//{
//  Eigen::Matrix<double, 1, 2> derivative1,derivative2;
//  derivative1 = Bezier::derivative(u, 1);
//  derivative2 = Bezier::derivative(u, 2);
//  return std::abs((derivative2[1] * derivative1[0] - derivative1[1] * derivative2[0])
//      / (std::pow(derivative1.squaredNorm(),1.5)));
//}

///**
//* @brief 得到Bezier曲线在参数u处的曲率半径,和曲率圆的圆心坐标
//* @param 参数u 取值范围 [0,1]
//* @return 曲率半径,
//*/
//Eigen::Matrix<double, 1, 2> Bezier::circleCurvature(double u)
//{
//  double r ;
//  Eigen::Matrix<double, 1, 2> pointBezier;
//  Eigen::Matrix<double, 1, 2> normalBezier;
//  r = 1 / curvature(u);
//  pointBezier = point(u);
//  normalBezier = normal(u);
//  return pointBezier - normalBezier * r;
//}

///**
//* @brief 得到在Bezier曲线上距离输入坐标的最近点
//* @param 二维空间任意一点[x,y]
//* @param 迭代初始参数u, [0,1], 默认值为0
//* @return Bezier曲线上的点参数 NearestPointArgs
//*/
//double Bezier::pointNearest(Eigen::Matrix<double, 1, 2> inputPoint, double u, double maxDeltaU)
//{
//  Eigen::Matrix<double, 1, 2> position,tangent,linePP;
//  double deltaU;
//  int i = 0;
//  while(i<=1000)
//  {
//    position = point(u);
//    linePP = inputPoint - position;
//    tangent = derivative(u, 1);
//    deltaU = tangent.dot(linePP) / tangent.squaredNorm();
//    if(std::abs(deltaU) < maxDeltaU) break;
//    if((u == 1.0 && deltaU > 0.0) || (u == 0.0 && deltaU < 0.0)) break;
//    u += deltaU;
//    if(u > 1.0){
//      u = 1.0;
//    }
//    if(u < 0.0){
//      u = 0.0;
//    }
//    i++;
//  }
//  if(i>1000){
//    printf("垂直逼近求最短距离算法不收敛\n");
//    ros::shutdown();
//  }
//  return u;
//}

///**
//* @brief 得到一点到bezier曲线的切线
//* @param 二维空间任意一点[x,y]
//* @param 迭代初始参数u, [0,1], 默认值为0
//* @return Line 切线参数
//*/
//Line<double> Bezier::pointTangent(Eigen::Matrix<double, 1, 2> inputPoint, double u, double maxDeltaU)
//{
//  Line<double> tangent;
//  return tangent;
//}

///**
//* @brief 得到在Bezier曲线和直线的交点,
//* @param l1 二维空间任意一点[x,y] ,二维空间任意一个矢量，无限距离[x,y]
//* @param uInit 迭代初始参数u, [0,1], 默认值为0
//* @param maxDeltaU
//* @return PointArgs 若没有交线则返回参数为最近点参数，若有交线，则返回参数为交点参数
//*/
//double Bezier::bezierLineIntersection(Line<double> l1, double u, double maxDeltaU)
//{
//  Eigen::Matrix<double, 1, 2> P3;
//  double deltaU;
//  Line<double> l2,l3;
//  int i = 0;
//  double theta1, theta3;

//  while(i<100)
//  {
//    l2.position = point(u);
//    l2.direction = derivative(u,1);

//    l3.position = l1.position;
//    l3.direction = l1.position - l2.position;
//    P3 = lineLineIntersection(l1,l2);

//    if((P3 - l1.position).dot(l1.direction) < 0)
//    {
//      l1.direction = -l1.direction;
//    }

//    theta1 = std::acos(l3.direction.dot(l2.direction) / (l3.direction.norm() * l2.direction.norm()));
//    theta3 = std::acos(l1.direction.dot(l2.direction) / (l1.direction.norm() * l2.direction.norm()));

//    if((P3 - l2.position).dot(l2.direction) < 0)
//    {
//      theta1 = M_PI - theta1;
//      theta3 = M_PI - theta3;
//      deltaU = -(l3.direction.norm() * cos(theta1) + l3.direction.norm() * sin(theta1) / tan(theta3)) / l2.direction.norm();
//    }else{
//      deltaU = (l3.direction.norm() * cos(theta1) + l3.direction.norm() * sin(theta1) / tan(theta3)) / l2.direction.norm();
//    }

//    std::cout << "**********************" << std::endl;
//    //    std::cout << "l2.position " << l2.position <<std::endl;
//    //    std::cout << "l2.direction " << l2.direction <<std::endl;
//    //    std::cout << "P3 " << P3 <<std::endl;
//    //    std::cout << "theta1 " << theta1 <<std::endl;
//    //    std::cout << "theta3 " << theta3 <<std::endl;
//    std::cout << "u " << u <<std::endl;
//    std::cout << "deltaU " << deltaU <<std::endl;

//    if(std::abs(deltaU) < maxDeltaU) break;
//    if((u == 1.0 && deltaU > 0.0) || (u == 0.0 && deltaU < 0.0)) break;
//    u += deltaU;
//    if(u > 1.0){
//      u = 1.0;
//    }
//    if(u < 0.0){
//      u = 0.0;
//    }
//    i++;
//  }
//  std::cout << "*****i******* " << i <<std::endl;
//  if(i>100-1){
//    printf("垂直逼近求最短距离算法不收敛\n");
//    ros::shutdown();
//  }

//  return u;
//}


