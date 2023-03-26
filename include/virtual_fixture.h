#ifndef VIRTUAL_FIXTURE
#define VIRTUAL_FIXTURE
#include <deque>
#include <vector>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>

template<typename T>
class Point{
public:
Eigen::Matrix<T, 1, Eigen::Dynamic> position;
unsigned int dimension;

Point(std::initializer_list<T> p_i);
void setPoint(std::initializer_list<T> p_i);

template<typename M>
void setPoint(const Eigen::DenseBase<M>& p_i)
{
//    static_assert(p_i.rows() == 1,"Point维度设置错误\n");
    dimension = p_i.cols();
    position.resize(1,dimension);
    position = p_i;
    std::cout<< "reSet position " << std::endl << position << std::endl;
}
};
template<typename T>
void setPoint(const Point<T>& p, const Eigen::DenseBase<T>& p_i);


template<typename T>
class Vector{
public:
Eigen::Matrix<T, 1, Eigen::Dynamic> direction;
unsigned int dimension;

Vector(std::initializer_list<T> p_i);
void setVector(std::initializer_list<T> p_i);
};

template<typename T>
class Line: public Point<T>,public Vector<T>{
public:

Point<T> lineIntersection_2D(Line<T> l2);
};

template<typename T>
class Bezier
{
private:
  unsigned int degree;
  Eigen::Matrix<double, Eigen::Dynamic, 2> ctrlPoints;
  std::vector<Eigen::Matrix<double, 1, 2>> uInterval;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M;

public:

  Bezier(const Eigen::MatrixBase<T>& inputCtrlPoints);

  Eigen::Matrix<double, 1, 2> point(double u);
  Eigen::Matrix<double, 1, 2> derivative(double u, int n);
  Eigen::Matrix<double, 1, 2> normal(double u);
  double curvature(double u);
  Eigen::Matrix<double, 1, 2> circleCurvature(double u);
  double pointNearest(Eigen::Matrix<double, 1, 2> point, double uInit=0, double maxDeltaU = 1e-6);
  Line<double> pointTangent(Eigen::Matrix<double, 1, 2> inputPoint, double uInit=0, double maxDeltaU = 1e-6);
  double bezierLineIntersection(Line<double> l1, double uInit=0, double maxDeltaU = 1e-6);
};



#endif // VIRTUAL_FIXTURE
