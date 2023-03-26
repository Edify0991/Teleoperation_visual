#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <Eigen/Dense>
#include <vector>
#include<initializer_list>
#include <virtual_fixture.h>
#include <config.h>

#include "virtual_fixture.cpp"


template<typename T,typename M>
void setPoint(const Point<M>& p, const Eigen::DenseBase<T>& p_i)
{
//  static_assert(p_i.rows() == 1,"Point维度设置错误\n");
//  p.dimension = p_i.cols();
//  p.position.resize(1,p.dimension);
//  p.position = p_i;
//  std::cout<< "reSet position2 " << std::endl << p.position << std::endl;
}
template <typename Derived>
void print_block(const Point<Derived>& p, const Eigen::DenseBase<Derived>& b, int x, int y, int r, int c)
{
  std::cout << "block: " << b.block(x,y,r,c) << std::endl;
}

int main(void)
{

//  Point<double> p1({1,2,3,4});

////  p1.setPoint({3,2,1});

//Eigen::Matrix<double,1,2> p2;
//p2 << 6,7;
//p1.setPoint(p2);

//    Eigen::Matrix<double,4,2> ctrlPoints;
  //  ctrlPoints << 0,0,1,6,7,10,10,10;
  //  Bezier bezier(ctrlPoints);
  //  Eigen::Matrix<double,1,2> point;
  //  point << 3,3;
  //  bezier.pointNearest(point,0.5,1e-10);

  //  Line<double> l1,l2,l3,l4;

  //  l1.point.position << 3,8;
  //  l1.vector.direction << 2,3;

  //  l2.point.position << 2.0, 5.0;
  //  l2.vector.direction << 1.0, 1.8;

  //  l3.point.position <<0,0;
  //  l3.vector.direction<<1,1;

  //  l4.point.position <<1,1;
  //  l4.vector.direction<<1,1.000000000000001;
  //  std::cout <<"*********** " << std::endl << l3.lineIntersection_2D(l4).position << std::endl;

  //    std::cout <<"*********** " << std::endl << 1/(l4.direction[1]) << std::endl;
  //    std::cout <<"*********** " << std::endl << 10/(l4.direction[1]) << std::endl;
  //    std::cout <<"*********** " << std::endl << 100/(l4.direction[1]) << std::endl;
  //    std::cout <<"*********** " << std::endl << 1000/(l4.direction[1]) << std::endl;
  //    std::cout <<"*********** " << std::endl << 10000/(l4.direction[1]) << std::endl;

  //    std::cout <<"circleCurvature(0) " << std::endl << bezier.circleCurvature(0) << std::endl;
  //    std::cout <<"circleCurvature(0.001) " << std::endl << bezier.circleCurvature(0.001) << std::endl;
  //    std::cout <<"circleCurvature(0.002) " << std::endl << bezier.circleCurvature(0.002) << std::endl;
  //    std::cout <<"circleCurvature(0.003) " << std::endl << bezier.circleCurvature(0.003) << std::endl;
  //    std::cout <<"circleCurvature(0.25) " << std::endl << bezier.circleCurvature(0.25) << std::endl;
  //    std::cout <<"circleCurvature(0.5) " << std::endl << bezier.circleCurvature(0.5) << std::endl;
  //    std::cout <<"circleCurvature(0.75) " << std::endl << bezier.circleCurvature(0.75) << std::endl;
  //    std::cout <<"circleCurvature(1) " << std::endl << bezier.circleCurvature(1) << std::endl;


  //  LineArgs l2;
  //  l2.position << 4.25,7.25;
  //  l2.direction << 12,10.5;
  //  std::cout <<"point " <<intersectPointLines(l1, l2) << std::endl;

  //  PointArgs lineIntersection =  bezier.lineIntersection(l1,0,1e-10);
  //  std::cout <<"lineIntersection.u " << lineIntersection.u << std::endl;
  //  std::cout <<"lineIntersection.position " << lineIntersection.position << std::endl;
  //  std::cout <<"lineIntersection.pointNormal " << lineIntersection.pointNormal << std::endl;
  //  std::cout <<"lineIntersection.pointderivative " << lineIntersection.pointTangent << std::endl;

}

