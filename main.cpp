#include<iostream>
#include<string>
using namespace std;
#include<Eigen/Dense>
using namespace Eigen;
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/se3.hpp"
namespace pin = pinocchio;

void test01()
{
  pin::Model model;
  pin::buildModels::manipulator(model);
  pin::Data data(model);
 
  VectorXd q = pin::neutral(model);
  VectorXd v = VectorXd::Zero(model.nv);
  VectorXd a = VectorXd::Zero(model.nv);
 
  const VectorXd & tau = pin::rnea(model,data,q,v,a); // const typr& 常引用
  cout << "tau = " << tau.transpose() << endl;
}

void test02()
{
  // urdf path
  const string urdf_filename = string("./urdf/ur5_robot.urdf");
  // load the urdf model
  pin::Model model;
  pin::urdf::buildModel(urdf_filename, model);
  cout << "model name: " << model.name << endl;
  // create data required by the algorithms
  pin::Data data(model);
  // sample a random configuration
  VectorXd q = pin::randomConfiguration(model);
  cout << "q: " << q.transpose() << endl;
  // perform the forward kinematics over the kinematic tree
  pin::forwardKinematics(model, data, q);
  // Print out the placement of each joint of the kinematic tree
  for(pin::JointIndex joint_id = 0; joint_id < (pin::JointIndex)model.njoints; ++joint_id)
  {
    cout << setw(24) << left
        << model.names[joint_id] << ": "
        << fixed << setprecision(2)
        << data.oMi[joint_id].translation().transpose()
        << endl;
  }
}

void test03()
{
  // skew 
  Vector3d p1(0,0,1); 
  Matrix3d M1 = pin::skew(p1);
  cout << "vector: " << p1.transpose() << endl;
  cout << "skew symmetric matrix: " << endl << M1 << endl << endl;

  // so3 -> SO3
  Vector3d w1(1,0,0);
  Matrix3d R1 = pin::exp3(w1);
  cout << "rotation axis: " << w1.transpose() << endl;
  cout << "Rotation matrix: \n" << R1 << endl;
  
  // se3 -> SE3
  Vector3d v(0,0,1);
  Vector3d w(1,0,0); 
  pin::Motion nu = pin::Motion(v,w);
  cout << "Motion expression: " << endl << nu;
  cout << "Linear term: " << nu.linear().transpose() << endl;
  cout << "Angular term: " << nu.angular().transpose() << endl << endl;  
  pin::SE3 T = pin::exp6(nu);
  Matrix3d R = T.rotation();
  Vector3d p = T.translation();
  cout << "SE3 expression: " << endl << T << endl;
  cout << "Homogenous expression: " << endl << T.toHomogeneousMatrix() << endl;
  cout << "Rotation term: " << endl << R << endl;
  cout << "Translation term: " << p.transpose() << endl << endl;
  
  // SO3 -> so3
  Matrix3d R2 = Matrix3d::Random();
  Vector3d w2 = pin::log3(R2);
  cout << "rotation axis: " << w2.transpose() << endl;
  cout << "Rotation matrix: \n" << R2 << endl << endl;
  
  // SE3 -> se3
  Vector3d p2(1,1,1);
  pin::SE3 T2 = pin::SE3(R2, p2);
  pin::Motion nu2 = pin::log6(T2);
  cout << "Linear term: " << nu2.linear().transpose() << endl;
  cout << "Angular term: " << nu2.angular().transpose() << endl << endl; 
  
  // spatial force
  Vector3d f(1,0,0);
  Vector3d tau(1,1,1);
  pin::Force phi = pin::Force(f, tau);
  cout << "Force expression: " << endl << phi;
  cout << "Linear term: " << phi.linear().transpose() << endl;
  cout << "Angular term: " << phi.angular().transpose() << endl << endl;

  // adjoint 
  pin::SE3 T3 = pin::SE3::Random();
  MatrixXd adT = T3.toActionMatrix();
  cout << "SE3 expression: " << endl << T3;
  cout << "Adjoint expression: " << endl << adT << endl;
}


int main()
{
  test03();
  return 0;
}
