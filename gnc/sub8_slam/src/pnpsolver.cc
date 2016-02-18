#include <sub8_slam/slam.h>
#include <unsupported/Eigen/NumericalDiff>
namespace slam {

//Copied from https://github.com/libigl/eigen/blob/master/unsupported/test/NumericalDiff.cpp
// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  int m_inputs, m_values;
  const slam::PointVector points2d;
  const slam::Point3Vector points3d;
  const cv::Mat K;

  Functor(int inputs, int values, const slam::PointVector points2, const slam::Point3Vector points3, const cv::Mat _K) : m_inputs(inputs), m_values(values), points2d(points2),
  points3d(points3), K(_K){}


  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

  int operator()(const VectorXf &x, VectorXf &fvec) const{
        cv::Mat points2d_measured(points2d);

        VectorXf rot(3);
        rot << x[3], x[4], x[5];
        VectorXf trans(3);
        trans << x[0], x[1], x[2];

        cv::Mat mytrans;
        cv::Mat myrot;

        cv::eigen2cv(trans, mytrans);
        cv::eigen2cv(rot, myrot);

        cv::Mat points2d_est;
        cv::Mat error;

        cv::projectPoints(points3d, myrot, mytrans, K, cv::Mat(), points2d_est);
        error = points2d_measured - points2d_est;

        for (int k = 0; k < error.rows; k++) {
            cv::Mat row = error.row(k);
            fvec(k) = cv::norm(row);
        }

        return 0;

  }

};



    slam::Pose PNPSolver::solvePNP(slam::PointVector points2d, Point3Vector points3d, Pose pose_guess, cv::Mat K, int method){
        int max_iter = 10;
        float max_error = 1;

        CvPose cv_pose(pose_guess);
        cv::Mat rotation_vector;
        cv::Rodrigues(cv_pose.rotation, rotation_vector);

        int num_points = points2d.size();

        MatrixXf jac(num_points, 6);
        Functor<float> my_functor = Functor<float>(6, num_points, points2d, points3d, K);
        NumericalDiff<Functor<float>> numDiff(my_functor);

        VectorXf rotVec(3);
        VectorXf tranVec(3);
        VectorXf motion_vec(6);

        cv2eigen(cv_pose.translation, tranVec);
        cv2eigen(rotation_vector, rotVec);

        motion_vec << tranVec, rotVec;

        for(int i = 0; i != max_iter; ++i){
            std::cout<<"Hey"<<std::endl;
            numDiff.df(motion_vec, jac);
            VectorXf rvec(num_points);
            my_functor.operator()(motion_vec, rvec);
            VectorXf b = jac * motion_vec - rvec;
            VectorXf gradient = (2 * jac.transpose()) * rvec;
            motion_vec = jac.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
            if(gradient.norm() < max_error || i == max_iter - 1){
                if(gradient.norm() < max_error){
                  std::cout << "grd" << gradient.norm() <<std::endl;
                }
                Eigen::Vector3f translation;
                VectorXf rot1(3);
                rot1 << motion_vec[3], motion_vec[4], motion_vec[5];
                translation << motion_vec[0], motion_vec[1], motion_vec[2];

                cv::Mat rot;
                eigen2cv(rot1, rot);
                //eigen2cv(trans1, trans);

                cv::Mat rotmat;
                cv::Rodrigues(rot, rotmat);

                Eigen::Matrix3f rotation;
                //Eigen::Vector3f translation;

                cv::cv2eigen(rotmat, rotation);
               // cv::cv2eigen(trans, translation);
                Eigen::Matrix4f final_pose;

                final_pose << rotation, rotation.transpose().eval() * translation, 0.0, 0.0,
      0.0, 1.0;
                Eigen::Affine3f true_final_pose;
                true_final_pose = final_pose;
                return true_final_pose;
            }

        }
        std::cout<<"-------e"<<std::endl;
    };
}
