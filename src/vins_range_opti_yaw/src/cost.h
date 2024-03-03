
#ifndef COST_H
#define COST_H

#include "structure.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/autodiff_cost_function.h"
// #include "ceres_header.h"

#include "Eigen/Dense"
#include <math.h>

using namespace std;


// Cost for Prior Residual. 
// Note that the original paper transforms the marginalized pose 
// to compare with the first pose in the sliding window,
// while the implementation transforms the first pose instead.  
// 
class P_cost {
    public:

    P_cost(Pose z, Pose m, Eigen::Matrix<double, 4, 4> covar)
        : z_(std::move(z)), m_(std::move(m)), covar_(std::move(covar)) {
            Eigen::Matrix<double, 4, 4> info = covar.inverse();
            Eigen::LLT<Eigen::MatrixXd> llt(info);
            wh_ = llt.matrixL().transpose();
        }

    template <typename T>
    bool operator()(const T* const p_b_ptr,
                  const T* const yaw_b,
                  T* residuals_ptr) const {
        
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);


        // m-1 * a = z
        // m*z = a
        // m = a * z-1
        
        Pose zinv = z_.inv();

        Eigen::Matrix<T, 3, 1> estimated_p(cos(*yaw_b)*zinv.p.x()-sin(*yaw_b)*zinv.p.y()+p_b.x(), 
                                        sin(*yaw_b)*zinv.p.x()+cos(*yaw_b)*zinv.p.y()+p_b.y(), 
                                        zinv.p.z()+p_b.z());
            //b*zinv;
        T estimated_yaw = zinv.yaw + *yaw_b;

        
        Eigen::Matrix<T, 3, 1> expected_p = m_.p.template cast<T>();
        T expected_yaw = T(m_.yaw);



        Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals(residuals_ptr);


        residuals.template block<3, 1>(0, 0) = (estimated_p - expected_p);
        residuals(3) =  (estimated_yaw-expected_yaw) -
         T(2*acos(-1)) * ceres::floor(((estimated_yaw-expected_yaw) + T(acos(-1))) / T(2*acos(-1)));


        residuals.applyOnTheLeft(wh_.template cast<T>());
    

        return true;
    }

    static ceres::CostFunction* Create(const Pose& z, const Pose& m, 
                    const Eigen::Matrix<double, 4, 4>& covar) {
        return new ceres::AutoDiffCostFunction<P_cost, 4, 3, 1>(new P_cost(z, m, covar));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
    
    const Pose z_;
    const Pose m_;
    const Eigen::Matrix<double, 4, 4> covar_;
    Eigen::Matrix<double, 4, 4> wh_;
};


// Cost for Range. 
class R_cost {
    public:
    R_cost(float r, float stdvar)
        : r_(std::move(r)), stdvar_(std::move(stdvar)) {}

    template <typename T>
    bool operator()(const T* const a,
                  const T* const b,
                  T* residuals_ptr) const {

        T norm = sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2));
        
        residuals_ptr[0] = (norm - T(r_)) / T(stdvar_);


        return true;
    }

    static ceres::CostFunction* Create(const float r, const float stdvar){
        return new ceres::AutoDiffCostFunction<R_cost, 1, 3, 3>(new R_cost(r, stdvar));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    const float r_;
    const float stdvar_;
};


// Cost for odometry. 
class O_cost {
    public:
    O_cost(Pose z, Eigen::Matrix<double, 4, 4> covar)
        : z_(std::move(z)), covar_(std::move(covar)) {
            Eigen::Matrix<double, 4, 4> info = covar.inverse();
            Eigen::LLT<Eigen::MatrixXd> llt(info);
            wh_ = llt.matrixL().transpose();
        }

    template <typename T>
    bool operator()(const T* const p_a_ptr,
                  const T* const yaw_a,
                  const T* const p_b_ptr,
                  const T* const yaw_b,
                  T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);

        Pose zinv = z_.inv();
        
        // we need a-1 b
        Eigen::Matrix<T, 3, 1> ainv_p(-p_a.x()*cos(*yaw_a)-p_a.y()*sin(*yaw_a), 
                                    p_a.x()*sin(*yaw_a)-p_a.y()*cos(*yaw_a),
                                    -p_a.z());
        T ainv_yaw = -(*yaw_a);

        Eigen::Matrix<T, 3, 1> estimated_p(ainv_p.x() + p_b.x()*cos(ainv_yaw)-p_b.y()*sin(ainv_yaw), 
                                        ainv_p.y() + p_b.x()*sin(ainv_yaw)+p_b.y()*cos(ainv_yaw), 
                                        ainv_p.z() + p_b.z());
        T estimated_yaw = *yaw_b + ainv_yaw;


        Eigen::Matrix<T, 3, 1> expected_p = z_.p.template cast<T>();
        T expected_yaw = T(z_.yaw);        

        Eigen::Map<Eigen::Matrix<T, 4, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = (estimated_p - expected_p);
        residuals(3) = (estimated_yaw-expected_yaw) -
         T(2*acos(-1)) * ceres::floor(((estimated_yaw-expected_yaw) + T(acos(-1))) / T(2*acos(-1)));
        
        residuals.applyOnTheLeft(wh_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Pose& z, 
                    const Eigen::Matrix<double, 4, 4>& covar) {
        return new ceres::AutoDiffCostFunction<O_cost, 4, 3, 1, 3, 1>(new O_cost(z, covar));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    const Pose z_;
    const Eigen::Matrix<double, 4, 4> covar_;
    Eigen::Matrix<double, 4, 4> wh_;
};

#endif