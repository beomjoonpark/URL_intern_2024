
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



class P_cost {
    public:

    P_cost(Pose z, Pose m, Eigen::Matrix<double, 6, 6> covar)
        : z_(std::move(z)), m_(std::move(m)), covar_(std::move(covar)) {
            Eigen::Matrix<double, 6, 6> info = covar.inverse();
            Eigen::LLT<Eigen::MatrixXd> llt(info);
            wh_ = llt.matrixL().transpose();
        }

    template <typename T>
    bool operator()(const T* const p_b_ptr,
                  const T* const q_b_ptr,
                  T* residuals_ptr) const {
        
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        Pose zinv = z_.inv();

        // Pose minv = m_.inv();
        // Eigen::Quaternion<T> q_r = (minv.q.template cast<T>()) * q_b;
        // Eigen::Matrix<T, 3, 1> p_r = minv.p.template cast<T>() + (minv.q.template cast<T>()) * p_b;

        // Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        // residuals.template block<3, 1>(0, 0) = p_r;
        // residuals.template block<3, 1>(3, 0) = q_r.vec();
        Eigen::Quaternion<T> estimated_q = q_b * (zinv.q.template cast<T>());
        Eigen::Matrix<T, 3, 1> estimated_p = p_b + estimated_q *(zinv.p.template cast<T>());
        
        Eigen::Matrix<T, 3, 1> expected_p = m_.p.template cast<T>();
        Eigen::Quaternion<T> expected_q = m_.q.template cast<T>();

        Eigen::Quaternion<T> delta_q =
            expected_q.conjugate() * estimated_q;


        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = (estimated_p - expected_p);

        residuals.template block<3, 1>(3, 0) = delta_q.vec();

        residuals.applyOnTheLeft(wh_.template cast<T>());
    

        return true;
    }

    static ceres::CostFunction* Create(const Pose& z, const Pose& m, 
                    const Eigen::Matrix<double, 6, 6>& covar) {
        return new ceres::AutoDiffCostFunction<P_cost, 6, 3, 4>(new P_cost(z, m, covar));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    private:
    
    const Pose z_;
    const Pose m_;
    const Eigen::Matrix<double, 6, 6> covar_;
    Eigen::Matrix<double, 6, 6> wh_;
};

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

class O_cost {
    public:
    O_cost(Pose z, Eigen::Matrix<double, 6, 6> covar)
        : z_(std::move(z)), covar_(std::move(covar)) {
            Eigen::Matrix<double, 6, 6> info = covar.inverse();
            Eigen::LLT<Eigen::MatrixXd> llt(info);
            wh_ = llt.matrixL().transpose();
        }

    template <typename T>
    bool operator()(const T* const p_a_ptr,
                  const T* const q_a_ptr,
                  const T* const p_b_ptr,
                  const T* const q_b_ptr,
                  T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        Eigen::Quaternion<T> delta_q =
            z_.q.conjugate().template cast<T>() * q_ab_estimated;
        
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = (p_ab_estimated - z_.p.template cast<T>());
        // residuals.template block<3, 1>(3, 0) = delta_q.toRotationMatrix().eulerAngles(0,1,2)*T(180/acos(-1))/T(3);
        residuals.template block<3, 1>(3, 0) = delta_q.vec();
        
        residuals.applyOnTheLeft(wh_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Pose& z, 
                    const Eigen::Matrix<double, 6, 6>& covar) {
        return new ceres::AutoDiffCostFunction<O_cost, 6, 3, 4, 3, 4>(new O_cost(z, covar));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    const Pose z_;
    const Eigen::Matrix<double, 6, 6> covar_;
    Eigen::Matrix<double, 6, 6> wh_;
};

#endif