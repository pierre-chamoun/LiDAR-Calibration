#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Core>

class ScanToScanFactor : public gtsam::NoiseModelFactorN<gtsam::Vector6> {
    gtsam::Point3 src_pt_;
    gtsam::Point3 tgt_pt_;
    double T_;

   public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector6>;

    ScanToScanFactor(gtsam::Key key_twist, const gtsam::Point3& src_pt, const gtsam::Point3& tgt_pt,
                     double T, const gtsam::SharedNoiseModel& noise)
        : Base(noise, key_twist), src_pt_(src_pt), tgt_pt_(tgt_pt), T_(T) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector6& twist,
        boost::optional<gtsam::Matrix&> H_twist = boost::none) const override {
        gtsam::Matrix66 H_exp;
        const gtsam::Pose3 transformation =
            gtsam::Pose3::Expmap(twist * T_, H_twist ? gtsam::OptionalJacobian<6, 6>(H_exp)
                                                     : gtsam::OptionalJacobian<6, 6>());

        gtsam::Matrix36 H_trans;
        const gtsam::Point3 transformed_src_pt = transformation.transformFrom(
            src_pt_,
            H_twist ? gtsam::OptionalJacobian<3, 6>(H_trans) : gtsam::OptionalJacobian<3, 6>(),
            gtsam::OptionalJacobian<3, 3>());

        if (H_twist) *H_twist = H_trans * H_exp * T_;

        return transformed_src_pt - tgt_pt_;
    }
};

class CrossLidarFactor : public gtsam::NoiseModelFactorN<gtsam::Vector6, gtsam::Pose3> {
    gtsam::Point3 src_pt_;
    gtsam::Point3 tgt_pt_;
    double dt_;

   public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector6, gtsam::Pose3>;

    CrossLidarFactor(gtsam::Key key_twist, gtsam::Key key_extrinsic, const gtsam::Point3& src_pt,
                     const gtsam::Point3& tgt_pt, double dt, const gtsam::SharedNoiseModel& noise)
        : Base(noise, key_twist, key_extrinsic), src_pt_(src_pt), tgt_pt_(tgt_pt), dt_(dt) {}

    gtsam::Vector evaluateError(const gtsam::Vector6& twist, const gtsam::Pose3& T_ref_other,
                                boost::optional<gtsam::Matrix&> H_twist = boost::none,
                                boost::optional<gtsam::Matrix&> H_T = boost::none) const override {
        const double t = dt_;

        gtsam::Matrix36 H_ext_pose;
        gtsam::Matrix33 H_ext_pt;
        const gtsam::Point3 p_in_ref = T_ref_other.transformFrom(
            src_pt_,
            H_T ? gtsam::OptionalJacobian<3, 6>(H_ext_pose) : gtsam::OptionalJacobian<3, 6>(),
            H_twist ? gtsam::OptionalJacobian<3, 3>(H_ext_pt) : gtsam::OptionalJacobian<3, 3>());

        gtsam::Matrix66 H_exp;
        const gtsam::Pose3 transformation =
            gtsam::Pose3::Expmap(twist * t, H_twist ? gtsam::OptionalJacobian<6, 6>(H_exp)
                                                    : gtsam::OptionalJacobian<6, 6>());

        gtsam::Matrix36 H_trans;
        gtsam::Matrix33 H_trans_pt;
        const gtsam::Point3 pred = transformation.transformFrom(
            p_in_ref,
            H_twist ? gtsam::OptionalJacobian<3, 6>(H_trans) : gtsam::OptionalJacobian<3, 6>(),
            (H_twist || H_T) ? gtsam::OptionalJacobian<3, 3>(H_trans_pt)
                             : gtsam::OptionalJacobian<3, 3>());

        if (H_twist) *H_twist = H_trans * H_exp * t;
        if (H_T) *H_T = H_trans_pt * H_ext_pose;

        return pred - tgt_pt_;
    }
};
