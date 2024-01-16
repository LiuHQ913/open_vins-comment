/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "UpdaterZeroVelocity.h"

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterZeroVelocity::UpdaterZeroVelocity(UpdaterOptions &options, 
                                         NoiseManager &noises, 
                                         std::shared_ptr<ov_core::FeatureDatabase> db,
                                         std::shared_ptr<Propagator> prop, 
                                         double gravity_mag, 
                                         double zupt_max_velocity,
                                         double zupt_noise_multiplier, 
                                         double zupt_max_disparity)
    : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
      _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity) 
{
  // Gravity
  _gravity << 0.0, 0.0, gravity_mag;

  // Save our raw pixel noise squared
  _noises.sigma_w_2  = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2  = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 1000; i++) { // 创建一个卡方分布表
    boost::math::chi_squared chi_squared_dist(i); // 卡方分布自由度为i
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

bool UpdaterZeroVelocity::try_update(std::shared_ptr<State> state, double timestamp) {

  // Return if we don't have any imu data yet
  if (imu_data.empty()) {
    last_zupt_state_timestamp = 0.0; // 最后的一个zupt的时间戳，复位
    return false;
  }

  // Return if the state is already at the desired time
  if (state->_timestamp == timestamp) {
    last_zupt_state_timestamp = 0.0; // 最后的一个zupt的时间戳，复位
    return false;
  }

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0); // 获取相机到imu的时间偏移量
    have_last_prop_time_offset = true;
  }

  // assert that the time we are requesting is in the future
  // assert(timestamp > state->_timestamp);

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0); // 获取相机到imu的时间偏移量

  // First lets construct an IMU vector of measurements we need
  // todo last_prop_time_offset与t_off_new的区别？ // lhq 一个是上一次的时间偏移量，一个是当前的时间偏移量
  // double time0 = state->_timestamp+t_off_new;
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;

  // Select bounding inertial measurements
  std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1); // 获取time0~time1之间的imu数据

  // Move forward in time
  last_prop_time_offset = t_off_new; // 维护时间偏移量

  // Check that we have at least one measurement to propagate with
  if (imu_recent.size() < 2) {
    PRINT_WARNING(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // If we should integrate the acceleration and say the velocity should be zero
  // Also if we should still inflate the bias based on their random walk noises
  /*
    功能选项：
      1. 系统是否积分加速度并假设速度应为零。这是一个未经测试的特性，所以默认值为假。
      2. 系统是否考虑随时间变化的随机游走偏差。在实际的IMU数据处理中，通常需要考虑随时间变化的偏差，所以默认值为真。
      3. 系统是否通过检查图像的视差来覆盖其他检查。因为如果视差较大，那么很可能设备正在移动，不应进行零速度更新。默认值为真。
      4. 系统是否明确地强制运动为零。这是一个更强硬的措施，只有在确定设备完全静止的情况下才会使用。默认值为假。
  */
  bool integrated_accel_constraint = false; // untested
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order; // 存放状态变量的指针
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  if (integrated_accel_constraint) { // todo 这是在做什么？
    Hx_order.push_back(state->_imu->v());
  }

  // note 基于惯性的zupt检测
  // Large final matrices used for update (we will compress these)
  int h_size = (integrated_accel_constraint) ? 12 : 9; // note 这里是9维度，q提供3个自由度 // todo 各个开源算法中都是这样吗？
  int m_size = 6 * ((int)imu_recent.size() - 1);       // 一个imu提供6个变量
  Eigen::MatrixXd H   = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);

  // IMU intrinsic calibration estimates (static)
  Eigen::Matrix3d Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value()); // imu 内参矩阵，这里都是单位矩阵
  Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value()); // 零矩阵

  // 参考 https://docs.openvins.com/update-zerovelocity.html#update-zerovelocity-meas
  // Loop through all our IMU and construct the residual and Jacobian
  // TODO: should add jacobians here in respect to IMU intrinsics!!
  // State order is: [q_GtoI, bg, ba, v_IinG] // note 这里坐标系变换
  // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na  // todo R、g的坐标系 // lhq g在G系下
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) { // 遍历imu测量

    // Precomputed values
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    /*
      残差： err = a_true - measurement function
            a_true = 0 => err = -measurement function
      残差： a_hat = am - ba
      残差： w_hat = wm - bg 
      注意，白噪声的均值为0
    */
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot()  * Da * (imu_recent.at(i).am - state->_imu->bias_a()); // imu系下的测量
    Eigen::Vector3d w_hat = state->_calib_imu_GYROtoIMU->Rot() * Dw * (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);

    // note 连续噪声 -> 离散噪声 ; 白化
    // Measurement noise (convert from continuous to discrete)
    // NOTE: The dt time might be different if we have "cut" any imu measurements
    // NOTE: We are performing "whittening" thus, we will decompose R_meas^-1 = L*L^t
    // NOTE: This is then multiplied to the residual and Jacobian (equivalent to just updating with R_meas)
    // NOTE: See Maybeck Stochastic Models, Estimation, and Control Vol. 1 Equations (7-21a)-(7-21c)
    double w_omega = std::sqrt(dt) / _noises.sigma_w; // todo 这是标准差的倒数吗？
    double w_accel = std::sqrt(dt) / _noises.sigma_a;
    double w_accel_v = 1.0 / (std::sqrt(dt) * _noises.sigma_a);

    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) = -w_omega * w_hat; // todo 带有权重的残差？
    if (!integrated_accel_constraint) {  // 加速度残差
      res.block(6 * i + 3, 0, 3, 1) = -w_accel * (a_hat - state->_imu->Rot() * _gravity);
    }
    else { // 速度残差
      res.block(6 * i + 3, 0, 3, 1) = -w_accel_v * (state->_imu->vel() - _gravity * dt + state->_imu->Rot().transpose() * a_hat * dt);
    }

    // Measurement Jacobian
    // todo state->_imu->Rot_fej()返回的fej矩阵，是如何计算的？
    // gpt -w_omega是一个根据时间间隔和噪声标准差计算出的权重，用于在状态估计过程中对角速度测量进行适当的加权和白化
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -w_omega * Eigen::Matrix3d::Identity();       // 对bg的导数 // TODO w_omega表示什么 // lhq 白化
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * _gravity); // 对R_GtoI的导数
      H.block(6 * i + 3, 6, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();     // 对ba的导数
    } 
    else {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) =  w_accel_v * Eigen::Matrix3d::Identity();
    }
    dt_summed += dt;
  }

  // Compress the system (we should be over determined)
  UpdaterHelper::measurement_compress_inplace(H, res); // todo 左零空间投影，排除什么信息？ 做下打印吧
  if (H.rows() < 1) {
    return false;
  }

  // note 对噪声矩阵进行缩放，通常是为了增加噪声的方差，使得系统更加保守，不能太相信imu。
  // Multiply our noise matrix by a fixed amount
  // We typically need to treat the IMU as being "worst" to detect / not become overconfident
  Eigen::MatrixXd R = _zupt_noise_multiplier * Eigen::MatrixXd::Identity(res.rows(), res.rows()); // 默认单位矩阵 // todo 为什么是这个大小

  // note bias 协方差传播 // todo（帧间传播）？
  // todo 这里是如何计算bias的协方差的？
  // 雅可比矩阵为单位矩阵
  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*(1/dt*Qc)*dt = dt*Qc // todo Qd是离散 Qc是连续？
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb_2; // imu测量累积时间 * 角度随机游走协方差
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab_2;

  // note 卡方距离检验（Chi-squared test），用于确定状态更新是否应该被接受。用于确定观测数据是否与预期的分布一致
  /*
    gpt 在这段代码中，作者提到了关于IMU偏差（bias）的传播。
    gpt 通常，在状态估计中，我们会在每次更新之前对状态进行预测或传播，以考虑在两次更新之间的系统动态。
    gpt 在这种情况下，偏差的传播是指在状态更新之前，根据IMU的随机游走模型预测偏差的变化。
    gpt 然而，作者在这里指出，他们并没有首先执行这个传播步骤，因为如果卡方检验（chi-squared test）失败了，他们希望直接返回并执行正常的逻辑处理。
    gpt 这意味着，只有在卡方检验通过，状态更新被接受的情况下，他们才会考虑偏差的传播。
  */
  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if this was to be accepted (just the bias evolution)
  // NOTE: we don't propagate first since if we fail the chi2 then we just want to return and do normal logic
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  if (model_time_varying_bias) { // 考虑随时间变化的随机游走偏差
    // 起始位置[3 3]，大小6x6
    P_marg.block(3, 3, 6, 6) += Q_bias; // gpt 将偏差的噪声协方差矩阵Q_bias添加到边缘协方差矩阵的对应块中。这是为了考虑在状态更新之前偏差的随机游走。
  }
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R; // note 协方差传播，参考文档https://docs.openvins.com/update-zerovelocity.html
  // code S.llt().solve(res)是利用S的Cholesky分解来解线性方程 S*x=res，得到向量x，即 x=(S^-1)*res。chi2 = res*(S^-1)*res
  // note 卡方距离检验（Chi-squared test），以判断当前的状态估计是否可靠。
  double chi2 = res.dot(S.llt().solve(res));

  // Get our threshold (we precompute up to 1000 but handle the case that it is more)
  double chi2_check;
  if (res.rows() < 1000) { // todo 怎么是res表示自由度？ 自由度：自由变化的个数（观测值）
    chi2_check = chi_squared_table[res.rows()];
  } 
  else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }
  // --- end 基于惯性的zupt检测

  // note 基于视差的zupt检测
  // Check if the image disparity
  bool disparity_passed = false; // 标识图像视差检查是否通过
  if (override_with_disparity_check) {

    // Get the disparity statistics from this image to the previous
    double time0_cam = state->_timestamp;
    double time1_cam = timestamp; // Next camera timestamp we want to see if we should propagate to
    int num_features = 0;
    double disp_avg = 0.0;
    double disp_var = 0.0;
    // 出参：计算视差均值、方差、个数
    FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, disp_avg, disp_var, num_features);

    // Check if this disparity is enough to be classified as moving
    disparity_passed = (disp_avg < _zupt_max_disparity && num_features > 20); // note 基于视差的zupt检测阈值
    if (disparity_passed) {
      PRINT_INFO(CYAN "[ZUPT]: passed disparity (%.3f < %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    } 
    else {
      PRINT_DEBUG(YELLOW "[ZUPT]: failed disparity (%.3f > %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    }
  }
  // --- end 基于视差的zupt检测

  // Check if we are currently zero velocity
  // We need to pass the chi2 and not be above our velocity threshold
  if (!disparity_passed && 
      (chi2 > _options.chi2_multipler * chi2_check || state->_imu->vel().norm() > _zupt_max_velocity)) 
  {
    last_zupt_state_timestamp = 0.0;
    last_zupt_count = 0;
    PRINT_DEBUG(YELLOW "[ZUPT]: rejected |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
                _options.chi2_multipler * chi2_check);
    return false;
  }
  PRINT_INFO(CYAN "[ZUPT]: accepted |v_IinG| = %.3f (chi2 %.3f < %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
             _options.chi2_multipler * chi2_check);

  /*
    todo 这段注释在说啥？
    gpt 如果已经进行过至少两次ZUPT，那么就不会在这个时间戳进行克隆操作，而是进行零速度更新。
    gpt 这是因为在第二次及以后的ZUPT中，不需要克隆状态，而是直接进行更新。
    gpt 因此，这段代码的目的是在进行了至少两次ZUPT后，清理对应时间戳的测量数据。
  */
  // Do our update, only do this update if we have previously detected
  // If we have succeeded, then we should remove the current timestamp feature tracks
  // This is because we will not clone at this timestep and instead do our zero velocity update
  // NOTE: We want to keep the tracks from the second time we have called the zv-upt since this won't have a clone
  // NOTE: All future times after the second call to this function will also *not* have a clone, so we can remove those
  if (last_zupt_count >= 2) {
    _db->cleanup_measurements_exact(last_zupt_state_timestamp); // 清理last_zupt_state_timestamp时间戳的测量数据
  }

  /* 
    gpt 零速度更新是一种用于惯性导航系统（INS）的技术，它利用了当载体静止时，速度为零的事实来校正IMU（惯性测量单元）的误差。
    gpt 代码的主要逻辑分为两部分：
    gpt   1. 如果explicitly_enforce_zero_motion标志为false，则执行正常的状态更新。这包括使用IMU测量直接更新状态，以及将偏差（bias）向前传播。
    gpt   2. 如果explicitly_enforce_zero_motion标志为true， 则执行一个更为严格的更新。这涉及到将状态向前传播，并显式地将方向（orientation）、位置（position）和速度（velocity）设置为零。

    gpt 在第二部分中，代码首先使用propagate_and_clone方法将状态向前传播到新的时间戳。
    gpt 然后，它创建了一个更新系统，包括残差（residuals）、雅可比矩阵（Jacobian）和噪声矩阵（noise matrix）。
    gpt 残差是基于两个时间戳的IMU克隆状态之间的方向、位置和速度差异计算的。雅可比矩阵和噪声矩阵用于在扩展卡尔曼滤波（EKF）更新步骤中定义状态更新的线性化模型和测量噪声。
    gpt 最后，使用EKFUpdate方法更新状态，并通过marginalize方法和erase操作移除旧的克隆状态，以保持状态向量的大小不变。
  */
  // Else we are good, update the system
  // 1) update with our IMU measurements directly
  // 2) propagate and then explicitly say that our ori, pos, and vel should be zero
  if (!explicitly_enforce_zero_motion) {

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc // todo 这是什么公式？
    if (model_time_varying_bias) { // 考虑随时间变化的随机游走偏差,更新协方差
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg()); // todo 这里bg、ba在该函数前段有修改吗？
      Phi_order.push_back(state->_imu->ba());
      // kernel 传播(相关)状态变量协方差矩阵
      StateHelper::EKFPropagation(state, 
                                  Phi_order, // order_NEW Contiguous variables that have evolved according to this state transition
                                  Phi_order, // order_OLD Variable ordering used in the state transition
                                  Phi_bias,  // Phi       State transition matrix (size order_NEW by size order_OLD) 状态转移矩阵
                                  Q_bias);   // Q         Additive state propagation noise matrix (size order_NEW by size order_NEW)  
      // tips 这里Phi_bias为单位矩阵，bias传播仅累加了Q_bias
    }

    // Finally move the state time forward
    // kernel EKF更新卡尔曼增益，协方差矩阵，状态量（残差状态量）
    StateHelper::EKFUpdate(state, 
                           Hx_order, // 存放状态变量的指针
                           H,        // H   压缩的
                           res,      // res 压缩的
                           R);       // R   压缩的
    state->_timestamp = timestamp;

  } 
  else { // propagate and then explicitly say that our ori, pos, and vel should be zero

    // Propagate the state forward in time
    double time0_cam = last_zupt_state_timestamp;
    double time1_cam = timestamp;
    _prop->propagate_and_clone(state, time1_cam);

    // Create the update system!
    H = Eigen::MatrixXd::Zero(9, 15);
    res = Eigen::VectorXd::Zero(9);
    R = Eigen::MatrixXd::Identity(9, 9);

    // residual (order is ori, pos, vel)
    Eigen::Matrix3d R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot();
    Eigen::Vector3d p_I0inG = state->_clones_IMU.at(time0_cam)->pos();
    Eigen::Matrix3d R_GtoI1 = state->_clones_IMU.at(time1_cam)->Rot();
    Eigen::Vector3d p_I1inG = state->_clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->_imu->vel();
    res *= -1;

    // jacobian (order is q0, p0, q1, p1, v0)
    Hx_order.clear();
    Hx_order.push_back(state->_clones_IMU.at(time0_cam));
    Hx_order.push_back(state->_clones_IMU.at(time1_cam));
    Hx_order.push_back(state->_imu->v());
    if (state->_options.do_fej) {
      R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot_fej();
    }
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, 6, 3, 3) = -R_GtoI0;
    H.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    H.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(6, 12, 3, 3) = Eigen::Matrix3d::Identity();

    // noise (order is ori, pos, vel)
    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    // finally update and remove the old clone
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(time1_cam));
    state->_clones_IMU.erase(time1_cam);
  }

  // Finally return
  last_zupt_state_timestamp = timestamp;
  last_zupt_count++;
  return true;
}