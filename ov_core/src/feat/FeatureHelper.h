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

#ifndef OV_CORE_FEATURE_HELPER_H
#define OV_CORE_FEATURE_HELPER_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "Feature.h"
#include "FeatureDatabase.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Contains some nice helper functions for features.
 *
 * These functions should only depend on feature and the feature database.
 */
class FeatureHelper {

public:
  /**
   * @brief This functions will compute the disparity between common features in the two frames.
   *
   * First we find all features in the first frame.
   * Then we loop through each and find the uv of it in the next requested frame.
   * Features are skipped if no tracked feature is found (it was lost).
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param time0 First camera frame timestamp
   * @param time1 Second camera frame timestamp
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, 
                                double time0, 
                                double time1, 
                                double &disp_mean,
                                double &disp_var, 
                                int &total_feats) 
{

    // Get features seen from the first image
    std::vector<std::shared_ptr<Feature>> feats0 = db->features_containing(time0, false, true);

    // Compute the disparity
    std::vector<double> disparities;
    for (auto &feat : feats0) {

      // Get the two uvs for both times
      for (auto &campairs : feat->timestamps) { // code 遍历map容器的键，即相机id

        // First find the two timestamps
        size_t camid = campairs.first; // 获取相机id
        auto it0 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time0); // 在该相机id下查找time0时间戳
        auto it1 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time1);
        if (it0 == feat->timestamps.at(camid).end() || it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        // Now lets calculate the disparity
        Eigen::Vector2f uv0 = feat->uvs.at(camid).at(idx0).block(0, 0, 2, 1);
        Eigen::Vector2f uv1 = feat->uvs.at(camid).at(idx1).block(0, 0, 2, 1);
        disparities.push_back((uv1 - uv0).norm()); // note 计算两帧之间的视差
      }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

  /**
   * @brief This functions will compute the disparity over all features we have
   *
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   * @param newest_time Only compute disparity for ones older (-1 to disable)
   * @param oldest_time Only compute disparity for ones newer (-1 to disable)
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, 
                                double &disp_mean, 
                                double &disp_var, 
                                int &total_feats,
                                double newest_time = -1, 
                                double oldest_time = -1) 
  {
    // Compute the disparity
    std::vector<double> disparities;
    for (auto &feat : db->get_internal_data()) {       // std::unordered_map<size_t, std::shared_ptr<Feature>> 不同相机下对应的数据库
      for (auto &campairs : feat.second->timestamps) { // std::unordered_map<size_t, std::vector<double>>      遍历数据库中的时间戳(相机id, 时间戳)

        // Skip if only one observation
        if (campairs.second.size() < 2)
          continue;

        // Now lets calculate the disparity (assumes time array is monotonic)
        size_t camid = campairs.first;
        bool found0 = false;
        bool found1 = false;
        Eigen::Vector2f uv0 = Eigen::Vector2f::Zero();
        Eigen::Vector2f uv1 = Eigen::Vector2f::Zero();

        // feat := <size_t, std::shared_ptr<Feature>>
        // todo feat.second->timestamps.at(camid).size() 与 campairs.second.size()有区别吗？// 做下测试
        for (size_t idx = 0; idx < feat.second->timestamps.at(camid).size(); idx++) { // 遍历该数据库中该相机[id]下的时间戳
          double time = feat.second->timestamps.at(camid).at(idx); // 获取该数据库中该相机[id]下[idx]的时间戳
          if ((oldest_time == -1 || time > oldest_time) && !found0) {
            // uvs := std::unordered_map<size_t, std::vector<Eigen::VectorXf>> 
            // code at(idx)返回值元素类型为Eigen::VectorXf
            uv0 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1); // 获取的第一个时间戳的uv坐标
            found0 = true;
            continue;
          }
          if ((newest_time == -1 || time < newest_time) && found0) { // code 在满足第一个条件后，才会处理第二个条件
            uv1 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1); // 获取的第二个时间戳的uv坐标
            found1 = true;
            continue;
          }
        } // todo 这里的视差计算逻辑是什么？

        // If we found both an old and a new time, then we are good!
        if (!found0 || !found1)
          continue;
        disparities.push_back((uv1 - uv0).norm()); // note 计算视差（同一个特征点在不同时刻下的视差）
      }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2); // note 方差计算
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

private:
  // Cannot construct this class
  FeatureHelper() {}
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_HELPER_H */