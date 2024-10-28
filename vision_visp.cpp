/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2022 by INRIA. All rights reserved.
 *
 * Description:
 * Calibrator node with validation checks and enhanced image processing.
 *
 *****************************************************************************/

#include "visp_hand2eye_calibration/calibrator.h"
#include "visp_hand2eye_calibration/names.h"
#include <visp_bridge/3dpose.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/vision/vpHandEyeCalibration.h>

namespace visp_hand2eye_calibration
{

bool Calibrator::validateTransformData() const {
    if (cMo_vec_.size() != wMe_vec_.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transformation vectors have different sizes.");
        return false;
    }
    if (cMo_vec_.empty() || wMe_vec_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transformation vectors are empty.");
        return false;
    }
    return true;
}

void Calibrator::cameraObjectCallback(const geometry_msgs::msg::Transform &trans)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "new cMo: [%lf,%lf,%lf -- %lf,%lf,%lf,%lf]", trans.translation.x,
               trans.translation.y, trans.translation.z, trans.rotation.x, trans.rotation.y, trans.rotation.z,
               trans.rotation.w);
  vpHomogeneousMatrix cMo;
  cMo = visp_bridge::toVispHomogeneousMatrix(trans);
  cMo_vec_.push_back(cMo);
}

void Calibrator::worldEffectorCallback(const geometry_msgs::msg::Transform &trans)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "new wMe: [%lf,%lf,%lf -- %lf,%lf,%lf,%lf]", trans.translation.x,
               trans.translation.y, trans.translation.z, trans.rotation.x, trans.rotation.y, trans.rotation.z,
               trans.rotation.w);
  vpHomogeneousMatrix wMe;
  wMe = visp_bridge::toVispHomogeneousMatrix(trans);
  wMe_vec_.push_back(wMe);
}

void Calibrator::computeEffectorCameraCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Request> req,
    std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Response> res)
{
  (void)request_header;
  (void)req;

  // Validate data before processing
  if (!validateTransformData()) {
      return;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "computing %d values...", (int)wMe_vec_.size());
  vpHomogeneousMatrix eMc;
  vpHandEyeCalibration::calibrate(cMo_vec_, wMe_vec_, eMc);
  geometry_msgs::msg::Transform trans;
  trans = visp_bridge::toGeometryMsgsTransform(eMc);

  res->effector_camera = trans;
}

void Calibrator::computeEffectorCameraQuickCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Request> req,
    std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Response> res)
{
  (void)request_header;

  visp_hand2eye_calibration::msg::TransformArray camera_object = req->camera_object;
  visp_hand2eye_calibration::msg::TransformArray world_effector = req->world_effector;

  std::vector<vpHomogeneousMatrix> cMo_vec;
  std::vector<vpHomogeneousMatrix> wMe_vec;
  for (unsigned int i = 0; i < camera_object.transforms.size(); i++) {
      cMo_vec.push_back(visp_bridge::toVispHomogeneousMatrix(camera_object.transforms[i]));
      wMe_vec.push_back(visp_bridge::toVispHomogeneousMatrix(world_effector.transforms[i]));
  }

  if (camera_object.transforms.size() != world_effector.transforms.size() || world_effector.transforms.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Transformation vectors have different sizes or contain too few elements.");
      return;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "computing...");
  vpHomogeneousMatrix eMc;
  vpHandEyeCalibration::calibrate(cMo_vec, wMe_vec, eMc);
  geometry_msgs::msg::Transform trans;
  trans = visp_bridge::toGeometryMsgsTransform(eMc);
  res->effector_camera = trans;
}

void Calibrator::resetCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Request> req,
                               std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Response> res)
{
  (void)request_header;
  (void)req;
  (void)res;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "resetting...");
  cMo_vec_.clear();
  wMe_vec_.clear();
  return;
}

Calibrator::Calibrator(const rclcpp::NodeOptions &options) : Node("calibrator", options)
{
  // define subscribers
  camera_object_subscriber_ = this->create_subscription<geometry_msgs::msg::Transform>(
      visp_hand2eye_calibration::camera_object_topic, queue_size_,
      std::bind(&Calibrator::cameraObjectCallback, this, std::placeholders::_1));
  world_effector_subscriber_ = this->create_subscription<geometry_msgs::msg::Transform>(
      visp_hand2eye_calibration::world_effector_topic, queue_size_,
      std::bind(&Calibrator::worldEffectorCallback, this, std::placeholders::_1));

  // define services
  compute_effector_camera_service_ = this->create_service<visp_hand2eye_calibration::srv::ComputeEffectorCamera>(
      visp_hand2eye_calibration::compute_effector_camera_service,
      std::bind(&Calibrator::computeEffectorCameraCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));

  compute_effector_camera_quick_service_ =
      this->create_service<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick>(
          visp_hand2eye_calibration::compute_effector_camera_quick_service,
          std::bind(&Calibrator::computeEffectorCameraQuickCallback, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));

  reset_service_ = this->create_service<visp_hand2eye_calibration::srv::Reset>(
      "reset_service",
      std::bind(&Calibrator::resetCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

} // namespace visp_hand2eye_calibration
