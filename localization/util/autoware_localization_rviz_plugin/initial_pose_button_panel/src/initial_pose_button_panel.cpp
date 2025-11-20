/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <QHBoxLayout>
#include <QSizePolicy>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

#include "initial_pose_button_panel/initial_pose_button_panel.hpp"

namespace autoware_localization_rviz_plugin
{
namespace
{
constexpr char kDefaultTopic[] = "/sensing/gnss/pose_with_covariance";
constexpr char kInitializerService[] = "/localization/util/pose_initializer_srv";
constexpr auto kServiceWait = std::chrono::seconds(5);
}  // namespace

InitialPoseButtonPanel::InitialPoseButtonPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  topic_label_ = new QLabel(QStringLiteral("PoseWithCovarianceStamped"));
  topic_label_->setAlignment(Qt::AlignCenter);

  topic_edit_ = new QLineEdit(QString::fromUtf8(kDefaultTopic));
  connect(topic_edit_, &QLineEdit::textEdited, this, &InitialPoseButtonPanel::editTopic);

  initialize_button_ = new QPushButton(QStringLiteral("Wait for subscribe topic"));
  initialize_button_->setEnabled(false);
  connect(initialize_button_, &QPushButton::clicked, this, &InitialPoseButtonPanel::pushInitialzeButton);

  status_label_ = new QLabel(QStringLiteral("Not Initialized"));
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet(QStringLiteral("QLabel { background-color : gray;}"));

  QSizePolicy q_size_policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  initialize_button_->setSizePolicy(q_size_policy);

  auto * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(topic_label_);
  topic_layout->addWidget(topic_edit_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(topic_layout);
  v_layout->addWidget(initialize_button_);
  v_layout->addWidget(status_label_);

  setLayout(v_layout);
}

void InitialPoseButtonPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    throw std::runtime_error("Failed to lock RosNodeAbstraction");
  }

  node_ = ros_node_abstraction->get_raw_node();
  setupRosInterfaces();
}

void InitialPoseButtonPanel::setupRosInterfaces()
{
  if (!node_) {
    return;
  }

  subscribeToTopic(topic_edit_->text().toStdString());
  client_ = node_->create_client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>(
    kInitializerService);
}

void InitialPoseButtonPanel::subscribeToTopic(const std::string & topic_name)
{
  if (!node_) {
    return;
  }

  pose_subscription_.reset();
  pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_name, rclcpp::SensorDataQoS(),
    std::bind(&InitialPoseButtonPanel::callbackPoseCov, this, std::placeholders::_1));

  updateButton(QStringLiteral("Wait for subscribe topic"), false);
}

void InitialPoseButtonPanel::callbackPoseCov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = *msg;
  }

  updateButton(QStringLiteral("Pose Initializer   Let's GO!"), true);
}

void InitialPoseButtonPanel::editTopic()
{
  if (!node_) {
    return;
  }
  subscribeToTopic(topic_edit_->text().toStdString());
}

void InitialPoseButtonPanel::pushInitialzeButton()
{
  if (!client_) {
    updateStatus(QStringLiteral("No Service Client"), QStringLiteral("QLabel { background-color : red;}"));
    return;
  }

  updateButton(QStringLiteral("Initializing..."), false);
  updateStatus(QStringLiteral("Initializing..."), QStringLiteral("QLabel { background-color : dodgerblue;}"));

  auto request = std::make_shared<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request>();
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    request->pose_with_cov = latest_pose_;
  }

  std::thread([this, request]() {
    if (!client_->wait_for_service(kServiceWait)) {
      updateStatus(QStringLiteral("Service Unavailable"), QStringLiteral("QLabel { background-color : red;}"));
      updateButton(QStringLiteral("Pose Initializer   Let's GO!"), true);
      return;
    }

    auto future = client_->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(node_, future, kServiceWait);
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      updateStatus(QStringLiteral("OK!!!"), QStringLiteral("QLabel { background-color : lightgreen;}"));
    } else {
      updateStatus(QStringLiteral("Failed!"), QStringLiteral("QLabel { background-color : red;}"));
    }
    updateButton(QStringLiteral("Pose Initializer   Let's GO!"), true);
  }).detach();
}

void InitialPoseButtonPanel::updateStatus(const QString & text, const QString & style)
{
  QMetaObject::invokeMethod(
    status_label_,
    [this, text, style]() {
      status_label_->setText(text);
      status_label_->setStyleSheet(style);
    },
    Qt::QueuedConnection);
}

void InitialPoseButtonPanel::updateButton(const QString & text, bool enabled)
{
  QMetaObject::invokeMethod(
    initialize_button_,
    [this, text, enabled]() {
      initialize_button_->setText(text);
      initialize_button_->setEnabled(enabled);
    },
    Qt::QueuedConnection);
}

}  // namespace autoware_localization_rviz_plugin

PLUGINLIB_EXPORT_CLASS(autoware_localization_rviz_plugin::InitialPoseButtonPanel, rviz_common::Panel)
