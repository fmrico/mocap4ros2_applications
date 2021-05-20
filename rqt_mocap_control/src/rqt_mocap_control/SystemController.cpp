// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QString>

#include "mocap_control/ControllerNode.hpp"

#include "rqt_mocap_control/SystemController.hpp"

namespace rqt_mocap_control {

SystemController::SystemController(const std::string & system_name)
: QTreeWidgetItem(), system_name_(system_name)
{
  setCheckState(0, Qt::Checked);
  setText(1, QString(system_name.c_str()));
}

void
SystemController::add_topic(const std::string & topic)
{
  if (topics_.find(topic) == topics_.end()) {
    topics_[topic] = {nullptr};
    update_topics();
  }
}

const std::vector<std::string>
SystemController::get_topics()
{
  std::vector<std::string> ret;
  for (const auto & topic : topics_) {
    ret.push_back(topic.first);
  }
  return ret;
}

void
SystemController::set_active(bool active)
{
  setCheckState(0, active? Qt::Checked : Qt::Unchecked);
}

void
SystemController::set_log_all(bool log)
{
  for (auto & topic : topics_) {
    topic.second.item->setCheckState(2, log? Qt::Checked : Qt::Unchecked);
  }
}

void
SystemController::update_topics()
{
  for (auto & topic : topics_) {
    // Initialize new entries
    if (topic.second.item == nullptr) {
      topic.second.item = new QTreeWidgetItem();
      topic.second.item->setText(1, QString(topic.first.c_str()));
      topic.second.item->setCheckState(2, Qt::Unchecked);
      addChild(topic.second.item);
    }
  }
}

void
SystemController::update_elapsed_ts(double elapsed)
{
  setText(3, QString(std::to_string(elapsed).c_str()) + " secs");

}


}  // namespace rqt_mocap_control

