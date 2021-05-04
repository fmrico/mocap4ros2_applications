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


#include <rqt_mocap_control/mocap_control.h>

#include <pluginlib/class_list_macros.hpp>

#include <QDebug>
#include <QTime>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

#include "mocap_control_msgs/msg/mocap_info.hpp"

namespace rqt_mocap_control {

MocapControl::MocapControl()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("MocapControl");
}

void MocapControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.treeWidget->setColumnCount(3);
  ui_.treeWidget->setHeaderLabels({"Active", "Topic", "Record"});
  
  // Add tf and tf_static
  auto system_item =  new QTreeWidgetItem();
  system_item->setText(1, "System");
  system_item->setCheckState(0, Qt::Checked);
  ui_.treeWidget->addTopLevelItem(system_item);
  auto tf_item = new QTreeWidgetItem();
  auto tf_static_item = new QTreeWidgetItem();
  tf_item->setText(1, "tf");
  tf_item->setCheckState(2, Qt::Unchecked);
  tf_static_item->setText(1, "tf_static");
  tf_static_item->setCheckState(2, Qt::Unchecked);
  system_item->addChild(tf_item);
  system_item->addChild(tf_static_item);

  ui_.treeWidget->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui_.treeWidget->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui_.treeWidget->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

  mocap_env_sub_ = node_->create_subscription<mocap_control_msgs::msg::MocapInfo>(
    "mocap_environment", rclcpp::QoS(1000).reliable().transient_local().keep_all(),
    [this] (const mocap_control_msgs::msg::MocapInfo::SharedPtr msg)
    {
      mocap_env_[msg->system_source] = *msg;
      update_tree();
    });
  
  controller_node_ = std::make_shared<mocap_control::ControllerNode>();

  QObject::connect(ui_.startButton, &QPushButton::clicked, ui_.startButton, [=]()->void{std::cerr << "clicked";});

  connect(ui_.startButton, SIGNAL(clicked()), this, SLOT(start_capture()));
  connect(ui_.recordAllCheckBox, SIGNAL(clicked(bool)), this, SLOT(select_record_all(bool)));
  connect(ui_.activeAllCheckBox, SIGNAL(clicked(bool)), this, SLOT(select_active_all(bool)));
}

void MocapControl::shutdownPlugin()
{
}

void MocapControl::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

void MocapControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}

void
MocapControl::update_tree()
{
  for (const auto & system : mocap_env_) {
    auto system_item_list = ui_.treeWidget->findItems(QString(system.first.c_str()), Qt::MatchExactly, 0);

    // Get System root node
    QTreeWidgetItem * system_item;
    if (system_item_list.empty()) {  // Insert a new root if it is new
      system_item =  new QTreeWidgetItem();
      system_item->setText(1, QString(system.first.c_str()));
      system_item->setCheckState(0, Qt::Checked);

      ui_.treeWidget->addTopLevelItem(system_item);
    } else {
      system_item = system_item_list.front();
    }

    // Check if already exist, and add otherwise
    for (const auto & topic : system.second.topics) {
      bool already_exist = false;
      for (int i = 0; i < system_item->childCount(); i++) {
        already_exist = already_exist || system_item->child(i)->text(1) == QString(topic.c_str());
      }
      if (!already_exist) {
        QTreeWidgetItem * new_item = new QTreeWidgetItem();
        new_item->setText(1, QString(topic.c_str()));
        new_item->setCheckState(2, Qt::Unchecked);
        system_item->addChild(new_item);
      }
    }
  }
}

void
MocapControl::start_capture()
{
  if (!capturing_) {
    std::vector<std::string> capture_systems;
    for (int i = 0; i < ui_.treeWidget->topLevelItemCount(); ++i ) {
      if (ui_.treeWidget->topLevelItem(i)->checkState(0) == Qt::Checked &&
        ui_.treeWidget->topLevelItem(i)->text(1) != "System")
      {
        capture_systems.push_back(ui_.treeWidget->topLevelItem(i)->text(1).toUtf8().constData());
      }
    }

    controller_node_->start_system(ui_.sessionTextEdit->toPlainText().toUtf8().constData(), capture_systems);
    capturing_ = true;

    ui_.startButton->setText("Stop");
    QPalette pal = ui_.startButton->palette();
    pal.setColor(QPalette::Button, QColor(Qt::red));
    ui_.startButton->setAutoFillBackground(true);
    ui_.startButton->setPalette(pal);
    ui_.startButton->update();
  } else {
    controller_node_->stop_system();
    capturing_ = false;

    ui_.startButton->setText("Start");
    QPalette pal = ui_.startButton->palette();
    pal.setColor(QPalette::Button, QColor(Qt::lightGray));
    ui_.startButton->setAutoFillBackground(true);
    ui_.startButton->setPalette(pal);
    ui_.startButton->update();
  }
}  

void
MocapControl::select_record_all(bool checked)
{
  for (int i = 0; i < ui_.treeWidget->topLevelItemCount(); ++i ) {
    for (int j = 0; j < ui_.treeWidget->topLevelItem(i)->childCount(); j++) {
       ui_.treeWidget->topLevelItem(i)->child(j)->setCheckState(2, checked? Qt::Checked : Qt::Unchecked);
    }
  }
}

void
MocapControl::select_active_all(bool checked)
{
  for (int i = 0; i < ui_.treeWidget->topLevelItemCount(); ++i ) {
    ui_.treeWidget->topLevelItem(i)->setCheckState(0, checked? Qt::Checked : Qt::Unchecked);
  }
}

}  // namespace rqt_mocap_control

PLUGINLIB_EXPORT_CLASS(rqt_mocap_control::MocapControl, rqt_gui_cpp::Plugin)
