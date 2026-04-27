// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!******************************************************************************
 *  \file       display_loader.cpp
 *  \brief      display loader implementation file.
 *  \authors    Asil Arnous
 ********************************************************************************/

#include "display_loader.hpp"

#include <QColor>
#include <QString>
#include <QVariant>

#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/parse_color.hpp>

#ifdef None
#undef None
#endif
#ifdef Bool
#undef Bool
#endif

namespace as2_camera_overlay
{
namespace
{
const std::set<std::string> kExcluded{
  "rviz_default_plugins/Camera",
  "rviz_default_plugins/Image",
};
}  // namespace

DisplayLoader::DisplayLoader(
  HeadlessDisplayContext * context,
  rclcpp::Logger logger)
: context_(context), logger_(std::move(logger)),
  loader_(std::make_unique<pluginlib::ClassLoader<rviz_common::Display>>(
      "rviz_common", "rviz_common::Display")) {}

DisplayLoader::~DisplayLoader()
{
  for (auto & d : displays_) {
    try {
      d->setEnabled(false);
    } catch (...) {
    }
  }
  displays_.clear();
}

bool DisplayLoader::isExcluded(const std::string & class_id)
{
  return kExcluded.count(class_id) > 0;
}

bool DisplayLoader::loadDisplay(const rviz_common::Config & config)
{
  QString class_id;
  if (!config.mapGetString("Class", &class_id)) {
    RCLCPP_ERROR(logger_, "Display configuration missing 'Class' key.");
    return false;
  }

  std::string class_id_std = class_id.toStdString();
  if (isExcluded(class_id_std)) {
    RCLCPP_ERROR(
      logger_,
      "Display '%s' requires a screen and is not supported.",
      class_id_std.c_str());
    return false;
  }

  std::shared_ptr<rviz_common::Display> display;
  try {
    display = loader_->createSharedInstance(class_id_std);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(
      logger_, "Failed to load plugin '%s': %s",
      class_id_std.c_str(), e.what());
    return false;
  }

  try {
    display->initialize(context_);
    display->load(config);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Plugin initialization/load failed: %s", e.what());
    return false;
  }

  displays_.push_back(std::move(display));
  RCLCPP_INFO(logger_, "Loaded RViz plugin: %s", class_id_std.c_str());
  return true;
}

void DisplayLoader::updateAll(float wall_dt, float ros_dt)
{
  for (auto & d : displays_) {
    if (!d->isEnabled()) {
      continue;
    }
    try {
      d->update(wall_dt, ros_dt);
    } catch (...) {
    }
  }
}
}  // namespace as2_camera_overlay
