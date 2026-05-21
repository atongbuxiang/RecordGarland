/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This component entry point mirrors run_subscribe_msckf.cpp but leaves
 * spinning to the ROS 2 component container.
 */

#include <memory>
#include <string>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros/ROS2Visualizer.h"
#include "utils/dataset_reader.h"

namespace ov_msckf {

class SubscribeMsckfNode {
public:
  explicit SubscribeMsckfNode(const rclcpp::NodeOptions &node_options) {
    auto options = node_options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("run_subscribe_msckf", options);

    std::string config_path = "unset_path_to_config.yaml";
    node_->get_parameter<std::string>("config_path", config_path);

    parser_ = std::make_shared<ov_core::YamlParser>(config_path);
    parser_->set_node(node_);

    std::string verbosity = "DEBUG";
    parser_->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    VioManagerOptions params;
    params.print_and_load(parser_);
    params.use_multi_threading_subs = true;
    sys_ = std::make_shared<VioManager>(params);

    viz_ = std::make_shared<ROS2Visualizer>(node_, sys_);
    viz_->setup_subscribers(parser_);

    if (!parser_->successful()) {
      PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
      throw std::runtime_error("OpenVINS parameter parsing failed");
    }

    viz_->start_image_publisher_thread();
    PRINT_DEBUG("done...spinning in component container\n");
  }

  ~SubscribeMsckfNode() {
    if (viz_ != nullptr) {
      viz_->visualize_final();
      viz_->stop_image_publisher_thread();
    }
    viz_.reset();
    sys_.reset();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_->get_node_base_interface();
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ov_core::YamlParser> parser_;
  std::shared_ptr<VioManager> sys_;
  std::shared_ptr<ROS2Visualizer> viz_;
};

} // namespace ov_msckf

RCLCPP_COMPONENTS_REGISTER_NODE(ov_msckf::SubscribeMsckfNode)
