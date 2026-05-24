#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <record/dataset_recorder.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<record::DatasetRecorder>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception &e) {
    std::fprintf(stderr, "oak_dataset_recorder failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
