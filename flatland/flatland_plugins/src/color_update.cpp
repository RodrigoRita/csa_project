#include <flatland_server/model_plugin.h>
#include <flatland_server/model.h>
#include <flatland_server/body.h>
#include <flatland_server/timekeeper.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <string>
#include <iostream>

namespace flatland_plugins {

class ColorUpdate : public flatland_server::ModelPlugin {
public:
  ColorUpdate() = default;
  ~ColorUpdate() override = default;

  void OnInitialize(const YAML::Node &config) override {
    std::cout << "[ColorUpdate] OnInitialize called" << std::endl;

    model_ = GetModel();
    std::cout << "[ColorUpdate] Model name: " << model_->GetName() << std::endl;

    if (!config["body"]) {
      throw flatland_server::YAMLException("[ColorUpdate] Missing 'body' in YAML");
    }

    std::string body_name = config["body"].as<std::string>();
    std::cout << "[ColorUpdate] Looking for body: " << body_name << std::endl;

    body_ = model_->GetBody(body_name);
    if (!body_) {
      throw flatland_server::YAMLException(
        "[ColorUpdate] Body with name \"" + body_name + "\" does not exist");
    }

    std::cout << "[ColorUpdate] Body found successfully" << std::endl;

    std::string topic_name = "color_update";
    if (config["topic"]) {
      topic_name = config["topic"].as<std::string>();
    }

    std::cout << "[ColorUpdate] Subscribing to topic: " << topic_name << std::endl;

    node_ = rclcpp::Node::make_shared(model_->GetName() + "_color_update");

    color_sub_ = node_->create_subscription<std_msgs::msg::ColorRGBA>(
      topic_name, 10,
      [this](const std_msgs::msg::ColorRGBA::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_color_ = *msg;
        update_color_ = true;

        std::cout << "[ColorUpdate] Color message received: "
                  << "r=" << msg->r << " "
                  << "g=" << msg->g << " "
                  << "b=" << msg->b << " "
                  << "a=" << msg->a << std::endl;
      });

    executor_.add_node(node_);

    std::cout << "[ColorUpdate] Initialization complete" << std::endl;
  }

  void BeforePhysicsStep(const flatland_server::Timekeeper &) override {
    executor_.spin_some();

    static int counter = 0;
    if (++counter % 200 == 0) {
      std::cout << "[ColorUpdate] BeforePhysicsStep running..." << std::endl;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (update_color_ && body_) {
      std::cout << "[ColorUpdate] Applying color to body" << std::endl;

      body_->SetColor(
        flatland_server::Color(
          latest_color_.r,
          latest_color_.g,
          latest_color_.b,
          latest_color_.a));

      update_color_ = false;
    }
  }

private:
  flatland_server::Model *model_{nullptr};
  flatland_server::Body *body_{nullptr};

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::ColorRGBA>> color_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  std_msgs::msg::ColorRGBA latest_color_;
  bool update_color_{false};
  std::mutex mutex_;
};

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::ColorUpdate,
                       flatland_server::ModelPlugin)
