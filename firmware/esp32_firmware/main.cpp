#include <Arduino.h>

// 下面的代码展示了一个最小的 micro-ROS 客户端骨架。
// 实际工程中需要根据使用的 LIDAR、IMU、电机驱动等补全对应的驱动调用。

extern "C" {
#include <micro_ros_arduino.h>
}

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// Wi-Fi 与 agent 地址需要根据实际网络环境配置
static const char *wifi_ssid = "YOUR_WIFI_SSID";
static const char *wifi_password = "YOUR_WIFI_PASSWORD";
static const char *agent_address = "192.168.1.100";
static const uint16_t agent_port = 8888;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t odom_publisher;
static rcl_timer_t timer;
static rclc_executor_t executor;
static std_msgs__msg__String odom_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer == nullptr) {
    return;
  }

  const char *text = "dummy odom from micro-ros esp32";
  odom_msg.data.data = const_cast<char *>(text);
  odom_msg.data.size = strlen(text);
  odom_msg.data.capacity = odom_msg.data.size + 1;

  rcl_publish(&odom_publisher, &odom_msg, nullptr);
}

void setup()
{
  Serial.begin(115200);
  set_microros_wifi_transports(wifi_ssid, wifi_password, agent_address, agent_port);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, nullptr, &allocator);

  rclc_node_init_default(&node, "esp32_microros_node", "", &support);

  rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "esp32/odom_text");

  const unsigned int period_ms = 100;
  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(period_ms),
      timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}


