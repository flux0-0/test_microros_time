#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "motor.h"
#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <tuple>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Sync timeout
const int timeout_ms = 1000;

unsigned long long time_offset = 0;

bool syncTime()
{
    const int timeout_ms = 2000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}


rcl_publisher_t encoder_publisher;
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;

std_msgs__msg__Int32MultiArray encoder_data;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Các tham số cấu hình cho robot
const float wheel_diameter = 0.064;     // diameter
const float wheel_base = 0.185;         // distance of 2 wheels
const float encoder_resolution = 530; // Số xung của JGA25- 130RPM 

float x = 0.0;    // Toạ độ x của robot
float y = 0.0;    // Toạ độ y của robot
float theta = 0.0; // Góc quay của robot

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    int linear_speed = msg->linear.x * 255;
    int angular_speed = msg->angular.z * 255;

    if (linear_speed > 0) {
        forward(linear_speed);
    } else if (linear_speed < 0) {
        reverse(abs(linear_speed));
    } else if (angular_speed > 0) {
        right(angular_speed);
    } else if (angular_speed < 0) {
        left(abs(angular_speed));
    } else {
        stop();
    }

}

void setup() {
    motor_setup();
    //
    IPAddress agent_ip(192, 168, 100, 163);
    size_t agent_port = 8888;
    char ssid[] = "P402";
    char psk[] = "hocmakhongchoi";

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection failed.");
        return;
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);
    
    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_data"
    );

    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );

    encoder_data.data.size = 4;
    encoder_data.data.capacity = 4;
    encoder_data.data.data = (int32_t*) malloc(encoder_data.data.capacity * sizeof(int32_t));

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
}

// Khai báo các biến lưu giá trị encoder trước đó
   static int previous_left_encoder = 0;
   static int previous_right_encoder = 0;

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    auto encoder_values = read_encoder();
    encoder_data.data.data[0] = std::get<0>(encoder_values);  // encoder left count 
    encoder_data.data.data[1] = std::get<1>(encoder_values);  // angle left
    encoder_data.data.data[2] = std::get<2>(encoder_values);  // encoder right count 
    encoder_data.data.data[3] = std::get<3>(encoder_values);  // angle right 

    rcl_publish(&encoder_publisher, &encoder_data, NULL);

   // Tính số xung thay đổi (delta encoder) cho mỗi bánh xe
   int delta_left_encoder = encoder_data.data.data[0] - previous_left_encoder;
   int delta_right_encoder = encoder_data.data.data[2] - previous_right_encoder;

   // Cập nhật giá trị encoder trước đó
   previous_left_encoder = encoder_data.data.data[0];
   previous_right_encoder = encoder_data.data.data[2];

   // Tính quãng đường di chuyển từ số xung encoder
   float left_distance = (delta_left_encoder * (PI * wheel_diameter)) / encoder_resolution;
   float right_distance = (delta_right_encoder * (PI * wheel_diameter)) / encoder_resolution;
   float distance = (left_distance + right_distance) / 2.0;

   
   float angle_change = (right_distance - left_distance) / wheel_base; //góc quay 

   // Cập nhật vị trí và góc quay
   float dt = 0.1; // Time giữa các lần đo = 100ms
   x += distance * cos(theta + angle_change / 2.0);
   y += distance * sin(theta + angle_change / 2.0);
   theta += angle_change;


    struct timespec time_stamp = getTime();


    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    // Cập nhật vận tốc (twist)

    rcl_publish(&odom_publisher, &odom_msg, NULL);
    delay(100);
}
