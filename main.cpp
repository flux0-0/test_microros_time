#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int64.h>
#include <rmw_microros/rmw_microros.h>

#include <stdio.h>
#include <unistd.h>

rcl_publisher_t publisher;
std_msgs__msg__Int64 msg;


#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); return 1;}}



// Callback 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    (void) timer;

    // Đồng bộ thời gian với Agent
    RCSOFTCHECK(rmw_uros_sync_session(1000));

    // Lấy thời gian Epoch tính bằng nanoseconds
    int64_t time_ns = rmw_uros_epoch_nanos();

    // Chuyển đổi thời gian Epoch thành định dạng chuẩn ROS 2
    msg.data = time_ns;

    // Xuất bản thông điệp
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    // In thời gian lên console
    printf("UNIX time: %ld nanoseconds\n", time_ns);
}

int main()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

    // Tạo publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "epoch_time_ns"));

    // Tạo timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000; // 1 giây
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Tạo executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // spin
    rclc_executor_spin(&executor);

    // clean
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}
