#include "robot_base.h"


// For sleep duration
using namespace std::chrono_literals;

double odom_variance[6] = {0.02213, 0.07221, 0.01221, 0.001, 0.001, 1e-6};
float temp_keyboard = 0;

namespace robomaster {
Robot::Robot(std::string device_path)
    : Node("robot_base"), device_path_(device_path), tf_broadcaster_(this) {
    if (!(ROSInit() && CommInit())) {
        rclcpp::shutdown();
    };
}

Robot::~Robot() {
    if (recv_thread_ && recv_thread_->joinable()) {
        recv_thread_->join();
    }
}


void Robot::ChassisCtrlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    chassis_ctrl_info_.vx = msg->linear.x;
    chassis_ctrl_info_.vy = msg->linear.y;
    chassis_ctrl_info_.vw = msg->angular.z * 1.5;
    RCLCPP_INFO(this->get_logger(),"vx: %f, vy: %f, vw: %f", chassis_ctrl_info_.vx, chassis_ctrl_info_.vy, chassis_ctrl_info_.vw);

    uint16_t send_length = SenderPackSolve((uint8_t *)&chassis_ctrl_info_, sizeof(chassis_ctrl_info_t),
                                           CHASSIS_CTRL_CMD_ID, send_buff_.get());
    device_ptr_->Write(send_buff_.get(), send_length);
}

bool Robot::ROSInit() {
    chassis_ctrl_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&Robot::ChassisCtrlCallback, this, std::placeholders::_1));

    chassis_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

    clock  = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);


    // Set frame IDs
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_footprint";
    
    return true;
}

bool Robot::CommInit() {
    this->declare_parameter("port", device_path_);
    int baud = 115200;
    this->declare_parameter("baud", baud);

    device_ptr_ = std::make_shared<SerialDevice>(device_path_, baud);
    if (!device_ptr_->Init()) {
        return false;
    }

    recv_buff_ = std::make_unique<uint8_t[]>(BUFF_LENGTH);
    send_buff_ = std::make_unique<uint8_t[]>(BUFF_LENGTH);

    memset(&frame_receive_header_, 0, sizeof(frame_header_struct_t));
    memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));

    // Start receiving thread
    recv_thread_ = std::make_shared<std::thread>(&Robot::RecvThread, this);

    return true;
}

void Robot::RecvThread() {
    int a = 0;
    int flag = 0;
    uint8_t last_len = 0;

    while (rclcpp::ok()) {
        last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, 128);

        while (flag == 0 && last_len == 1) {
            if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF)) {
                flag = 1;
                SearchFrameSOF(recv_buff_.get(), a);
            }
            a++;
        }
        flag = 0;
        a = 0;

        std::this_thread::sleep_for(1ms);
    }
}

void Robot::SearchFrameSOF(uint8_t *frame, uint16_t total_len) {
    uint16_t i;
    uint16_t index = 0;
    int a = 0;

    for (i = 0; i < total_len;) {
        if (*frame == HEADER_SOF) {
            ReceiveDataSolve(frame);
            i = total_len;
        } else {
            frame++;
            i++;
        }
    }
}

uint16_t Robot::ReceiveDataSolve(uint8_t *frame) {
    uint8_t index = 0;
    uint16_t cmd_id = 0;

    if (*frame != HEADER_SOF) {
        return 0;
    }

    memcpy(&frame_receive_header_, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    if ((!Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t))) || (!Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9))) {
        RCLCPP_ERROR(this->get_logger(), "CRC ERROR!");
        return 0;
    } else {
        memcpy(&cmd_id, frame + index, sizeof(uint16_t));
        index += sizeof(uint16_t);

        switch (cmd_id) {
            case CHASSIS_ODOM_FDB_ID: {
               RCLCPP_INFO(this->get_logger(),"get odom msg from vcom");
                memcpy(&chassis_odom_info_, frame + index, sizeof(chassis_odom_info_t));
                current_time = clock->now();
                odom_.header.stamp = current_time;
                odom_.pose.pose.position.z = 0.0;
                odom_.twist.twist.linear.x = chassis_odom_info_.vx_fdb;
                odom_.twist.twist.linear.y = chassis_odom_info_.vy_fdb;
                odom_.twist.twist.angular.z = chassis_odom_info_.vw_fdb;

                double dt = (current_time - last_time).seconds();
                x += (chassis_odom_info_.vx_fdb * cos(yaw) + chassis_odom_info_.vy_fdb * sin(yaw)) * dt;
                y += (-chassis_odom_info_.vx_fdb * sin(yaw) + chassis_odom_info_.vy_fdb * cos(yaw)) * dt;
                double delta_th = chassis_odom_info_.vw_fdb * dt;
                th += delta_th;

                odom_.header.stamp = current_time;
                tf2::Quaternion q;
                q.setRPY(0, 0, th);
                geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);
                odom_.pose.pose.position.x = x;
                odom_.pose.pose.position.y = y;
                odom_.pose.pose.position.z = 0.0;
                odom_.pose.pose.orientation = odom_quat;
                for (unsigned int i = 0; i < 6; i++)
                    odom_.pose.covariance[7 * i] = pow(odom_variance[i], 2);

                chassis_odom_pub_->publish(odom_);

                odom_tf_.header.stamp = current_time;
                odom_tf_.transform.translation.x = x;
                odom_tf_.transform.translation.y = y;
                odom_tf_.transform.translation.z = 0.0;
                odom_tf_.transform.rotation = odom_quat;

                tf_broadcaster_.sendTransform(odom_tf_);

                last_time = current_time;

                break;
            }
            default:
                break;
        }
        index += frame_receive_header_.data_length + 2;
        return index;
    }
}

uint16_t Robot::SenderPackSolve(uint8_t *data, uint16_t data_length, uint16_t cmd_id, uint8_t *send_buf) {
    uint8_t index = 0;
    frame_send_header_.SOF = HEADER_SOF;
    frame_send_header_.data_length = data_length;
    frame_send_header_.seq++;

    Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(frame_header_struct_t));
    memcpy(send_buf, &frame_send_header_, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
    memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);
    memcpy(send_buf + index, data, data_length);
    Append_CRC16_Check_Sum(send_buf, data_length + 9);

    return data_length + 9;
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robomaster::Robot>("/dev/robomaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
