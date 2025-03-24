#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <string>

// Global variables
std::string packet_data, packet_data2, packet_data3;
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void switch_callback(const std_msgs::String::ConstPtr& msg) {
    packet_data = msg->data;
}

void switch_callback2(const std_msgs::String::ConstPtr& msg) {
    packet_data2 = msg->data;
}

void switch_callback3(const std_msgs::String::ConstPtr& msg) {
    packet_data3 = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub1 = nh.subscribe("/write", 1000, write_callback);
    ros::Subscriber write_sub2 = nh.subscribe("/packet", 1000, switch_callback);
    ros::Subscriber write_sub3 = nh.subscribe("/packet2", 1000, switch_callback2);
    ros::Subscriber write_sub4 = nh.subscribe("/packet3", 1000, switch_callback3);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read_scara", 1000);

    try {
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(1000);
    // ros::Rate packet3_rate(1); // 1 Hz rate for packet3

    while (ros::ok()) {
        ros::spinOnce();

        // Send the first packet and wait for a response
        if (!packet_data.empty()) {
            ser.write(packet_data);
            packet_data.clear(); // Reset packet

            // Wait for a response for the first packet
            ros::Time start_time = ros::Time::now();
            while ((ros::Time::now() - start_time).toSec() < 0.1) { // Timeout after 1 second
                if (ser.available()) {
                    ROS_INFO_STREAM("Reading response for packet 1 from serial port");
                    std_msgs::String result;
                    result.data = "1" + ser.read(ser.available());
                    ROS_INFO_STREAM("Read: " << result.data);
                    read_pub.publish(result);
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        // Send the second packet and wait for a response
        if (!packet_data2.empty()) {
            ser.write(packet_data2);
            packet_data2.clear(); // Reset packet

            // Wait for a response for the second packet
            ros::Time start_time = ros::Time::now();
            while ((ros::Time::now() - start_time).toSec() < 0.1) { // Timeout after 1 second
                if (ser.available()) {
                    ROS_INFO_STREAM("Reading response for packet 2 from serial port");
                    std_msgs::String result;
                    result.data = "2" + ser.read(ser.available());
                    ROS_INFO_STREAM("Read: " << result.data);
                    read_pub.publish(result);
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        // Send the third packet and wait for a response
        if (!packet_data3.empty()) {
            // packet3_rate.sleep(); // Ensure the packet3 is sent only once per second
            ser.write(packet_data3);
            packet_data3.clear(); // Reset packet

            // Wait for a response for the third packet
            ros::Time start_time = ros::Time::now();
            while ((ros::Time::now() - start_time).toSec() < 0.1) { // Timeout after 1 second
                if (ser.available()) {
                    ROS_INFO_STREAM("Reading response for packet 3 from serial port");
                    std_msgs::String result;
                    result.data = "3" + ser.read(ser.available());
                    ROS_INFO_STREAM("Read: " << result.data);
                    read_pub.publish(result);
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        loop_rate.sleep();
    }
}
