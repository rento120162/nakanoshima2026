#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int openSerial(const char *device_name){
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0) return -1;

    fcntl(fd, F_SETFL, 0);
    struct termios tio{};
    tcgetattr(fd, &tio);

    speed_t BAUDRATE = B9600;
    cfsetispeed(&tio, BAUDRATE);
    cfsetospeed(&tio, BAUDRATE);

    tio.c_lflag &= ~(ECHO | ICANON);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tio);
    return fd;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("serial_receive_node");

    auto serial_pub =
        node->create_publisher<std_msgs::msg::Bool>("/emergency_cmd", 10);

    int fd = openSerial("/dev/ttyUSB0");
    if(fd < 0){
        RCLCPP_ERROR(node->get_logger(), "Serial open failed");
        return -1;
    }

    rclcpp::WallRate loop_rate(20);

    while(rclcpp::ok()){
        char buf[32] = {0};

        int n = read(fd, buf, sizeof(buf)-1);
        if(n > 0){
            std::string recv(buf);

            std_msgs::msg::Bool msg;

            if(recv.find("1") != std::string::npos){
                msg.data = true;
            }else if(recv.find("0") != std::string::npos){
                msg.data = false;
            }else{
                continue; 
            }

            serial_pub->publish(msg);

            //RCLCPP_INFO(node->get_logger(), "recv: %s -> %d", recv.c_str(), msg.data);
        }

        loop_rate.sleep();
    }

    close(fd);
    rclcpp::shutdown();
    return 0;
}