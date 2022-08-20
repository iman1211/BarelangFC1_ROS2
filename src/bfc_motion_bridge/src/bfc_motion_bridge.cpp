#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bfc_msgs/msg/head_movement.hpp"
#include "bfc_msgs/msg/button.hpp"
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#define PORT 3838
#define BUFLEN 4096
#define BUFSIZE 1024

#define PORTM 5000 // motion
#define PORTH 5001 // head
#define HOST "localhost"

// Socket LUA head&motion
int sockmotion;
int sockhead;
struct sockaddr_in addrmotion;
struct sockaddr_in addrhead;
struct hostent *localserver;
char dataMotion[20];
char dataHead[20];
char line[10];

void initSendDataHead()
{
    sockhead = socket(AF_INET, SOCK_DGRAM, 0);
    localserver = gethostbyname(HOST);
    bzero((char *)&addrhead, sizeof(addrhead));
    addrhead.sin_family = AF_INET;
    addrhead.sin_port = htons(PORTH);
}

void initSendDataMotion()
{
    sockmotion = socket(AF_INET, SOCK_DGRAM, 0);
    localserver = gethostbyname(HOST);
    bzero((char *)&addrmotion, sizeof(addrmotion));
    addrmotion.sin_family = AF_INET;
    addrmotion.sin_port = htons(PORTM);
}

void runLuaProgram()
{
    // system("killall screen");
    system("cd /home/nvidia-tegra/BarelangFC5-Nasional/Player/;screen -S dcm lua run_dcm.lua;screen -S player lua walk_server.lua;");
    // system("ls");
}

class motion_bridge : public rclcpp::Node
{
public:
    motion_bridge();
    void walkCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
    void headCommand(const bfc_msgs::msg::HeadMovement::SharedPtr msg);
    void motionCommand(const std_msgs::msg::String::SharedPtr msg);
    void motion(char line[2]);
    double Walk(double x, double y, double a);
    void headMove(double pan, double tilt);
    void publishSensor();
    void getSensor();

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr walk_command_;
    rclcpp::Subscription<bfc_msgs::msg::HeadMovement>::SharedPtr head_command_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_command_;
    rclcpp::Publisher<bfc_msgs::msg::Button>::SharedPtr button_state_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    int accelero_x, accelero_y, accelero_z, gyroscope_x, gyroscope_y, gyroscope_z, roll, pitch, yaw, strategy, kill, voltage = 0;
};

motion_bridge::motion_bridge() : Node("motion_bridge")
{
    RCLCPP_INFO(this->get_logger(), "motion_bridge started");
    walk_command_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_walk", 10,
        std::bind(&motion_bridge::walkCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_walk has been started.");

    head_command_ = this->create_subscription<bfc_msgs::msg::HeadMovement>(
        "robot_head", 10,
        std::bind(&motion_bridge::headCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_head has been started.");

    motion_command_ = this->create_subscription<std_msgs::msg::String>(
        "robot_motion", 10,
        std::bind(&motion_bridge::motionCommand, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "robot_motion has been started.");

    button_state_ = this->create_publisher<bfc_msgs::msg::Button>("robot_button", 10);
    imu_state_ = this->create_publisher<sensor_msgs::msg::Imu>("robot_imu", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                            std::bind(&motion_bridge::publishSensor, this));
}

void motion_bridge::walkCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Walk(msg->linear.x, msg->linear.y, msg->linear.z);
}

void motion_bridge::headCommand(const bfc_msgs::msg::HeadMovement::SharedPtr msg)
{
    headMove(msg->pan, msg->tilt);
}

void motion_bridge::motionCommand(const std_msgs::msg::String::SharedPtr msg)
{
    motion(&msg->data[0]);
}

void motion_bridge::publishSensor()
{
    getSensor();
    auto msg_button_ = bfc_msgs::msg::Button();
    msg_button_.strategy = strategy;
    msg_button_.kill = kill;
    button_state_->publish(msg_button_);

    auto msg_imu_ = sensor_msgs::msg::Imu();
    msg_imu_.angular_velocity.x = roll;
    msg_imu_.angular_velocity.y = pitch;
    msg_imu_.angular_velocity.z = yaw;
    imu_state_->publish(msg_imu_);
}

void motion_bridge::motion(char line[2])
{
    char awalan[50];
    strcpy(awalan, "motion");
    sprintf(dataMotion, "%s,%s", awalan, line);
    sendto(sockmotion, dataMotion, strlen(dataMotion), 0, (struct sockaddr *)&addrmotion, sizeof(addrmotion));
    printf("  data motion = %s,%s,%s\n", dataMotion, awalan, line);
}

double motion_bridge::Walk(double x, double y, double a)
{
    char line[50];

    strcpy(line, "walk");
    sprintf(dataMotion, "%s,%.2f,%.2f,%.2f", line, x, y, a);
    sendto(sockmotion, dataMotion, strlen(dataMotion), 0, (struct sockaddr *)&addrmotion, sizeof(addrmotion));
    printf("  data walk = %s\n", dataMotion);
}

void motion_bridge::headMove(double pan, double tilt)
{
    sprintf(dataHead, "%.2f,%.2f", pan, tilt);
    sendto(sockhead, dataHead, strlen(dataHead), 0, (struct sockaddr *)&addrhead, sizeof(addrhead));
    printf("  data head = %s\n", dataHead);
}

int countParse = 0;
void motion_bridge::getSensor()
{
    char line[100];
	char * spr;

	FILE *outputfp;
	outputfp = fopen("/home/nvidia-tegra/BarelangFC5-Nasional/Player/SensorNote", "r");
	fscanf(outputfp, "%s", &line);
	countParse = 0;
	spr = strtok (line, ";");
	while (spr != NULL) {
		if (countParse == 0)	  { sscanf(spr, "%d", &accelero_x); } //x					#dpn-blkng
		else if (countParse == 1) { sscanf(spr, "%d", &accelero_y); } //y					#kiri-kanan
		else if (countParse == 2) { sscanf(spr, "%d", &accelero_z); } //z
		else if (countParse == 3) { sscanf(spr, "%d", &gyroscope_x); } //				#kiri-kanan
		else if (countParse == 4) { sscanf(spr, "%d", &gyroscope_y); } //				#dpn-blkng
		else if (countParse == 5) { sscanf(spr, "%d", &gyroscope_z); } //
		else if (countParse == 6) { sscanf(spr, "%d", &roll); } //roll
		else if (countParse == 7) { sscanf(spr, "%d", &pitch); } //pitch
		else if (countParse == 8) { sscanf(spr, "%d", &yaw); } //yaw
		else if (countParse == 9) { sscanf(spr, "%d", &strategy); } //
		else if (countParse == 10) { sscanf(spr, "%d", &kill); }
		else if (countParse == 11) { sscanf(spr, "%d", &voltage); //countParse = -1; 
		} //
		spr = strtok (NULL,";");
		countParse++;
	}
	fclose(outputfp);
    /* printf("  countParse = %d\n", countParse);
	printf("  accr(%d, %d, %d)", accelero_x, accelero_y, accelero_z);
	printf("  gyro(%d, %d, %d)", gyroscope_x, gyroscope_y, gyroscope_z);
	printf("  angle(%d, %d, %d)\n", roll, pitch, yaw);
	printf("  strategy,killNrun(%d, %d)\n", strategy, kill); */
}

int main(int argc, char **argv)
{
    initSendDataHead();
    initSendDataMotion();
    runLuaProgram();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motion_bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    system("killall -9 screen;");
    return 0;
}