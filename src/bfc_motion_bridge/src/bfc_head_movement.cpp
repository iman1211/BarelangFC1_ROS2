#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bfc_msgs/msg/head_movement.hpp"
#include "bfc_msgs/msg/button.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"

#define	PI	3.1415926535897932384626433832795

using namespace std;

class head_movement : public rclcpp::Node
{
public:
    head_movement();
    void publishHead();
    void objCoor(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg);
    int ballLost(int threshold);
    void tiltSearchBall(double temphead_pan_);
    void panSearchBall(double temphead_tilt_);
    void searchBallRectang(double atas, double kanan, double bawah, double kiri);
    void trackBall();
    void barelang_head();
    void display();

private:
    int robot_number_, frame_x_, frame_y_, ball_x_, ball_y_, goal_x_, goal_y_ = 0;
    double ball_pan_kp_, ball_pan_kd_, ball_tilt_kp_, ball_tilt_kd_, head_pan_, head_tilt_ = 0.0;
    bool tracked_ = false;

    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bounding_boxes_;
    rclcpp::Publisher<bfc_msgs::msg::HeadMovement>::SharedPtr head_move_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr robot_id_;
    rclcpp::TimerBase::SharedPtr timer_;
};

head_movement::head_movement() : Node("head_movement")
{
    this->declare_parameter("robot_number_", 1);
    this->declare_parameter("frame_x_", 640);
    this->declare_parameter("frame_y_", 480);
    this->declare_parameter("ball_pan_kp_", 0.075);
    this->declare_parameter("ball_pan_kd_", 0.0000505);
    this->declare_parameter("ball_tilt_kp_", 0.05);
    this->declare_parameter("ball_tilt_kd_", 0.0000755);

    robot_number_ = this->get_parameter("robot_number_").as_int();
    frame_x_ = this->get_parameter("frame_x_").as_int();
    frame_y_ = this->get_parameter("frame_y_").as_int();
    ball_pan_kp_ = this->get_parameter("ball_pan_kp_").as_double();
    ball_pan_kd_ = this->get_parameter("ball_pan_kd_").as_double();
    ball_tilt_kp_ = this->get_parameter("ball_tilt_kp_").as_double();
    ball_tilt_kd_ = this->get_parameter("ball_tilt_kd_").as_double();

    bounding_boxes_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>("darknet_ros/bounding_boxes", 10,
                                                                                      std::bind(&head_movement::objCoor, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "darknet_ros/bounding_boxes has been started.");

    head_move_ = this->create_publisher<bfc_msgs::msg::HeadMovement>("robot_head", 10);
    robot_id_ = this->create_publisher<std_msgs::msg::Int16>("robot_id", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     std::bind(&head_movement::publishHead, this));
}

void head_movement::publishHead()
{
    barelang_head();
    auto msg_head_ = bfc_msgs::msg::HeadMovement();
    msg_head_.pan = head_pan_;
    msg_head_.tilt = head_tilt_;
    head_move_->publish(msg_head_);

    auto msg_id_ = std_msgs::msg::Int16();
    msg_id_.data = robot_number_;
    robot_id_->publish(msg_id_);
}

int cnt_lost_ball_, cnt_lost_goal_ = 0;
void head_movement::objCoor(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg_bbox_)
{
    for (size_t i = 0; i < msg_bbox_->bounding_boxes.size(); i++)
    {
        if (msg_bbox_->bounding_boxes[i].class_id == "ball")
        {
            ball_x_ = (msg_bbox_->bounding_boxes[i].xmin + msg_bbox_->bounding_boxes[i].xmax) / 2;
            ball_y_ = (msg_bbox_->bounding_boxes[i].ymin + msg_bbox_->bounding_boxes[i].ymax) / 2;
            cnt_lost_ball_ = 0;
        }
        else
        {
            if (cnt_lost_ball_ > 10)
            {
                ball_x_ = ball_y_ = -1;
            }
            else
            {
                cnt_lost_ball_++;
            }
        }

        if (msg_bbox_->bounding_boxes[i].class_id == "goal")
        {
            goal_x_ = (msg_bbox_->bounding_boxes[i].xmin + msg_bbox_->bounding_boxes[i].xmax) / 2;
            goal_y_ = (msg_bbox_->bounding_boxes[i].ymin + msg_bbox_->bounding_boxes[i].ymax) / 2;
            cnt_lost_goal_ = 0;
        }
        else
        {
            if (cnt_lost_goal_ > 10)
            {
                goal_x_ = goal_y_ = -1;
            }
            else
            {
                cnt_lost_goal_++;
            }
        }
    }
}

// Checking Lost Ball ========================================================================
int countBallLost = 0,
    countBallFound = 0,
    returnBallVal;
int head_movement::ballLost(int threshold)
{
    if (ball_x_ == -1 && ball_y_ == -1)
    {
        countBallFound = 0;
        countBallLost++;
        if (countBallLost >= threshold)
        {
            returnBallVal = 1;
        }
    }
    else
    {
        countBallLost = 0;
        countBallFound++;
        if (countBallFound > 1)
        {
            returnBallVal = 0;
        }
    }
    return returnBallVal;
}

// Search ball ===============================================================================
double tiltRate = -0.05, //-0.03,//-0.075
    panRate = -0.05,     //-0.04,//-0.075

    tiltRate1 = -0.06,
       panRate1 = -0.06,

       tiltRate2 = -0.08,
       panRate2 = -0.08,

       tiltRate3 = -0.1,
       panRate3 = -0.1,

       tiltRate4 = -0.07,
       panRate4 = -0.07,

       searchKe = 0,

       batasKanan = -1.6,
       batasKiri = 1.6,
       batasAtas = -2.0,
       batasBawah = -0.6;

void head_movement::tiltSearchBall(double temphead_pan_)
{ // printf("  tiltSearchBall\n\n");
    head_pan_ = temphead_pan_;
    head_tilt_ += tiltRate1;

    if (head_tilt_ <= batasAtas || head_tilt_ >= batasBawah)
    {
        tiltRate1 *= -1;
    }

    if (head_tilt_ <= (-1.2 + tiltRate) && head_tilt_ >= (-1.2 - tiltRate))
    {
        // searchKe += 0.5;
    }

    if (head_tilt_ <= batasAtas)
    {
        head_tilt_ = batasAtas;
    }
    else if (head_tilt_ >= batasBawah)
    {
        head_tilt_ = batasBawah;
    }
}

void head_movement::panSearchBall(double temphead_tilt_)
{ // printf("  panSearchBall\n\n");
    head_tilt_ = temphead_tilt_;
    head_pan_ += panRate2;

    if (head_pan_ <= batasKanan || head_pan_ >= batasKiri)
    {
        panRate2 *= -1;
    }

    if (head_pan_ <= (0.0 + panRate) && head_pan_ >= (0.0 - panRate))
    {
        // searchKe += 0.5;
    }

    if (head_pan_ >= batasKiri)
    {
        head_pan_ = batasKiri;
    }
    else if (head_pan_ <= batasKanan)
    {
        head_pan_ = batasKanan;
    }
}

bool neckX;
void head_movement::searchBallRectang(double atas, double kanan, double bawah, double kiri)
{
    if (neckX)
    {
        head_pan_ += panRate2;
        if (head_pan_ >= kiri || head_pan_ <= kanan)
        {
            panRate2 *= -1;
            neckX = false;
        }
    }
    else
    {
        head_tilt_ += tiltRate1;
        if (head_tilt_ <= atas || head_tilt_ >= bawah)
        {
            tiltRate1 *= -1;
            neckX = true;
        }
    }

    if (head_pan_ >= kiri)
    {
        head_pan_ = kiri;
    }
    else if (head_pan_ <= kanan)
    {
        head_pan_ = kanan;
    }
    if (head_tilt_ <= atas)
    {
        head_tilt_ = atas;
    }
    else if (head_tilt_ >= bawah)
    {
        head_tilt_ = bawah;
    }
}

// Ball Tracking =============================================================================
double intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
       PPanB = 0, IPanB = 0, DPanB = 0,
       intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
       PTiltB = 0, ITiltB = 0, DTiltB = 0,
       dtB = 0.04;
double B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
    pOffsetB, iOffsetB, dOffsetB,
    errorPanBRad, errorTiltBRad,
    offsetSetPointBall;
void head_movement::trackBall()
{
    if (ball_x_ != -1 && ball_y_ != -1)
    {
        errorPanB = (double)ball_x_ - (frame_x_ / 2); // 160
        errorTiltB = (double)ball_y_ - (frame_y_ / 2); // 120
        errorPanB *= -1;
        errorTiltB *= -1;
        errorPanB *= (90 / (double)frame_x_);  // pixel per angle
        errorTiltB *= (60 / (double)frame_y_); // pixel per angle
        // errorPanB *= (77.32 / (double)frame_x_); // pixel per angle
        // errorTiltB *= (61.93 / (double)frame_y_); // pixel per angle

        errorPanBRad = (errorPanB * PI) / 180;
        errorTiltBRad = (errorTiltB * PI) / 180;
        // printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
        // printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
        // printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

        B_Pan_err_diff = errorPanBRad - B_Pan_err;
        B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

        // PID pan ==========================================================
        // PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
        PPanB = B_Pan_err * ball_pan_kp_; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
        intPanB += B_Pan_err * dtB;
        IPanB = intPanB * 0.0;
        dervPanB = B_Pan_err_diff / dtB;
        // DPanB = dervPanB * kamera.panKD;
        DPanB = dervPanB * ball_pan_kd_;
        B_Pan_err = errorPanBRad;
        head_pan_ += (PPanB + IPanB + DPanB);

        // PID tilt ==========================================================
        // PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
        PTiltB = B_Tilt_err * ball_tilt_kp_; // Tune in Kp Tilt 0.00030

        intTiltB += B_Tilt_err * dtB;
        ITiltB = intTiltB * 0.0;

        dervTiltB = B_Tilt_err_diff / dtB;
        // DTiltB = dervTiltB * kamera.tiltKD;
        DTiltB = dervTiltB * ball_tilt_kd_;

        preErrTiltB = errorTiltB;
        B_Tilt_err = errorTiltBRad;
        head_tilt_ += (PTiltB + ITiltB + DTiltB) * -1;

        if (head_pan_ >= 1.6)
        {
            head_pan_ = 1.6;
        }
        else if (head_pan_ <= -1.6)
        {
            head_pan_ = -1.6;
        }
        if (head_tilt_ <= -2.0)
        {
            head_tilt_ = -2.0;
        }
        else if (head_tilt_ >= -0.4)
        {
            head_tilt_ = -0.4;
        }
    }
}

void head_movement::barelang_head()
{
    display();

    if (ballLost(20))
    {
        tiltSearchBall(0.0);
        tracked_ = false;
    } else {
        trackBall();
        tracked_ = true;
    }
}

void head_movement::display()
{
    cout << "robot_number_ = " << robot_number_ << endl;
    cout << "ball_x_ = " << ball_x_ << ", ball_y_ = " << ball_y_ << endl;
    cout << "goal_x_ = " << goal_x_ << ", goal_y_ = " << goal_y_ << endl;
    cout << "head_pan_ = " << ceil(head_pan_ * 100.0) / 100.0 << ", head_tilt_ = " << ceil(head_tilt_ * 100.0) / 100.0 << endl;

    cout << "\033[2J\033[1;1H";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<head_movement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}