#include "camera/stopline_func.cpp"
const std::string CAM = "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_964D8E9E-video-index0";

StopLine::StopLine()
: Node("find_stop_line")
{
    distance_pub_ = this->create_publisher<std_msgs::msg::Int16>("/Vision/stopline",rclcpp::SensorDataQoS());
    cap_.open(CAM);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    if (!cap_.isOpened())
    {
    	RCLCPP_ERROR(this->get_logger(), "카메라 안 켜짐!!");
    	RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        return;
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StopLine::image_processing, this)); 
}

void StopLine::on_mouse(int event, int x, int y, int flags, void* userdata) 
{
    if (event == cv::EVENT_MOUSEMOVE) {
        cv::Mat* image = static_cast<cv::Mat*>(userdata);

        //마우스 위치 좌표를 이미지에 표시
        cv::putText(*image, "(" + std::to_string(x) + ", " + std::to_string(y) + ")",
                    cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        cv::imshow("Image", *image);
    }
}

void StopLine::image_processing()
{
    cv::Mat frame;
    cap_ >> frame;

    // sub 이미지 리사이즈
    cv::Mat resized_img;
    cv::resize(frame, resized_img, cv::Size(640, 480), 0, 0);

    // 복사본 생성
    cv::Mat resized_img_clone;
    resized_img_clone = resized_img.clone();

    // 추가 변수 생성
    cv::Mat bird_eye_viewed;
    cv::Mat bird_eye_viewed_clone;

    // 버드아이뷰 이미지 변환
    if(StopLine::first_run == 1)
    {
        bird_eye_viewed = StopLine::to_bird_eye_view(resized_img_clone);
        StopLine::first_run = 0;
    }
    else
    {
        // 변환 매트릭스 타입 확인
        if (StopLine::warp_matrix.type() != CV_32F && StopLine::warp_matrix.type() != CV_64F)
        {
            std::cout << "Converting matrix type to CV_32F" << std::endl;
            StopLine::warp_matrix.convertTo(StopLine::warp_matrix, CV_32F);
        }

        cv::warpPerspective(resized_img_clone, bird_eye_viewed, StopLine::warp_matrix, cv::Size(resized_img_clone.cols, resized_img_clone.rows));
    }


    // 버드아이뷰 이미지 복사
    bird_eye_viewed_clone = bird_eye_viewed.clone();

    // 원본이미지 사이즈 재변경
    cv::Mat re_resized_img;
    cv::resize(resized_img_clone, re_resized_img, cv::Size(641, 481), 0, 0);

    cv::Mat find_line_result = find_line(bird_eye_viewed);
    cv::resize(find_line_result, find_line_result, re_resized_img.size());

    cv::Mat final;
    addWeighted(find_line_result, 1, re_resized_img, 1, 0, final);
    StopLine::is_stop_ = show_return_distance(final);
    imshow("final", final);
    cv::waitKey(1);

    cv::setMouseCallback("final", on_mouse, &final);

    std_msgs::msg::Int16 msg;
    msg.data = StopLine::is_stop_;
    distance_pub_->publish(msg);
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopLine>());
    rclcpp::shutdown();
    return 0;
}
