#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>


class StopLine : public rclcpp::Node
{
	public:
		StopLine();

	private:
		void image_processing();
		cv::Mat to_bird_eye_view(const cv::Mat& raw_img); // 이 함수를 통해 리턴된 객체 => bird_eye_viewd_img
		cv::Mat find_line(const cv::Mat& bird_eye_viewed_img);
		cv::Mat filterImg(const cv::Mat& imgUnwarp, const int toColorChannel, const int mode);	//지연
		cv::Mat make_zeros(const cv::Mat& img);	//지연
		cv::Mat mask_filter(const cv::Mat& integraled_img, const int mask_width, const int mask_height, \
		const int thresh);
		cv::Mat hsl_binarization(const cv::Mat& bird_eye_viewed_img_with_white);
		cv::Scalar get_img_mean(const cv::Mat& bird_eye_viewed_img_with_white, const int img_top_left_x,\
		const int img_width, const int img_top_left_y, const int img_height);
		int show_return_distance(const cv::Mat& processed_img);
		static void on_mouse(int event, int x, int y, int flags, void* userdata);


		int is_stop_ = 100; // 100으로 초기화 시키기 (거리 확인 출력용 변수명 변경 권장 -> distance)
		cv::Mat warp_matrix;
		int first_run = 1;
		cv::VideoCapture cap_;

		rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr distance_pub_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
		rclcpp::TimerBase::SharedPtr timer_; // 호출 주기

};