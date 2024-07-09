#include <camera/by_id_parking.hpp>
#include <vector>
#include <sstream>
#include <string>
const std::string CAM = "dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_E5BC8E9E-video-index0"; // 여기에 왼쪽 카메라 인덱스 넣기
using namespace cv;
//--------------------------------------------------------------------------------------------------
/**
 * @brief 스테레오 카메라 만들 때 사용하는 함수, 초기 제작할 때 수평을 맞추기 위해서 사용함
 * @param frame 입력 영상
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 */
void Parking::line_symmetry(const cv::Mat& frame, const std::string camera)
{
	rectangle(frame, cv::Rect(cv::Point(635,0),cv::Point(645,720)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,680),cv::Point(1280,690)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,640),cv::Point(1280,650)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,600),cv::Point(1280,610)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,560),cv::Point(1280,570)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,520),cv::Point(1280,530)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,480),cv::Point(1280,490)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,360),cv::Point(1280,370)),cv::Scalar(0,255,0),2,4,0);

	if(camera == CAM)
	{
		// imshow("left_line_symmetry", frame);
	}
	else
	{
		// imshow("right_line_symmetry", frame);
	}
}

//--------------------------------------------------------------------------------------------------
/**
 * @brief 왜곡 보정 함수
 * @param frame 입력 영상
 */
cv::Mat Parking::undistort_frame(const cv::Mat& frame)
{
	// 카메라 내부 파라미터
	cv::Mat intrinsic_param = cv::Mat::zeros(3,3,CV_64FC1); // zeros로 테스트 중
	//cv::Mat intrinsic_param = Mat::eye(3,3,CV_64FC1); // eye를 쓴 이유가 뭐지?
	intrinsic_param=(cv::Mat1d(3,3) << 509.5140, 0, 321.9972, 0, 510.5093, 258.7457, 0., 0., 1. );

	// 카메라 왜곡 계수
	cv::Mat distortion_coefficient = cv::Mat::zeros(1,5,CV_64FC1);
	distortion_coefficient=(cv::Mat1d(1,5) << 0.0891, -0.1673, 0., 0., 0.);

	// 새로운 카메라 매개변수 생성
	// newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionParameters, { frame.cols, frame.rows }, 1);

	cv::Mat undistorted_frame;
	cv::undistort(frame, undistorted_frame, intrinsic_param, distortion_coefficient);

	return undistorted_frame;
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief HSV값으로 영상을 이진화하는 함수
 * 
 * @param frame 입력하고자 하는 화면
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return Mat 
 */
cv::Mat Parking::add_hsv_filter(const cv::Mat& frame, const std::string camera) {

	cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
	cv::Mat mask;

	// logitech c930e
	std::vector<int> left_lowerYellow = { 10, 160, 100 };     // Lower limit for yellow
	std::vector<int> left_upperYellow = { 40, 255, 255 };	 // Upper limit for yellow
	std::vector<int> right_lowerYellow = { 10, 160, 100 };     // Lower limit for yellow
	std::vector<int> right_upperYellow = { 40, 255, 255 };	 // Upper limit for yellow

	if(camera == CAM)
	{
		inRange(frame, left_lowerYellow, left_upperYellow, mask);
	}
	else
	{
		inRange(frame, right_lowerYellow, right_upperYellow, mask);
	}
	
	return mask;
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 캘리용 함수
 * 
 * @param frame 입력 영상
 * @param mask 이진화된 영상 (HSV로 이진화하든 어쨌든 이진화된 영상)
 * @return Point 
 */
cv::Point Parking::find_ball(const cv::Mat& frame, const cv::Mat& mask)
{

	std::vector<std::vector<cv::Point> > contours;

	/*
	cv::RETR_EXTERNAL: 가장 외곽의 윤곽선만 검색.
	cv::CHAIN_APPROX_SIMPLE: 윤곽선 압축하여 저장. ex)직선 부분은 끝점만 저장, 곡선 부분은 시작점과 끝점 사이의 중간 점을 저장.
	*/
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	
	// Sort the contours to find the biggest one
	sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
		return contourArea(c1, false) < contourArea(c2, false);
	});

	if (contours.size() > 0) 
	{
		std::vector<cv::Point> largestContour = contours[contours.size() - 1];
		cv::Point2f center;
		float radius;
		
		minEnclosingCircle(largestContour, center, radius);
		// 0623 아래 부분 수정함.
		//cv::Moments m = moments(largestContour);
		//cv::Point centerPoint(m.m10 / m.m00, m.m01 / m.m00);
		cv::Point centerPoint(center.x, center.y);

		// Only preceed if the radius is greater than a minimum threshold
		if (radius > 10) {
			// Draw the circle and centroid on the frame
			circle(frame, center, int(radius), cv::Scalar(0, 255, 255), 2);
			circle(frame, centerPoint, 5, cv::Scalar(0, 0, 255), -1);
		}

		return centerPoint;
	}
	return { 0,0 };
}



//--------------------------------------------------------------------------------------------------
cv::Mat Parking::find_edge(const cv::Mat& frame, const std::string camera) {

	int center_x = frame.cols /2 +20;//-50;// + 20;
	int center_y = frame.rows /2 +10;//-50;// + 10;

	Mat gray_image;
	cvtColor(frame, gray_image, CV_RGB2GRAY);

	// imshow("gray_image", gray_image); 
	GaussianBlur(gray_image,gray_image, Size(5,5),1);
	// imshow("gau", gray_image);
	
	Mat dx, dy;
	Sobel(gray_image, dx, CV_32FC1, 1, 0);
	Sobel(gray_image, dy, CV_32FC1, 0, 1);

	Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);

	Mat edge = mag > 80;

	///////////////////그림자 처리(ex.밝->어두워지는 거)

	cv::dilate(edge, edge, Mat(), Point(-1, -1), 2);
	// erode(edge, edge, Mat(), Point(-1, -1), 1);

	if(camera == CAM)
	{
		imshow("LEFT EDGE ", edge);
	}
	else
	{
		imshow("RIGHT EDGE ", edge);
	}
	cv::waitKey(1);

	return edge;
}


//--------------------------------------------------------------------------------------------------
/**
 * @brief 가로로 긴 사각형인지 확인
 * @param vertices 꼭짓점 좌표
 * @return bool 
 */
bool Parking::isHorizontalPolygon(const std::vector<cv::Point>& points) {
    int maxX = points[0].x;
    int minX = points[0].x;

    for (const auto& point : points) 
	{
        if (point.x > maxX)
            maxX = point.x;
        if (point.x < minX)
            minX = point.x;
    }

    int maxY = points[0].y;
    int minY = points[0].y;

    for (const auto& point : points) 
	{
        if (point.y > maxY)
            maxY = point.y;
        if (point.y < minY)
            minY = point.y;
    }

    return (maxX - minX) > (maxY - minY);
}

//--------------------------------------------------------------------------------------------------
/**
 * @brief 세로로 긴 사각형인지 확인
 * @param vertices 꼭짓점 좌표
 * @return bool 
 */

bool Parking::isVerticalPolygon(const std::vector<cv::Point>& points) {
    int maxX = points[0].x;
    int minX = points[0].x;

    for (const auto& point : points) 
	{
        if (point.x > maxX)
            maxX = point.x;
        if (point.x < minX)
            minX = point.x;
    }

    int maxY = points[0].y;
    int minY = points[0].y;

    for (const auto& point : points) 
	{
        if (point.y > maxY)
            maxY = point.y;
        if (point.y < minY)
            minY = point.y;
    }

    return (maxY - minY) > (maxX - minX);
}

//--------------------------------------------------------------------------------------------------
/**
 * @brief 이진화한 영상에서 중심점을 찾고 하단, 중단, 상단의 x,y좌표를 double 형의 자료형으로 저장함
 * 
 * @param img 입력 영상 (이진화된 영상을 넣으면 됨)
 * @param array double자료형의 배열(바닥 x,y, 가운데 x,y, 상단 x,y 를 저장함)
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return double* 
 */
double* Parking::find_center(const cv::Mat& img, double array[], const std::string camera)
{

	int center_x = 1280 /2 +20;//-50;// + 20;
	int center_y = 720 /2 +10;//-50;// + 10;

	Mat mask = img.clone();
    // contours
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
    double nowarea = 0;

	std::vector<Point2f> approx;

	int min_distance = 1000;

	for (size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		int vertex = approx.size();
		
		Moments m = moments(contours[i], true);
		Point p(m.m10/m.m00, m.m01/m.m00);
		
		std::vector<Point> hull;
        convexHull(Mat(contours[i]), hull, false);

		nowarea = contourArea(Mat(hull));

		// cout << nowarea << endl;

		int now_distance = sqrt(pow(p.x-center_x,2)+pow(p.y-center_y,2));
		if(vertex > 3 && vertex < 12)
		{
			if ((nowarea > 35000) && (nowarea < 300000))		// 일단 25000이었음
			{
				// if(p.y < min_y)
				// {
				// 	min_y = p.y;
				// 	lrgctridx = i;
				// 	// cout << " 넓이 : " << nowarea << endl;
				// } ////////////////////////////////////////////////수정 필요. 중심점 기준으로 컨투어 찾도록.
				if(now_distance < min_distance)
				{
					min_distance = now_distance;
					lrgctridx = i;
					// cout << " 넓이 : " << nowarea << endl;
				} 
			
			}

		}
	}

	// cout << "contour size = " << minarea << endl;

    if(contours.size() > 0)
    {
        Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );

        // 모든 외각선 그리기
        // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        // {
        //     drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        // }

        // 특정한 외각선만 그리기
        drawContours( drawing, contours, lrgctridx, color, 2, LINE_8, hierarchy);

        std::vector<Point> hull;
        convexHull(Mat(contours[lrgctridx]), hull, false);

        std::vector<std::vector<Point>> fake_hull;
        fake_hull.push_back(hull);
        drawContours(drawing, fake_hull, 0, color, 2, LINE_8);

        int top_x_left = hull[0].x;
	    int top_y_left = hull[0].y;
        int top_num_left = 0;

        int bottom_x_left = hull[0].x;
        int bottom_y_left = hull[0].y;
        int bottom_num_left = 0;

        int top_x_right = hull[0].x;
	    int top_y_right = hull[0].y;
        int top_num_right = 0;

        int bottom_x_right = hull[0].x;
        int bottom_y_right = hull[0].y;
        int bottom_num_right = 0;

        for(int i = 0; i < hull.size(); i++)
        {
            // if(hull[i].y < top_y_left)
            // {
            //     top_x_left = hull[i].x;
            //     top_y_left = hull[i].y;
            //     top_num_left = i;
            // }
            if(sqrt(pow(hull[i].x - img.cols*7/8, 2) + pow(hull[i].y - 0, 2)) < sqrt(pow(top_x_right - img.cols*7/8, 2) + pow(top_y_right - 0, 2)))
            {
                top_x_right = hull[i].x;
                top_y_right = hull[i].y;
                top_num_right = i;
            }
            if((1*sqrt(pow(hull[i].x - 0, 2) + pow(hull[i].y - img.rows, 2)) + 9*hull[i].x) < (1*sqrt(pow(bottom_x_left - 0, 2) + pow(bottom_y_left - img.rows, 2)) + 9*bottom_x_left))
            {
                bottom_x_left = hull[i].x;
                bottom_y_left = hull[i].y;
                bottom_num_left = i;
            }
            if(sqrt(pow(hull[i].x - img.cols, 2) + pow(hull[i].y - img.rows, 2)) < sqrt(pow(bottom_x_right - img.cols, 2) + pow(bottom_y_right - img.rows, 2)))
            {
                bottom_x_right = hull[i].x;
                bottom_y_right = hull[i].y;
                bottom_num_right = i;
            }
        }

		double daegaksun = sqrt(pow(bottom_x_left - top_x_right, 2) + pow(bottom_y_left - top_y_right, 2));
		double long_sin = 0;
		double fake_sin;

		for(int j=0; j < hull.size(); j++)
		{
			if((hull[j].y < bottom_y_left -50) && (hull[j].x < top_x_right - 50) && (hull[j].x < bottom_x_right - 50))
			{
				double sasun1 = sqrt(pow(hull[j].x - bottom_x_left, 2) + pow(hull[j].y - bottom_y_left, 2));
				double sasun2 = sqrt(pow(hull[j].x - top_x_right, 2) + pow(hull[j].y - top_y_right, 2));

				double theta = acos((pow(sasun1, 2) - pow(sasun2, 2) + pow(daegaksun, 2)) / (2*sasun1*daegaksun));

				fake_sin = sasun1 * sin(theta);

				if(fake_sin > long_sin)
				{
					long_sin = fake_sin;

					top_x_left = hull[j].x;
					top_y_left = hull[j].y;
				}

			}
		}

        double mean_top_x = (double)((top_x_left + top_x_right) / 2.0);
        double mean_top_y = (double)((top_y_left + top_y_right) / 2.0);
		double mean_mid_x = (double)((top_x_left+top_x_right+bottom_x_left+bottom_x_right)/4.0);
		double mean_mid_y = (double)((top_y_left+top_y_right+bottom_y_left+bottom_y_right)/4.0);
        double mean_bottom_x = (double)((bottom_x_left + bottom_x_right) / 2.0);
        double mean_bottom_y = (double)((bottom_y_left + bottom_y_right) / 2.0);

        circle(drawing, Point((int)mean_top_x, (int)mean_top_y), 10, Scalar(0, 0, 255), -1);
		circle(drawing, Point((int)mean_mid_x, (int)mean_mid_y), 10, Scalar(0, 255, 0), -1);
        circle(drawing, Point((int)mean_bottom_x, (int)mean_bottom_y), 10, Scalar(255, 0, 0), -1);
        
        circle(drawing, Point(top_x_left, top_y_left), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(top_x_right, top_y_right), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(bottom_x_left, bottom_y_left), 4, Scalar(255, 255, 255), -1);
        circle(drawing, Point(bottom_x_right, bottom_y_right), 4, Scalar(255, 255, 255), -1);

        array[0] = mean_bottom_x;
		array[1] = mean_bottom_y;
		array[2] = mean_mid_x;
		array[3] = mean_mid_y;
		array[4] = mean_top_x;
		array[5] = mean_top_y;

		if(camera == CAM)
		{
			imshow("Left", drawing);
		}
		else
		{
			imshow("Right", drawing);
		}
		
		return array;
    }
    else
    {
        return array;
    }
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 실제 erp의 GPS를 기준으로 X, Z값을 계산하는 함수
 * 
 * left_point와 right_point는 서로 대응되는 점을 넣어야 함
 * ex) {left_array[0],left_array[1]}를 넣으면 {right_array[0],right_array[1]}를 넣어야 함
 * 
 * @param left_point find_center 함수로 구한 좌측 카메라에서의 x,y값
 * @param right_point find_center 함수로 구한 우측 카메라에서의 x,y값
 * @param left_frame 왼쪽화면 입력영상
 * @param right_frame 오른쪽화면 입력영상
 * @param alpha 카메라가 고개를 숙인 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @param beta 카메라가 틀어진 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @return Point2d (실제 X거리값, 실제 Z거리값, cm단위임)
 */
cv::Point2d Parking::find_xz(const cv::Point2d circle_left, const cv::Point2d circle_right, \
const cv::Mat& left_frame, const cv::Mat& right_frame, const float alpha, const float beta)
{
	float x_0 = 0;
	float y_0 = 0;

	if ((right_frame.cols == left_frame.cols) && (right_frame.rows == left_frame.rows))
	{	
		x_0 = right_frame.cols/2;
		y_0 = right_frame.rows/2;
	}
	else {
		std::cout << "Left and Right Camera frames do not have the same pixel width" << std::endl;	
	}

	float xLeft = circle_left.x;
	float xRight = circle_right.x;
	float yLeft = circle_left.y;
	float yRight = circle_right.y;

	float realX = 0;
	float realY = 0;
	float realZ = 0;
	float distance = 0;

	if(xLeft != x_0)
	{
		realX = (float)Parking::baseline/(1 - (x_0 - xRight)/(x_0 - xLeft));
		realZ = abs(realX*Parking::focal_pixels/(x_0 - xLeft));
	}
	else if(xRight != x_0)
	{
		realX = -(float)Parking::baseline/(1 - (x_0 - xLeft)/(x_0 - xRight));
		realZ = abs(realX*Parking::focal_pixels/(x_0 - xRight));
		realX = realX + (float)Parking::baseline; //왼쪽 카메라 기준
	}
	else
	{
		realX = 0;
		realY = 0;
	}
	realY = realZ*(2*y_0-yLeft-yRight)/(2*Parking::focal_pixels);

	distance = sqrt(pow(realX,2)+pow(realY,2) + pow(realZ,2));

	//////// std::cout << " realX : " << realX << "   realY : "<< realY << "     realZ : " << realZ << std::endl << std::endl;
	// cout << " distance : " << distance << endl;
	
	// cout << " 영점 조절 : " << realX << endl;
	
	//ERP 기준 좌표로 변환
	Parking::alpha = alpha * CV_PI / 180;
	Parking::beta = beta * CV_PI / 180;

	float fakeZ = realZ;
	float fakeY = realY;

	float theta = atan(fakeY/fakeZ) - 35 * CV_PI / 180;
	realZ = sqrt(pow(fakeZ,2)+pow(fakeY,2))*cos(theta);
	realY = sqrt(pow(fakeZ,2)+pow(fakeY,2))*sin(theta);

	float realZ_copy = realZ;
	float realX_copy = realX;

	float gama = atan(realX/realZ) + 48 * CV_PI / 180;
	realZ = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*cos(gama);
	realX = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*sin(gama);
	
	// float angle = 0;
	// angle = atan(realX/realZ)*180/CV_PI;

	std::cout << " realX : " << realX << "   realY : "<< realY << "     realZ : " << realZ << std::endl << std::endl;


	// std::cout << "realZ : " << realZ << "  realX : " << realX << std::endl;
	return {realZ, realX};
}

cv::Mat Parking::applyHighPassFilter(const cv::Mat& src)
{
    cv::Mat image;
    cv::Mat binary;

    image = src.clone();

    resize(image, image, cv::Size(1280, 720));

    cv::cvtColor(image, binary, CV_BGR2GRAY);

    // Apply High Pass Filter (Laplacian)
    cv::Mat highPassImage;
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1);
    cv::filter2D(binary, highPassImage, CV_32F, kernel);

    // Convert the result to grayscale image in [0, 255] range
    double minVal, maxVal;
    cv::minMaxLoc(highPassImage, &minVal, &maxVal);
    highPassImage = (highPassImage - minVal) * (255.0 / (maxVal - minVal));
    highPassImage.convertTo(highPassImage, CV_8U);

    // Apply adaptive threshold
    cv::adaptiveThreshold(highPassImage, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 9, 5);

    cv::Mat binary_inv;

    binary_inv = ~binary;

    // Morphological operations
    cv::morphologyEx(binary_inv, binary_inv, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

    cv::erode(binary_inv, binary_inv, cv::Mat());

    resize(binary_inv, binary_inv, cv::Size(1280, 720));

    cv::imshow("dst", binary_inv);

    return binary_inv;
}

//--------------------------------------------------------------------------------------------------
/**
 * @brief 그림자 문제를 해결하기 위해 적응형 이진화 함수를 사용해 색공간이 아닌 다른 방법으로 영상을 이진화함
 * 
 * @param src 입력영상 
 * @return Mat 이진화된 영상 
 */
/*
cv::Mat Parking::adapt_th(cv::Mat src)
{
	cv::Mat image;
	cv::Mat binary;

	image = src.clone();

	resize(image, image, cv::Size(1280, 720));

	// imshow("cap", image);

	cvtColor(image, binary, CV_BGR2GRAY);
	// namedWindow("dst");
	// createTrackbar("Block_Size", "dst", 0, 200, on_trackbar, (void*)&binary);
	// setTrackbarPos("Block_Size", "dst", 11);

	adaptiveThreshold(binary, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 9, 5);

	cv::Mat binary_inv;

	binary_inv = ~binary;

	// morphologyEx(binary, binary, MORPH_OPEN, Mat(), Point(-1, -1), 3);

	morphologyEx(binary_inv, binary_inv, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

	erode(binary_inv, binary_inv, cv::Mat());

	resize(binary_inv, binary_inv, cv::Size(640, 480));

	imshow("dst", binary_inv);

	return binary_inv;
}*/