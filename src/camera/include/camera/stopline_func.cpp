#include <camera/stopline.hpp>
#include <iostream>

//-----------------------------------------------------------------------------------------------------------
cv::Mat StopLine::make_zeros(const cv::Mat& img)
{
    return cv::Mat::zeros(img.rows, img.cols, img.type());
}

//-----------------------------------------------------------------------------------------------------------
// 이 함수를 통해 리턴된 객체 => bird_eye_viewed_img
cv::Mat StopLine::to_bird_eye_view(const cv::Mat& raw_img)
{
    int width = raw_img.cols;
	int height = raw_img.rows;
    //cv::Mat warp_matrix; -> 클래스 이동
    cv::Mat warp_matrix_inv;
    cv::Point2f warp_src_point[4];
	cv::Point2f warp_dst_point[4];

    //원본의 좌표(좌하단, 우하단, 좌상단, 우상단) <ORIGIN>
	warp_src_point[0].x = -60; //5;
	warp_src_point[0].y = height;// - 15;
	warp_src_point[1].x = width - warp_src_point[0].x;
	warp_src_point[1].y = warp_src_point[0].y;
	warp_src_point[2].x = 260;
	warp_src_point[2].y = 270; //140;
	warp_src_point[3].x = width - warp_src_point[2].x;
	warp_src_point[3].y = warp_src_point[2].y;

    //목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ origin
	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - warp_dst_point[0].x;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - warp_dst_point[2].x;
	warp_dst_point[3].y = 0;

	StopLine::warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
    invert(StopLine::warp_matrix, warp_matrix_inv);

    // 변환 매트릭스 타입 확인
    if (StopLine::warp_matrix.type() != CV_32F && StopLine::warp_matrix.type() != CV_64F)
    {
        std::cout << "Converting matrix type to CV_32F" << std::endl;
        StopLine::warp_matrix.convertTo(StopLine::warp_matrix, CV_32F);
    }

    cv::Mat bird_eye_viewed_img;
    cv::warpPerspective(raw_img, bird_eye_viewed_img, StopLine::warp_matrix, cv::Size(width, height));
    return bird_eye_viewed_img;
}



//-----------------------------------------------------------------------------------------------------------
cv::Mat StopLine::find_line(const cv::Mat& bird_eye_viewed_img)
{
    cv::Mat img_warp_clone, img_warp_clone_hls, img_show;

	cv::Mat white = cv::Mat::zeros(480, 640, CV_8UC3);
	cv::rectangle(white, cv::Rect(0, 450, 30, 30), cv::Scalar(250, 250, 250), -1);

	img_warp_clone = bird_eye_viewed_img.clone();
	img_show = bird_eye_viewed_img.clone();
	img_warp_clone_hls = img_warp_clone.clone();

	cv::Mat img_warp_clone_hls_white = img_warp_clone_hls + white;

	cv::Mat img_OUT_HLS;
	img_OUT_HLS = hsl_binarization(img_warp_clone_hls_white);

	cv::Mat BLURRED;
	cv::medianBlur(img_OUT_HLS, BLURRED, 3);

	cv::Mat OPEN_CLOSE;
	cv::morphologyEx(BLURRED, OPEN_CLOSE, cv::MORPH_CLOSE, cv::Mat());
	cv::morphologyEx(OPEN_CLOSE, OPEN_CLOSE, cv::MORPH_OPEN, cv::Mat());

	cv::rectangle(img_show, cv::Rect(cv::Point(bird_eye_viewed_img.cols * 1 / 7, bird_eye_viewed_img.rows * 3 / 5), cv::Point(bird_eye_viewed_img.cols * 2 / 7, bird_eye_viewed_img.rows * 4 / 5)), cv::Scalar(0, 0, 255), 1, 8, 0);
	cv::rectangle(img_show, cv::Rect(cv::Point(bird_eye_viewed_img.cols * 4 / 7, bird_eye_viewed_img.rows * 3 / 5), cv::Point(bird_eye_viewed_img.cols * 5 / 7, bird_eye_viewed_img.rows * 4 / 5)), cv::Scalar(0, 0, 255), 1, 8, 0);
	cv::rectangle(img_show, cv::Rect(cv::Point(bird_eye_viewed_img.cols * 1 / 2 - 30, bird_eye_viewed_img.rows * 1 / 7), cv::Point(bird_eye_viewed_img.cols * 1 / 2 + 30, bird_eye_viewed_img.rows * 2 / 7)), cv::Scalar(0, 0, 255), 1, 8, 0);

    cv::Mat img_binary;
    img_binary = OPEN_CLOSE;

    cv::Mat img_integral;
    cv::integral(img_binary, img_integral);

    cv::Mat img_mask;
    img_mask = mask_filter(img_integral, 5, 5, 200);

    cv::Mat processed_img;
    cv::Mat warp_matrix_inv;
    // Invert the matrix
    cv::invert(StopLine::warp_matrix, warp_matrix_inv);

    // Check the type of the matrix
    if (warp_matrix_inv.type() != CV_32F && warp_matrix_inv.type() != CV_64F)
    {
        std::cout << "Converting matrix type to CV_32F" << std::endl;
        warp_matrix_inv.convertTo(warp_matrix_inv, CV_32F);
    }

    cv::warpPerspective(img_mask, processed_img, warp_matrix_inv, bird_eye_viewed_img.size());

    return processed_img;

}



//-----------------------------------------------------------------------------------------------------------
cv::Mat StopLine::filterImg(const cv::Mat& imgUnwarp, const int toColorChannel, const int mode)
{
    cv::Mat imgConverted;
    cv::Mat imgOUT = cv::Mat::zeros(480, 640, CV_8UC1);

    /* 1. Convert color channel from BGR to HLS or LAB. */
    if (toColorChannel == 0)
    {
        cv::cvtColor(imgUnwarp, imgConverted, cv::COLOR_BGR2HLS);
    }
    else if (toColorChannel == 1)
    {
        cv::cvtColor(imgUnwarp, imgConverted, cv::COLOR_BGR2Lab);
    }

    /* 2. Pixel pointer variable setting. */
    const uint8_t *pixelPtr = imgConverted.data;
    int cn = imgConverted.channels();

    switch (mode)
    {
    case 0:
        // Set H space only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 0];
            }
        }
        break;

    case 1:
        // Set L space only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 1];
            }
        }
        break;

    case 2:
        // Set S space only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 2];
            }
        }
        break;

    default:
        break;
    }

    return imgOUT;
}



//-----------------------------------------------------------------------------------------------------------
cv::Mat StopLine::mask_filter(const cv::Mat& integraled_img, const int mask_width, const int mask_height, \
const int thresh)
{
    int height = integraled_img.rows;
    int width = integraled_img.cols;
    cv::Mat img_maskfilter = cv::Mat::zeros(height, width, CV_8UC1); // Initialize filter with zeros
    cv::Mat stop_line_mask_img = cv::Mat::zeros(height, width, CV_8UC3); // Initialize stop_img with zeros
    float mask[3];
    //int sx = 0;
    StopLine::is_stop_ = 100;

    uint *image = (uint *)integraled_img.data;
    uchar *score_data = (uchar *)img_maskfilter.data;
    int mask_w = mask_width;
    int mask_h = mask_height;

    int sy = 0;

    int roi_w = 150;
    int histo = 0;

    for (int y = height - 15; y > 20; y--)
    {
        histo = 0;
        for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
        {
            for (int i = 0; i < 3; i++)
            {
                sy = y + (2 * mask_h + 1) * (i - 1);
                int dx, cx, bx, ax;
                int dy, cy, by, ay;
                dy = sy + mask_h;
                dx = x + mask_w;
                cy = sy - mask_h - 1;
                cx = x + mask_w;
                by = sy + mask_h;
                bx = x - mask_w - 1;
                ay = sy - mask_h - 1;
                ax = x - mask_w - 1;
                mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
            }
            float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
            if (sum > 6000)
            {
                score_data[width * y + x] = 255;
                histo++;
            }
        }
        cv::line(stop_line_mask_img, cv::Point(int(width / 2) + roi_w, 20), cv::Point(int(width / 2) + roi_w, height), cv::Scalar(255, 255, 0), 5);
        cv::line(stop_line_mask_img, cv::Point(int(width / 2) - roi_w, 20), cv::Point(int(width / 2) - roi_w, height), cv::Scalar(255, 255, 0), 5);

        if (histo > thresh)
        {
            cv::line(stop_line_mask_img, cv::Point(int(width / 2) - roi_w, y), cv::Point(int(width / 2) + roi_w, y), cv::Scalar(255, 0, 0), 30);
            if (y < 301)
            {
                std::cout << "stop line distance: 10M" << std::endl;
                StopLine::is_stop_ = 10;
            }
            else if (y < 308)
            {
                std::cout << "stop line distance: 9M" << std::endl;
                StopLine::is_stop_ = 9;
            }
            else if (y < 317)
            {
                std::cout << "stop line distance: 8M" << std::endl;
                StopLine::is_stop_ = 8;
            }
            else if (y < 328)
            {
                std::cout << "stop line distance: 7M" << std::endl;
                StopLine::is_stop_ = 7;
            }
            else if (y < 348)
            {
                std::cout << "stop line distance: 6M" << std::endl;
                StopLine::is_stop_ = 6;
            }
            else if (y < 371)
            {
                std::cout << "stop line distance: 5M" << std::endl;
                StopLine::is_stop_ = 5;
            }
            else if (y < 403)
            {
                std::cout << "stop line distance: 4M" << std::endl;
                StopLine::is_stop_ = 4;
            }
            else if (y < 455)
            {
                std::cout << "stop line distance: 3M" << std::endl;
                StopLine::is_stop_ = 3;
            }
            // else if (y < 307) //367
            // {
                // std::cout << "stop line distance: 2M" << std::endl;
                // StopLine::is_stop_ = 2;
            // }
            else
            {
                std::cout << "GO" << std::endl;
                StopLine::is_stop_ = 100;
            }

            break;
        }
    }
    cv::imshow("stop_line_mask_img", stop_line_mask_img);
    return stop_line_mask_img;
}



//-----------------------------------------------------------------------------------------------------------
cv::Mat StopLine::hsl_binarization(const cv::Mat& bird_eye_viewed_img_with_white)
{

    /* normalizing L color channel pixel from hls img. */
    cv::Mat imgHLS_L, imgNormal;
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    int lowThres=200; // origin : 200

    cv::Scalar mean_color1 = get_img_mean(bird_eye_viewed_img_with_white, bird_eye_viewed_img_with_white.rows * 3 / 5, \
    bird_eye_viewed_img_with_white.rows * 1 / 5, bird_eye_viewed_img_with_white.cols * 1 / 7, \
    bird_eye_viewed_img_with_white.cols / 7);

    cv::Scalar mean_color2 = get_img_mean(bird_eye_viewed_img_with_white, bird_eye_viewed_img_with_white.rows * 3 / 5, \
    bird_eye_viewed_img_with_white.rows * 1 / 5, bird_eye_viewed_img_with_white.cols * 4 / 7, \
    bird_eye_viewed_img_with_white.cols * 1 / 7);

    int mean_thres = (mean_color1[1] + mean_color2[1]) / 2 + 20;

    if (mean_thres < 159) {
        lowThres = 170;
    }
    else {
        lowThres = mean_thres;
    }

    // printf("%d",lowThres);
    // get a single channel img(filtered one.)
    imgHLS_L = filterImg(bird_eye_viewed_img_with_white, 0, 1);
    // imshow("filterimg",imgHLS_L);
    // get max, min value of the matrix.
    cv::minMaxLoc(imgHLS_L, &minVal, &maxVal, &minLoc, &maxLoc);

    // make normalized img.
    imgNormal = (255 / maxVal) * imgHLS_L;

    // imshow("normalimg",imgNormal);
    // apply threshold for L channel.
    cv::Mat hsl_2_binarized_img = make_zeros(imgNormal);
    // Mat imgOut2 = make_zeros(imgNormal);

    // 적응형 이진화
    cv::threshold(imgNormal, hsl_2_binarized_img, lowThres, 255, cv::THRESH_BINARY);

    return hsl_2_binarized_img;
}



//-----------------------------------------------------------------------------------------------------------
/*
* @brief (GPT오류지적)영상에서 특정 위치의 픽셀 평균값 계산
* @param 흰배경+버드아이뷰 이미지
* @param 좌상단x값
* @param 이미지너비
* @param 좌상단y값
* @param 이미지높이
*/
cv::Scalar StopLine::get_img_mean(const cv::Mat& bird_eye_viewed_img_with_white, const int img_top_left_x,\
const int img_width, const int img_top_left_y, const int img_height)
{
    //GPT는 좌표를 다음과 같이 추천함 (y,x), (y+너비, x+높이)
    cv::Mat img_roi = bird_eye_viewed_img_with_white(cv::Rect(cv::Point(img_top_left_y, \
    img_top_left_x), cv::Point(img_top_left_y + img_height, img_top_left_x + img_width)));

	cv::Scalar return_pixel_average = mean(img_roi);
	// std::cout << return_pixel_average << std::endl;
    return return_pixel_average;
}



//-----------------------------------------------------------------------------------------------------------
/*
* @brief 최종 영상처리된 이미지와 정지선까지의 거리를 나타냄
* @param 영상처리된 이미지
* @param 정지선까지의 거리 확인
*/
int StopLine::show_return_distance(const cv::Mat& processed_img)
{
    int distance;
    std::string text;
    cv::Point text_position(500, 100);
    cv::Scalar text_color(0, 0, 255);

    if (StopLine::is_stop_ == 7)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
    }

    else if (StopLine::is_stop_ == 6)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
        // StopLine::is_stop_=100;
    }
    else if (StopLine::is_stop_ == 5)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
        // StopLine::is_stop_=100;
    }
    else if (StopLine::is_stop_ == 4)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
        // StopLine::is_stop_ = 100;
    }
    else if (StopLine::is_stop_ == 3)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
        // StopLine::is_stop_ = 100;
    }
    else if (StopLine::is_stop_ == 2)
    {
        distance = StopLine::is_stop_;
        text = std::to_string(distance) + "M";
        StopLine::is_stop_ = 100;
    }

    else if (StopLine::is_stop_ == 100)
    {
        distance = StopLine::is_stop_;
        text = "Go";
    }

    putText(processed_img, text, text_position, cv::FONT_HERSHEY_SIMPLEX, 3, text_color);

    return distance;
}