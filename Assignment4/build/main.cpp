#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>


std::vector<cv::Point2f> control_points;

double getDistance(cv::Point2f point1, cv::Point2f point2)
{
    double distance = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
    return distance;
}

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
	std::vector<cv::Point2f> temp_control_points;
	
	if (control_points.size() == 1){
		return control_points[0];
	}
	
	else
	{
		for(int i = 0; i < control_points.size()-1; i++ )
		{
			float x0 = control_points[i].x * (1.0 - t) + control_points[i+1].x * t;
			float y0 = control_points[i].y * (1.0 - t) + control_points[i+1].y * t;
            
			temp_control_points.push_back( cv::Point2f(x0, y0) );
		}
		
		return recursive_bezier( temp_control_points, t );
	}
    // TODO: Implement de Casteljau's algorithm
 
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
	for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier( control_points, t );

        //std::cout << "point.x = " << point.x << std::endl;
        //std::cout << "point.y = " << point.y << std::endl;

        cv::Point2f pixel_11(floor(point.x) + 0.5, floor(point.y) + 0.5);

        cv::Point2f pixel_21(pixel_11.x + 1 , pixel_11.y);

        cv::Point2f pixel_12(pixel_11.x , pixel_11.y + 1);

        cv::Point2f pixel_22(pixel_11.x + 1, pixel_11.y + 1);

        cv::Point2f pixel_00(pixel_11.x - 1, pixel_11.y - 1);

        cv::Point2f pixel_10(pixel_11.x , pixel_11.y - 1);

        cv::Point2f pixel_20(pixel_11.x + 1, pixel_11.y - 1);

        cv::Point2f pixel_01(pixel_11.x - 1, pixel_11.y);

        cv::Point2f pixel_02(pixel_11.x - 1, pixel_11.y + 1);

        double offset_11 = 1 - getDistance(pixel_11, point) / 2.12;

        double offset_12 = 1 - getDistance(pixel_12, point) / 2.12;

        double offset_21 = 1 - getDistance(pixel_21, point) / 2.12;

        double offset_22 = 1 - getDistance(pixel_22, point) / 2.12;

        double offset_00 = 1 - getDistance(pixel_00, point) / 2.12;

        double offset_10 = 1 - getDistance(pixel_10, point) / 2.12;

        double offset_20 = 1 - getDistance(pixel_20, point) / 2.12;

        double offset_01 = 1 - getDistance(pixel_01, point) / 2.12;

        double offset_02 = 1 - getDistance(pixel_02, point) / 2.12;

        window.at<cv::Vec3b>(pixel_11.y, pixel_11.x)[1] = std::max( 255 * offset_11, (double) window.at<cv::Vec3b>(pixel_11.y, pixel_11.x)[1]);

        window.at<cv::Vec3b>(pixel_21.y, pixel_21.x)[1] = std::max( 255 * offset_21, (double) window.at<cv::Vec3b>(pixel_21.y, pixel_21.x)[1]);

        window.at<cv::Vec3b>(pixel_12.y, pixel_12.x)[1] = std::max( 255 * offset_12, (double) window.at<cv::Vec3b>(pixel_12.y, pixel_12.x)[1]);

        window.at<cv::Vec3b>(pixel_22.y, pixel_22.x)[1] = std::max( 255 * offset_22, (double) window.at<cv::Vec3b>(pixel_22.y, pixel_22.x)[1]);

        window.at<cv::Vec3b>(pixel_00.y, pixel_00.x)[1] = std::max( 255 * offset_00, (double) window.at<cv::Vec3b>(pixel_00.y, pixel_00.x)[1]);

        window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1] = std::max( 255 * offset_10, (double) window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1]);

        window.at<cv::Vec3b>(pixel_20.y, pixel_20.x)[1] = std::max( 255 * offset_20, (double) window.at<cv::Vec3b>(pixel_20.y, pixel_20.x)[1]);

        window.at<cv::Vec3b>(pixel_01.y, pixel_01.x)[1] = std::max( 255 * offset_01, (double) window.at<cv::Vec3b>(pixel_01.y, pixel_01.x)[1]);

        window.at<cv::Vec3b>(pixel_02.y, pixel_02.x)[1] = std::max( 255 * offset_02, (double) window.at<cv::Vec3b>(pixel_02.y, pixel_02.x)[1]);

        //window.at<cv::Vec3b>(pixel_11.y, pixel_11.x)[1] = 255;

        //window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1] = std::max( 255 * 0.66, (double) window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1]);

        //window.at<cv::Vec3b>(pixel_01.y, pixel_01.x)[1] = std::max( 255 * 0.66, (double) window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1]);

        //window.at<cv::Vec3b>(pixel_11.y, pixel_11.x)[1] = std::max( 255 * 0.33, (double) window.at<cv::Vec3b>(pixel_10.y, pixel_10.x)[1]);

        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
