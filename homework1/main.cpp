#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	
	float pi_angle = rotation_angle * MY_PI / 180.0f;
	
	model << cos(pi_angle), -sin(pi_angle), 0, 0, 
             sin(pi_angle), cos(pi_angle), 0, 0, 
             0, 0, 1, 0,
		     0, 0, 0, 1;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
	Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
	
	Eigen::Matrix3f temp = Eigen::Matrix3f::Identity();
	
	Eigen::Matrix3f asym = Eigen::Matrix3f::Identity();
	
	asym << 0, -axis(2), axis(1),
			axis(2), 0, -axis(0),
			-axis(1), axis(0), 0;
	
	float pi_angle = angle * MY_PI / 180.0f;
	
	temp = cos(pi_angle) * temp + (1 - cos(pi_angle)) * axis * axis.transpose() + sin(pi_angle) * asym;
	
	rotation.block<3, 3>(0, 0) = temp;
	
	return rotation;
	
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	
	float r, t;

    zNear = zNear > 0 ? -zNear : zNear;
    zFar = zFar > 0 ? -zFar : zFar;
	
	t = fabs(zNear) * tan(eye_fov * MY_PI / 180 / 2);
	r = t * aspect_ratio;
	
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
	
	translate << 1, 0, 0, 0, 
				 0, 1, 0, 0, 
				 0, 0, 1, -(zNear + zFar) / 2, 
				 0, 0, 0, 1;
	
	scale << 1/r, 0, 0, 0,
			 0, 1/t, 0, 0,
			 0, 0, 2/(zNear - zFar), 0,
			 0, 0, 0, 1;
	
	projection << zNear, 0, 0, 0,
				  0, zNear, 0, 0,
				  0, 0, zNear+zFar, -zNear*zFar,
				  0, 0, 1, 0;
				  
	projection = scale * translate * projection;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
	Vector3f axis;
	
	float xs = 0;
	float ys = 0;
	float zs = 1;

    bool get_axis = true;

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc >= 4) {
            filename = std::string(argv[3]);
        }
		if (argc == 7) {
			xs = std::stof(argv[4]);
			ys = std::stof(argv[5]);
			zs = std::stof(argv[6]);
		}
    }
	axis << xs, ys, zs;

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
		r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (get_axis == true){
            std::cout << "please input the rotation vector(x, y, z):";
        
            std::cin >> xs >> ys >> zs;
            axis << xs, ys, zs;
            get_axis = false;
        }
		
		

        //r.set_model(get_model_matrix(angle));
		r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
