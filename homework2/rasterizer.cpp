// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool isLeft(float xt, float yt, float xa, float ya, float xb, float yb)   //ÅÐ¶ÏÒ»¸öµãÊÇ·ñÔÚÒ»¸öÏòÁ¿µÄ×ó±ß
{
    float cross_product_z = (xb - xa) * (yt - ya) - (xt - xa) * (yb - ya);
    if(cross_product_z >= 0){
        return true;
    }
    else{
        return false;
    }
}

static bool insideTriangle(float xt, float yt, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f v01, v12, v20, v0t, v1t, v2t;

    v01 << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0;

    v12 << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0;

    v20 << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0;

    v0t << xt - _v[0].x(), yt - _v[0].y(), 0;

    v1t << xt - _v[1].x(), yt - _v[1].y(), 0;

    v2t << xt - _v[2].x(), yt - _v[2].y(), 0;

    float z0 = v01.cross(v0t).z();

    float z1 = v12.cross(v1t).z();

    float z2 = v20.cross(v2t).z();
	
	if((z0 >= 0 && z1 >= 0 && z2 >= 0) || (z0 < 0 && z1 < 0 && z2 < 0) )
    {   
		return true;
	}
	else
    {
		return false;
	}
}



static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = (-vert.z()) * f1 + f2;     //use the minus z value to represent the depth
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
	
	float x_min, x_max, y_min, y_max;

	Vector3f pixel;
	
	x_min = min( {v[0].x(), v[1].x(), v[2].x()} );
	x_max = max( {v[0].x(), v[1].x(), v[2].x()} );

    //cout << "xmin = " << x_min << "   xmax = " << x_max << endl;
	
	y_min = min( {v[0].y(), v[1].y(), v[2].y()} );
	y_max = max( {v[0].y(), v[1].y(), v[2].y()} );

    //cout << "ymin = " << y_min << "   ymax = " << y_max << endl;
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
	
	for (int xt = x_min; xt <= x_max; xt++)
    {
        //cout << "xt = " << xt << " x_min = " << x_min << endl;
		for(int yt = y_min; yt <= y_max; yt++)
        {

            //cout << "f1 = " << f1 << endl;
            //cout << "f2 = " << f2 << endl;

			if(insideTriangle(xt, yt, t.v))
            {
					// If so, use the following code to get the interpolated z value.
				    auto[alpha, beta, gamma] = computeBarycentric2D(xt, yt, t.v);
				    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				    z_interpolated *= w_reciprocal;

                    //cout << "z_ip = " << z_interpolated << endl;

					if( z_interpolated < depth_buf[get_index(xt, yt)] )
                    {
							depth_buf[get_index(xt, yt)] = z_interpolated;
							Eigen::Vector3f temp_pixel;
							temp_pixel << xt, yt, z_interpolated;
							set_pixel(temp_pixel, t.getColor());   //if the coordinates of the bounding box are not int, there will be mistakes. 
					}
			
			}
		}
	}
		

    

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on