#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/GlobalFunctions.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <opencv2/opencv.hpp>

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

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float theta = rotation_angle*MY_PI/180.0f;

    float c = std::cos(theta);
    float s = std::sin(theta);

    Eigen::Matrix4f R_matrix;

    R_matrix << c, -s, 0, 0,
                s,  c, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1 ; 
    return R_matrix * model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float alpha = (eye_fov/2)*MY_PI/180.0f;

    float z = -zNear;
    float Z = -zFar;

    float y = std::tan(alpha)*std::abs(z);
    float x = aspect_ratio*y;

    Eigen::Matrix4f P_matrix = Eigen::Matrix4f::Identity();

    P_matrix << z, 0, 0, 0,
                0, z, 0, 0,
                0, 0, z+Z, -z*Z,
                0, 0, 1, 0;

    Eigen::Matrix4f M_ortho = Eigen::Matrix4f::Identity();

    M_ortho << (2.0f)/(x-(-x)), 0, 0, -(x+(-x))/(x-(-x)),
               0, (2.0f)/(y-(-y)), 0, -(y+(-y))/(y-(-y)),
               0, 0, (2.0f)/(z-Z), -(z+Z)/(z-Z),
               0, 0, 0, 1;

    projection = M_ortho * P_matrix;

    return projection;
}

// Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {

//     axis.normalize();

//     // 计算 YZ 平面上的投影长度（斜边）
//     float d_yz = std::sqrt(axis.y() * axis.y() + axis.z() * axis.z());

//     // 计算绕 X 轴所需的 sin 和 cos
//     float alpha_sin = (d_yz == 0) ? 0.0f : axis.y() / d_yz;
//     float alpha_cos = (d_yz == 0) ? 1.0f : axis.z() / d_yz;

//     Eigen::Matrix4f R_matrix_x;
//     R_matrix_x << 1, 0,          0,          0,
//                 0, alpha_cos,  -alpha_sin, 0,
//                 0, alpha_sin,  alpha_cos,  0,
//                 0, 0,          0,          1;

//     Eigen::Vector4f axis_new = R_matrix_x * Eigen::Vector4f(axis.x(), axis.y(), axis.z(), 1);

//     float d_xz = std::sqrt(axis_new[0] * axis_new[0] + axis_new[2] * axis_new[2]);

//     float theta_sin = (d_xz == 0) ? 0.0f : -axis_new[0] / d_xz;
//     float theta_cos = (d_xz == 0) ? 1.0f : axis_new[2] / d_xz;

//     Eigen::Matrix4f R_matrix_y;
//     R_matrix_y << theta_cos, 0, theta_sin, 0,
//                 0, 1, 0, 0,
//                 -theta_sin, 0, theta_cos, 0,
//                 0, 0, 0, 1;

//     Eigen::Matrix4f R_z = get_model_matrix(angle);

//     return R_matrix_x.transpose() * R_matrix_y.transpose() * R_z * R_matrix_y * R_matrix_x;
// }

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    axis.normalize();

    float rad = angle * MY_PI / 180.0f;

    Eigen::Matrix3f N;
    N << 0, -axis.z(), axis.y(),
         axis.z(), 0, -axis.x(),
         -axis.y(), axis.x(), 0;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f R =
        cos(rad) * I +
        (1 - cos(rad)) * axis * axis.transpose() +
        sin(rad) * N;

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block<3,3>(0,0) = R;

    return result;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    float angle2 = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

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

        r.set_model(get_rotation({0.0f, 1.0f, 0.0f}, angle2) * get_model_matrix(angle));
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

        r.set_model(get_rotation({0.0f, 1.0f, 0.0f}, angle2) * get_model_matrix(angle));
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
        else if (key == 'w') {
            angle2 += 10;
        }
        else if (key == 's') {
            angle2 -= 10;
        }
    }

    return 0;
}
