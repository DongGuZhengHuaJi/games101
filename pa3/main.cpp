#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection;

    float half_eye_fov = eye_fov / 2.0f;
    float theta = half_eye_fov * MY_PI / 180.0f;
    float t = std::tan(theta) * std::abs(zNear);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    float f = std::abs(zFar);
    float n = std::abs(zNear);

    projection << 2*n/(r-l), 0, (r+l)/(r-l), 0,
                  0, 2*n/(t-b), (t+b)/(t-b), 0,
                  0, 0, -(f+n)/(f-n), -2*f*n/(f-n),
                  0, 0, -1, 0;
    return projection;

}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());

    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized(); 
        float distance = (light.position - point).norm();
        Eigen::Vector3f half_vec = (light_dir + view_dir).normalized();

        Eigen::Vector3f ambient  = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f diffuse  = kd.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, normal.dot(light_dir));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, std::pow(normal.dot(half_vec), p));

        result_color += ambient + diffuse + specular;
        
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized(); 
        float distance = (light.position - point).norm();
        Eigen::Vector3f half_vec = (light_dir + view_dir).normalized();

        Eigen::Vector3f ambient  = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f diffuse  = kd.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, normal.dot(light_dir));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, std::pow(normal.dot(half_vec), p));

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here


    Eigen::Vector3f n = normal;
    Eigen::Vector3f t = Eigen::Vector3f(n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()), sqrt(n.x() * n.x() + n.z() * n.z()), n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()));
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t, b, n;
    TBN.transposeInPlace();
    float dU = kh * kn * (payload.texture->getColor(payload.tex_coords.x() + 1.0f / payload.texture->width, payload.tex_coords.y()).norm() - payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm());
    float dV = kh * kn * (payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y() + 1.0f / payload.texture->height).norm() - payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm());
    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1);
    Eigen::Vector3f adjusted_normal = (TBN * ln).normalized();
    
    Eigen::Vector3f adjusted_point = point + kn * normal * payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm();    


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = (light.position - adjusted_point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - adjusted_point).normalized(); 
        float distance = (light.position - adjusted_point).norm();
        Eigen::Vector3f half_vec = (light_dir + view_dir).normalized();

        Eigen::Vector3f ambient  = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f diffuse  = kd.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, adjusted_normal.dot(light_dir));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / (distance * distance) * std::max(0.f, std::pow(adjusted_normal.dot(half_vec), p));

        result_color += ambient + diffuse + specular;


    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here

    Eigen::Vector3f n = normal;
    Eigen::Vector3f t = Eigen::Vector3f(n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()), sqrt(n.x() * n.x() + n.z() * n.z()), n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()));
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t, b, n;
    TBN.transposeInPlace();
    float dU = kh * kn * (payload.texture->getColor(payload.tex_coords.x() + 1.0f / payload.texture->width, payload.tex_coords.y()).norm() - payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm());
    float dV = kh * kn * (payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y() + 1.0f / payload.texture->height).norm() - payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y()).norm());
    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1);
    Eigen::Vector3f adjusted_normal = (TBN * ln).normalized();


    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = adjusted_normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    // Parse shader name from either argv[1] (interactive) or argv[2] (command-line)
    std::string shader_arg;
    if (argc == 2)
    {
        // ./Rasterizer <shader>  → interactive mode with specified shader
        // ./Rasterizer <output>  → command-line mode (if not a shader name)
        std::string arg1(argv[1]);
        if (arg1 == "texture" || arg1 == "normal" || arg1 == "phong" ||
            arg1 == "bump" || arg1 == "displacement")
        {
            shader_arg = arg1;
            command_line = false;
        }
        else
        {
            command_line = true;
            filename = arg1;
        }
    }
    else if (argc >= 3)
    {
        command_line = true;
        filename = std::string(argv[1]);
        shader_arg = std::string(argv[2]);
    }

    if (shader_arg == "texture")
    {
        std::cout << "Rasterizing using the texture shader\n";
        active_shader = texture_fragment_shader;
        texture_path = "spot_texture.png";
        r.set_texture(Texture(obj_path + texture_path));
    }
    else if (shader_arg == "normal")
    {
        std::cout << "Rasterizing using the normal shader\n";
        active_shader = normal_fragment_shader;
    }
    else if (shader_arg == "phong")
    {
        std::cout << "Rasterizing using the phong shader\n";
        active_shader = phong_fragment_shader;
    }
    else if (shader_arg == "bump")
    {
        std::cout << "Rasterizing using the bump shader\n";
        active_shader = bump_fragment_shader;
    }
    else if (shader_arg == "displacement")
    {
        std::cout << "Rasterizing using the displacement shader\n";
        active_shader = displacement_fragment_shader;
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        std::cout << "Frame " << frame_count << " rendered. [a/d] rotate, [ESC] quit" << std::endl;
        key = cv::waitKey(0);

        if (key == 'a' )
        {
            angle -= 5.0;
        }
        else if (key == 'd')
        {
            angle += 5.0;
        }

    }
    return 0;
}
