// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

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
//输入参数为旋转角度
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    //转换弧度制
    double radian = rotation_angle / 180.0 * MY_PI;
    //绕Z轴变换矩阵
    translate <<    cos(radian), -sin(radian), 0, 0,
                    sin(radian), cos(radian), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    model = translate * model;
    return model;
}
//输入参数为视场角、宽高比、近平面和远平面
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection;
    Eigen::Matrix4f persp_to_ortho;
    Eigen::Matrix4f ortho;

    //透视投影到正交投影的变换矩阵
    persp_to_ortho <<   zNear, 0, 0, 0,
                        0, zNear, 0, 0,
                        0, 0, zNear + zFar, -(zNear * zFar),
                        0, 0, 1, 0;
    
    //视场角变为弧度制
    double eye_radian = eye_fov / 180.0 *MY_PI;
    //利用视场角和近平面计算高（一半）
    double t = -zNear * tan(eye_radian/2);
    //利用宽高比和高计算宽（一半）
    double r = t * aspect_ratio;

    //正交投影矩阵
    ortho <<    1/r, 0, 0, 0,
                0, 1/t, 0, 0,
                0, 0, 2/(zNear-zFar), -(zNear + zFar)/(zNear - zFar),
                0, 0, 0, 1;

    auto persp = ortho * persp_to_ortho;

    projection = persp * projection;


    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
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
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on