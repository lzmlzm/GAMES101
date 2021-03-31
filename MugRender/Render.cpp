/**
*RenderEngine test program
*/
#include "Triangle.hpp"
#include "RenderEngine.hpp"
#include <eigen3/Eigen/Eigen>
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

Eigen::Matrix4f rotate(std::string xyz,float rotate_angle)
{
    //弧度转换
    Eigen::Matrix4f rotateM;
    float radian =  rotate_angle/180.0*MY_PI;
    if(xyz.compare("x")==0){
        printf("x");
        rotateM<<1,0,0,0,
                0,cos(radian),-sin(radian),0,
                0,sin(radian),cos(radian),0,
                0,0,0,1;
    }else if(xyz.compare("y")==0){
        rotateM<<cos(radian),0,sin(radian),0,
                0,1,0,0,
                -sin(radian),0,cos(radian),0,
                0,0,0,1;
    }else if (xyz.compare("z")==0)
    {
        printf("z");
        rotateM << cos(radian),-sin(radian),0,0,
            sin(radian),cos(radian),0,0,
            0,0,1,0,
            0,0,0,1;
    }
    return rotateM;
}

Eigen::Matrix4f scale(float scalesize)
{
    Eigen::Matrix4f scaleM;
    scaleM<<scalesize,0,0,0,
            0,scalesize,0,0,
            0,0,scalesize,0,
            0,0,0,1;

    return scaleM;
}
Eigen::Matrix4f translate(std::string xyz,float translatesize)
{
    Eigen::Matrix4f translateM;
    if(xyz.compare("x")==0){
        printf("x");
        translateM<<1,0,0,translatesize,
                0,1,0,1,
                0,0,1,1,
                0,0,0,1;
    }else if(xyz.compare("y")==0){
        translateM<<1,0,0,1,
                0,1,0,translatesize,
                0,0,1,1,
                0,0,0,1;
    }else if (xyz.compare("z")==0)
    {
        printf("z");
        translateM<<1,0,0,1,
               0,1,0,1,
               0,0,1,translatesize,
               0,0,0,1;
    }

    return translateM;
}
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //初始化一个4阶单位矩阵
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    //
    Eigen::Matrix4f rotateM=rotate("z",rotation_angle);
    Eigen::Matrix4f scaleM(4,4);
    Eigen::Matrix4f translateM=translate("x",-1);
    
    //rotate(rotateM,"z",rotation_angle);
    //scale(scaleM,0.5);
    //TODO SCALE
    //float radian =  rotation_angle/180.0*MY_PI;
    /*
    rotateM << cos(radian),-sin(radian),0,0,
    sin(radian),cos(radian),0,0,
    0,0,1,0,
    0,0,0,1;*/
    
    model = rotateM*translateM*model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f M_persp2ortho(4, 4);
	Eigen::Matrix4f M_ortho_scale(4, 4);
	Eigen::Matrix4f M_ortho_trans(4, 4);
	//已更正
	float angle = eye_fov * MY_PI / 180.0; 
	float height = zNear * tan(angle) * 2;
	float width = height * aspect_ratio;

	auto t = -zNear * tan(angle / 2);  // 上截面
	auto r = t * aspect_ratio;  //右截面   
	auto l = -r;  // 左截面
	auto b = -t;  // 下截面
	// 透视矩阵"挤压"
	M_persp2ortho << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;
	// 正交矩阵-缩放
	M_ortho_scale << 2 / (r - l), 0, 0, 0,
		0, 2 / (t - b), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;
	// 正交矩阵-平移
	M_ortho_trans << 1, 0, 0, -(r + l) / 2,
		0, 1, 0, -(t + b) / 2,
		0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;
	Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_trans;
	projection = M_ortho * M_persp2ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
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

    //显示范围
    RenderEngine::Engine engine(700, 700);
    //Z轴的远近
    Eigen::Vector3f eye_pos = {0, 0, 10};

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

    auto pos_id = engine.load_positions(pos);
    auto ind_id = engine.load_indexs(ind);
    auto col_id = engine.load_colors(cols);


    int key = 0;
    int frame_count = 0;

    if (command_line) {
        engine.clear(RenderEngine::renderBuffers::rgbColor | RenderEngine::renderBuffers::Depth);

        engine.set_model(get_model_matrix(angle));
        engine.set_view(get_view_matrix(eye_pos));
        engine.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        engine.draw(pos_id, ind_id, col_id,RenderEngine::renderType::Triangle);
        cv::Mat image(700, 700, CV_32FC3, engine.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        //Clear
        engine.clear(RenderEngine::renderBuffers::rgbColor | RenderEngine::renderBuffers::Depth);

        //Set MVP
        engine.set_model(get_model_matrix(angle));
        engine.set_view(get_view_matrix(eye_pos));
        engine.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //Draw
        engine.draw(pos_id, ind_id,col_id, RenderEngine::renderType::Triangle);
        //生成一幅700*700的image
        cv::Mat image(700, 700, CV_32FC3, engine.frame_buffer().data());
        //类型转换
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
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