#ifndef RENDERENGINE_H
#define RENDERENGINE_H

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <cfloat>
using namespace Eigen;

namespace RenderEngine{
    //枚举类
    enum class renderBuffers
    {
        rgbColor =1,
        Depth =2
    };
    //操作符或
    inline renderBuffers operator|(renderBuffers a,renderBuffers b)
    {
        return renderBuffers((int)a | (int)b);
    }
    //操作符与
    inline renderBuffers operator&(renderBuffers a,renderBuffers b)
    {
        return renderBuffers((int)a & (int)b);
    }
    //枚举渲染类型
    enum class renderType
    {
        Line,
        Triangle
    };
    //顶点缓存的id
    struct position_buf_id
    {
        int pos_id = 0;
    };
    //顶点索引的id
    struct index_buf_id
    {
        int index_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

    //渲染引擎
    class Engine
    {
    public:
        Engine(int w,int h);
        ~Engine();
        //加载顶点缓冲
        position_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        //加载顶点索引
        index_buf_id load_indexs(const std::vector<Eigen::Vector3i>& indexs);

        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        //设置模型矩阵
        void set_model(const Eigen::Matrix4f & m);
        //设置视图矩阵
        void set_view(const Eigen::Matrix4f & v);
        //设置透视矩阵
        void set_projection(const Eigen::Matrix4f & p);
        //设置像素颜色
        void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);
        //清理屏幕
        void clear(renderBuffers buffer);
        //绘制基本渲染类型：线/三角形
        void draw(position_buf_id pos_buf, index_buf_id ind_buf, col_buf_id col_buffer,renderType type);
        //
        std::vector<Eigen::Vector3f> &frame_buffer(){
            return frame_buf;
        }

    private:
        Eigen::Matrix4f model;
        Eigen::Matrix4f view;
        Eigen::Matrix4f projection;
        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;

        int width,height;
        //帧缓存
        std::vector<Eigen::Vector3f> frame_buf;
        //深度缓存
        std::vector<float> depth_buffer;
        //
        int get_index(int x,int y);
        //
        int next_id = 0;
        //
        int get_next_id() {return next_id++;}

    private:
        //画直线算法
        void draw_Line(Eigen::Vector3f begin, Eigen::Vector3f end);
        void renderFrame(const Triangle &t);
        void rasterize_triangle(const Triangle& t);
    };
    
}


#endif