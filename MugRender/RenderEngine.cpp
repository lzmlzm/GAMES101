#include "RenderEngine.hpp"
#include <math.h>
#include <stdexcept>


RenderEngine::Engine::Engine(int w,int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buffer.resize(w * h);
}

RenderEngine::Engine::~Engine()
{

}
/**
 * 向顶点缓冲添加顶点位置
 * 返回顶点位置索引
 **/

RenderEngine::position_buf_id RenderEngine::Engine::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id,positions);

    return {id};
}

RenderEngine::index_buf_id RenderEngine::Engine::load_indexs(const std::vector<Eigen::Vector3i>& indexs)
{
    auto id = get_next_id();
    ind_buf.emplace(id,indexs);

    return {id};
}


void RenderEngine::Engine::draw_Line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
auto x1 = begin.x();
auto y1 = begin.y();
auto x2 = end.x();
auto y2 = end.y();
Eigen::Vector3f line_color = {255, 255, 255};
int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
dx=x2-x1;
dy=y2-y1;
dx1=fabs(dx);
dy1=fabs(dy);
px=2*dy1-dx1;
py=2*dx1-dy1;
if(dy1<=dx1)
{
    if(dx>=0)
    {
        x=x1;
        y=y1;
        xe=x2;
    }
    else
    {
        x=x2;
        y=y2;
        xe=x1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point,line_color);
    for(i=0;x<xe;i++)
    {
        x=x+1;
        if(px<0)
        {
            px=px+2*dy1;
        }
        else
        {
            if((dx<0 && dy<0) || (dx>0 && dy>0))
            {
                y=y+1;
            }
            else
            {
                y=y-1;
            }
            px=px+2*(dy1-dx1);
        }
          //delay(0);
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
    }
}
else
{
    if(dy>=0)
    {
        x=x1;
        y=y1;
        ye=y2;
    }
    else
    {
        x=x2;
        y=y2;
        ye=y1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point,line_color);
    for(i=0;y<ye;i++)
    {
        y=y+1;
        if(py<=0)
        {
            py=py+2*dx1;
        }
        else
        {
            if((dx<0 && dy<0) || (dx>0 && dy>0))
            {
                x=x+1;
            }
            else
            {
                x=x-1;
            }
            py=py+2*(dx1-dy1);
        }
          //delay(0);
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
    }
}
}

//转换齐次坐标系
auto to_vec4(const Eigen::Vector3f& vector3, float w = 1.0f)
{
    return Vector4f(vector3.x(), vector3.y(), vector3.z(), w);
}

void RenderEngine::Engine::draw(RenderEngine::position_buf_id pos_buffer, RenderEngine::index_buf_id ind_buffer,renderType type)
{  
    //判断类型必须为三角形
    if (type != RenderEngine::renderType::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    //将传入的数据根据id填充进本地pos，index数据
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& index = ind_buf[ind_buffer.index_id];
    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : index)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            //像素点级别的MVP变换
            mvp * to_vec4(buf[i[0]], 1.0f),//X
            mvp * to_vec4(buf[i[1]], 1.0f),//Y
            mvp * to_vec4(buf[i[2]], 1.0f) //Z
        };
        for (auto& vec : v) {
            vec /= vec.w();//对于每一个像素同除以W，归一化齐次坐标
        }
        for (auto & vert : v)
        {
            //0.5是将像素放置在显示区域的中间
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        for (int i = 0; i < 3; ++i)
        {
            //设置三角形三个点
            t.setVertex(i, v[i].head<3>());//X
            t.setVertex(i, v[i].head<3>());//Y
            t.setVertex(i, v[i].head<3>());//Z
        }
        //设置RGB颜色
        t.setColor(0, 255.0,  0.0,  0.0);//R
        t.setColor(1, 0.0  ,255.0,  0.0);//G
        t.setColor(2, 0.0  ,  0.0,255.0);//B
        //绘制三角形边框
        renderFrame(t);
    }
}

//渲染三角形边框
void RenderEngine::Engine::renderFrame(const Triangle &t)
{
    draw_Line(t.c(), t.a());
    draw_Line(t.c(), t.b());
    draw_Line(t.b(), t.a());
}
//模型矩阵
void RenderEngine::Engine::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

//视图矩阵
void RenderEngine::Engine::set_view(const Eigen::Matrix4f & v)
{
    view = v;
}

//透视矩阵
void RenderEngine::Engine::set_projection(const Eigen::Matrix4f & p)
{
    projection = p;
}


void RenderEngine::Engine::clear(RenderEngine::renderBuffers buff)
{
    if ((buff & RenderEngine::renderBuffers::rgbColor) == RenderEngine::renderBuffers::rgbColor)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & RenderEngine::renderBuffers::Depth) == RenderEngine::renderBuffers::Depth)
    {
        std::fill(depth_buffer.begin(), depth_buffer.end(), std::numeric_limits<float>::infinity());
    }
}

int RenderEngine::Engine::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void RenderEngine::Engine::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}
