#include "RenderEngine.hpp"
#include <math.h>
#include <stdexcept>
#include <vector>

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
//加载颜色
RenderEngine::col_buf_id RenderEngine::Engine::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

//判断像素点是否在三角形内
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Eigen::Vector2f p;
    p<<x,y;

    Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);
    Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);
    Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);

    Eigen::Vector2f AP = p - _v[0].head(2);
	Eigen::Vector2f BP = p - _v[1].head(2);
	Eigen::Vector2f CP = p - _v[2].head(2);


    return AB[0] * AP[1] - AB[1] * AP[0] > 0 
		&& BC[0] * BP[1] - BC[1] * BP[0] > 0
		&& CA[0] * CP[1] - CA[1] * CP[0] > 0;

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void RenderEngine::Engine::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();
    
// TODO : Find out the bounding box of current triangle.
// iterate through the pixel and find if the current pixel is inside the triangle
// If so, use the following code to get the interpolated z value.
//auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//z_interpolated *= w_reciprocal;
// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//Bouding box找到三角形的最小包围盒的xy范围
    float min_x = std::min(v[0][0],std::min(v[1][0],v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));
    min_x = (int)std::floor(min_x);
    max_x = (int)std::ceil(max_x);
    min_y = (int)std::floor(min_y);
    max_y = (int)std::ceil(max_y);
    bool MSAA = true;
if (MSAA) {
	// 格子里的细分四个小点坐标
	std::vector<Eigen::Vector2f> pos
	{
		{0.25,0.25},
		{0.75,0.25},
		{0.25,0.75},
		{0.75,0.75},
	};
	for (int x = min_x; x <= max_x; x++) {
		for (int y = min_y; y <= max_y; y++) {
			// 记录最小深度
			float minDepth = FLT_MAX;
			// 四个小点中落入三角形中的点的个数
			int count = 0;
			// 对四个小点坐标进行判断 
			for (int i = 0; i < 4; i++) {
				// 小点是否在三角形内
				if (insideTriangle((float)x + pos[i][0], (float)y + pos[i][1], t.v)) {
					// 如果在，对深度z进行插值
					auto tup = computeBarycentric2D((float)x + pos[i][0], (float)y + pos[i][1], t.v);
					float alpha;
					float beta;
					float gamma;
					std::tie(alpha, beta, gamma) = tup;
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
                    //更新z深度
					minDepth = std::min(minDepth, z_interpolated);
					count++;
				}
			}
			if (count != 0) {
				if (depth_buffer[get_index(x, y)] > minDepth) {
					Vector3f color = t.getColor() * count / 4.0;
					Vector3f point(3);
					point << (float)x, (float)y, minDepth;
					// 替换深度
					depth_buffer[get_index(x, y)] = minDepth;
					// 修改颜色
					set_pixel(point, color);
				}
			}
		}
	}
}
else {
	for (int x = min_x; x <= max_x; x++) {
		for (int y = min_y; y <= max_y; y++) {
			if (insideTriangle((float)x + 0.5, (float)y + 0.5, t.v)) {
				auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
				float alpha;
				float beta;
				float gamma;
				std::tie(alpha, beta, gamma) = tup;
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				if (depth_buffer[get_index(x, y)] > z_interpolated) {
					Vector3f color = t.getColor();
					Vector3f point(3);
					point << (float)x, (float)y, z_interpolated;
					depth_buffer[get_index(x, y)] = z_interpolated;
					(point, color);
				}
			}
		}
	}
}
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

void RenderEngine::Engine::draw(RenderEngine::position_buf_id pos_buffer, RenderEngine::index_buf_id ind_buffer, RenderEngine::col_buf_id col_buffer,renderType type)
{  
    //判断类型必须为三角形
    if (type != RenderEngine::renderType::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    //将传入的数据根据id填充进本地pos，index数据
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& index = ind_buf[ind_buffer.index_id];
    auto& col = col_buf[col_buffer.col_id];
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
        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        
        //绘制三角形边框
        rasterize_triangle(t);
    }
}

//渲染三角形边框
/**
void RenderEngine::Engine::renderFrame(const Triangle &t)
{
    draw_Line(t.c(), t.a());
    draw_Line(t.c(), t.b());
    draw_Line(t.b(), t.a());
}**/
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

