#include "polygon.h"

#include "line.h"

#include <imgui.h>

#include <cmath>

namespace USTC_CG
{
// Draw the open polygon using ImGui
void Polygon::draw(const Config& config) const  
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    size_t vector_size = x_list_.size();

    if (is_closed == true)
    {
        
        for ( int i = 0; i < vector_size; i++ )
        {
            Line line_now(  x_list_[i],         
                            y_list_[i], 
                            x_list_[ (i + 1) % vector_size ],     
                            y_list_[ (i + 1) % vector_size ] );    // 取余保证首尾相接

            line_now.draw(config);
        }
    }
    else
    {
        for ( int i = 0; i < vector_size - 1; i++ )
        {
            Line line_now(  x_list_[i],         
                            y_list_[i], 
                            x_list_[ (i + 1) ],     
                            y_list_[ (i + 1) ] );

            line_now.draw(config);
        }
        Line line_now ( x_list_[ x_list_.size() - 1 ],
                        y_list_[ y_list_.size() - 1 ],
                        real_time_x, real_time_y );
        line_now.draw(config);
    }    
}

void Polygon::set_closed()
{
    is_closed = true;
}

void Polygon::add_control_point(float x, float y)
{
    x_list_.push_back(x);
    y_list_.push_back(y);
}

void Polygon::update(float x, float y)
{
    real_time_x = x;
    real_time_y = y;
}

}  // namespace USTC_CG
