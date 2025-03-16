#include "freeHand.h"

#include "line.h"

#include <imgui.h>

#include <cmath>

namespace USTC_CG
{
// Draw the open polygon using ImGui
void FreeHand::draw(const Config& config) const  
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    size_t vector_size = x_list_.size();

    for ( int i = 0; i < vector_size - 1; i++ )
    {
        Line line_now(  x_list_[i],         
                        y_list_[i], 
                        x_list_[ (i + 1) ],     
                        y_list_[ (i + 1) ] );

            line_now.draw(config);
    }
}  

void FreeHand::add_control_point(float x, float y)
{
    x_list_.push_back(x);
    y_list_.push_back(y);
}

void FreeHand::update(float x, float y)
{
    x_list_[x_list_.size() - 1] = x;
    y_list_[y_list_.size() - 1] = y;
}

std::vector<std::pair<int, int>> FreeHand::get_interior_pixels() const
{
    std::vector<std::pair<int, int>> test;
    return test;
}


}  // namespace USTC_CG
