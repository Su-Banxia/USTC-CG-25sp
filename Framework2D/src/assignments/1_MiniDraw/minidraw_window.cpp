#include "minidraw_window.h"

#include <iostream>

namespace USTC_CG
{
MiniDraw::MiniDraw(const std::string& window_name) : Window(window_name)
{
    p_canvas_ = std::make_shared<Canvas>("Widget.Canvas");
}

MiniDraw::~MiniDraw()
{
}

void MiniDraw::draw()
{
    draw_canvas();
}

void MiniDraw::draw_canvas()
{
    // Set a full screen canvas view
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    if (ImGui::Begin(
            "Canvas",
            &flag_show_canvas_view_,
            ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoBackground))
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Shapes"))
            {
                if (ImGui::MenuItem("Line"))
                {
                    std::cout << "Set shape to Line" << std::endl;
                    p_canvas_->set_line();
                }
                add_tooltips("Set shape to Line.");
                if (ImGui::MenuItem("Rect"))
                {
                    std::cout << "Set shape to Rect" << std::endl;
                    p_canvas_->set_rect();
                }
                add_tooltips("Set shape to Rect.");
                if (ImGui::MenuItem("Ellipse"))
                {
                    std::cout << "Set shape to Ellipse" << std::endl;
                    p_canvas_->set_ellipse();
                }
                add_tooltips("Set shape to Ellipse.");
                if (ImGui::MenuItem("Polygon"))
                {
                    std::cout << "Set shape to Polygon" << std::endl;
                    p_canvas_->set_polygon();
                }
                add_tooltips("Set shape to Polygon.");
                if (ImGui::MenuItem("FreeHand"))
                {
                    std::cout << "Set shape to FreeHand" << std::endl;
                    p_canvas_->set_freehand();
                }
                add_tooltips("Set shape to FreeHand.");
                ImGui::EndMenu();
                
            }
            ImGui::Separator();
            ImGui::EndMainMenuBar();
        }

        // HW1_TODO: More primitives
        //    - Ellipse
        //    - Polygon
        //    - Freehand(optional)
        
        // Canvas component
        ImGui::Text("Press left mouse to add shapes.");
        // Set the canvas to fill the rest of the window
        const auto& canvas_min = ImGui::GetCursorScreenPos();
        const auto& canvas_size = ImGui::GetContentRegionAvail();
        p_canvas_->set_attributes(canvas_min, canvas_size);
        p_canvas_->draw();
    }
    ImGui::End();
}
void MiniDraw::add_tooltips(std::string desc)
{
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}
}  // namespace USTC_CG