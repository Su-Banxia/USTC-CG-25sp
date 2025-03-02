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
    draw_test();
//    draw_canvas();
}

void MiniDraw::draw_test()
{
    const ImGuiViewport* commonport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(commonport->WorkPos);
    ImGui::SetNextWindowSize(commonport->WorkSize);
    if (ImGui::Begin("Canvas", &flag_show_canvas_view_, ImGuiWindowFlags_NoDecoration))
    {
        ImGui::Text("This is a canvas window");
        if (ImGui::Button("Line"))
        {
            // 按钮单击后执行相应操作
            std::cout << "Set shape to Line" << std::endl;
        }
        if (ImGui::Button("Rect"))
        {
            std::cout << "Set shape to Rect" << std::endl;
        }
        
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        // 设置画布的大小和位置，填充窗口剩余（上一个按钮添加之后）的空间
        const auto& canvas_min = ImGui::GetCursorScreenPos();
        const auto& canvas_size = ImGui::GetContentRegionAvail();
        const auto& canvas_max = ImVec2(canvas_min.x + canvas_size.x, canvas_min.y + canvas_size.y);
        // 绘制（填充的）矩形作为背景版（颜色可以自行调整）
        draw_list->AddRectFilled(canvas_min, canvas_max, IM_COL32(50, 50, 50, 255));
        // 绘制矩形边框（颜色可以自行调整）
        draw_list->AddRect(canvas_min, canvas_max, IM_COL32(255, 255, 255, 255));


        ImVec2 start_point_, end_point_;  // 线段的起点和终点
        bool draw_status_ = false;       // 是否处于绘制状态

        ImGui::SetCursorScreenPos(canvas_min);
        ImGui::InvisibleButton(
            "Canvas.InvisibleButton", canvas_size, ImGuiButtonFlags_MouseButtonLeft);
        bool is_hovered = ImGui::IsItemHovered();
        // 鼠标交互
        ImGuiIO& io = ImGui::GetIO();
        const ImVec2 mouse_pos = io.MousePos;
        if (is_hovered && !draw_status_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            draw_status_ = true;
            start_point_ = end_point_ = mouse_pos;
        }
        if (draw_status_)
        {
            end_point_ = mouse_pos;
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
                draw_status_ = false;
        }
        // 线段绘制
        draw_list->AddLine(start_point_, end_point_, IM_COL32(255, 0, 0, 255), 2.0f);
    }
    ImGui::End();
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
        // Buttons for shape types
        if (ImGui::Button("Line"))
        {
            std::cout << "Set shape to Line" << std::endl;
            p_canvas_->set_line();
        }
        ImGui::SameLine();
        if (ImGui::Button("Rect"))
        {
            std::cout << "Set shape to Rect" << std::endl;
            p_canvas_->set_rect();
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
}  // namespace USTC_CG