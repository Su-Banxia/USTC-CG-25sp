#pragma once

#include <memory>

#include "common/window.h"
#include "canvas_widget.h"

namespace USTC_CG
{
class MiniDraw : public Window
{
   public:
    explicit MiniDraw(const std::string& window_name);
    ~MiniDraw();

    void draw();

   private:
    void draw_canvas();
    void draw_test();

    std::shared_ptr<Canvas> p_canvas_ = nullptr;

    bool flag_show_canvas_view_ = true;
};
}  // namespace USTC_CG