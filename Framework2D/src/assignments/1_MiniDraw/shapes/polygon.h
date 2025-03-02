#pragma once

#include "shape.h"

#include <vector>

namespace USTC_CG
{
class Polygon : public Shape
  {
     public:
      Polygon() = default;

      Polygon(
          std::vector<float> x_list, 
          std::vector<float> y_list,
          bool is_closed_ = false) 
          : x_list_(x_list),
            y_list_(y_list),
            is_closed(is_closed_)
      {
      }

      virtual ~Polygon() = default;
  
      void draw(const Config& config) const;
      //void draw_closed_polygon(const Config& config) const;
      void update(float x, float y);
      void add_control_point(float x, float y);
      void set_closed();
  
     private:
      std::vector<float> x_list_, y_list_;
      float real_time_x, real_time_y;
      bool is_closed;       //If the polygon is closed now
  };
}; // namespace USTC_CG


