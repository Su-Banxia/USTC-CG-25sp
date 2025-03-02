#pragma once

#include "shape.h"

#include <vector>

namespace USTC_CG
{
class FreeHand : public Shape
  {
     public:
      FreeHand() = default;

      FreeHand(
          std::vector<float> x_list, 
          std::vector<float> y_list) 
          : x_list_(x_list),
            y_list_(y_list)
      {
      }

      virtual ~FreeHand() = default;
  
      void draw(const Config& config) const;
      void add_control_point(float x, float y);
      void update(float x, float y);
  
     private:
      std::vector<float> x_list_, y_list_;
  };
}; // namespace USTC_CG


