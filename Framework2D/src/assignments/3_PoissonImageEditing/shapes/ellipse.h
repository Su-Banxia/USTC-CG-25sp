#pragma once

#include "shape.h"

namespace USTC_CG
{
class Ellipse : public Shape
{
   public:
    Ellipse() = default;

    // Constructor to initialize a line with start and end coordinates
    Ellipse(
        float start_point_x,
        float start_point_y,
        float end_point_x,
        float end_point_y)
        : start_point_x_(start_point_x),
          start_point_y_(start_point_y),
          end_point_x_(end_point_x),
          end_point_y_(end_point_y)
    {
    }

    virtual ~Ellipse() = default;

    // Overrides draw function to implement line-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override;

    std::vector<std::pair<int, int>> get_interior_pixels() const override;

   private:
    float start_point_x_, start_point_y_, end_point_x_, end_point_y_;
};
}  // namespace USTC_CG
