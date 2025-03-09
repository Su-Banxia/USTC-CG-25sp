// HW2_TODO: Please implement the abstract class Warper
// 1. The Warper class should abstract the **mathematical mapping** involved in
// the warping problem, **independent of image**.
// 2. The Warper class should have a virtual function warp(...) to be called in
// our image warping application.
//    - You should design the inputs and outputs of warp(...) according to the
//    mathematical abstraction discussed in class.
//    - Generally, the warping map should map one input point to another place.
// 3. Subclasses of Warper, IDWWarper and RBFWarper, should implement the
// warp(...) function to perform the actual warping.
#pragma once

#include <iostream>
#include <vector>
#include <annoylib.h>

#include "common/image_widget.h"

namespace USTC_CG
{
class Warper
{
   public:

    Warper() = delete;

    Warper( const std::vector<ImVec2>& start_points, 
            const std::vector<ImVec2>& end_points );

    virtual ~Warper() = default;

    virtual std::pair<int, int> warp(int x, int y) const = 0;

   protected:
    std::vector<ImVec2> start_points_, end_points_;
};
}  // namespace USTC_CG