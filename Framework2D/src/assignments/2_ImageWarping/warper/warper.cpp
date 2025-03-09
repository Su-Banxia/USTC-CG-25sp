#include "warper.h"

namespace USTC_CG
{
    Warper::Warper( const std::vector<ImVec2>& start_points, 
                    const std::vector<ImVec2>& end_points )
            :   start_points_(start_points), end_points_(end_points)
    {
        if (start_points_.size() != end_points_.size() || start_points_.empty())
        {
            throw std::invalid_argument("Points should be consistent and not empty");
        }
    }
}