#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "common/image_widget.h"

using namespace std;
using namespace USTC_CG;


namespace USTC_CG
{
    class CloningWay
    {
    public:
        CloningWay(
            shared_ptr<Image> src_img, 
            shared_ptr<Image> tar_img, 
            shared_ptr<Image> src_selected_mask)
            :   source_image(src_img), 
                target_image(tar_img), 
                mask_image(src_selected_mask) {};
        
        virtual ~CloningWay() = default;

        // Solve Poisson Equations，Return a result image of Clone 
        virtual shared_ptr<Image> solve() = 0; 

        shared_ptr<Image> get_Source()
        {
            return source_image;
        }
        shared_ptr<Image> get_Target()
        {
            return target_image;
        }
        shared_ptr<Image> get_mask()
        {
            return mask_image;
        }

    private:
        shared_ptr<Image> source_image;
        shared_ptr<Image> target_image;
        shared_ptr<Image> mask_image;
        
    };
}
