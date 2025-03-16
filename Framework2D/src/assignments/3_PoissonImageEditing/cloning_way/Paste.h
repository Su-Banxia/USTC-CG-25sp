#pragma once

#include "cloning_way.h"

namespace USTC_CG
{
    class Paste : public CloningWay
    {
    public:
        Paste(
            shared_ptr<Image> src_img, 
            shared_ptr<Image> tar_img, 
            shared_ptr<Image> src_selected_mask)
            : CloningWay(src_img, tar_img, src_selected_mask){};


        virtual ~Paste() = default;

        shared_ptr<Image> solve() override;

    private:
        
    };
}
