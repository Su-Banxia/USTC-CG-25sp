#pragma once

#include "cloning_way.h"
#include <Eigen/Sparse>

using namespace Eigen;

namespace USTC_CG
{
    class SeamlessClone : public CloningWay
    {
    public:
        SeamlessClone(
            shared_ptr<Image> src_img, 
            shared_ptr<Image> tar_img, 
            shared_ptr<Image> src_selected_mask,
            int Soffset_x, int Soffset_y,
            int Toffset_x, int Toffset_y)
            : CloningWay(src_img, tar_img, src_selected_mask),
            Soffset_x_(Soffset_x), Soffset_y_(Soffset_y), 
            Toffset_x_(Toffset_x), Toffset_y_(Toffset_y)
            {
                get_mask_size();
                build_matrix();
                build_B();
            };


        virtual ~SeamlessClone() = default;

        shared_ptr<Image> solve() override;

        void get_mask_size();

        bool in_Mask(int x, int y);

        int Source_offset_x()
        {
            return Soffset_x_;
        }
        int Source_offset_y()
        {
            return Soffset_y_;
        }
        int Target_offset_x()
        {
            return Toffset_x_;
        }
        int Target_offset_y()
        {
            return Toffset_y_;
        }

    private:
        void build_matrix();
        void build_B();
        vector<unsigned char> get_source_pixel(int x, int y);

        SimplicialLDLT<SparseMatrix<double>> solver;
        VectorXd B_r, B_g, B_b;

        int Soffset_x_, Soffset_y_;         // seleted region's position on source
        int Toffset_x_, Toffset_y_;         // seleted region's position on target
        int mask_h, mask_w;                 // the size of mask region
        bool is_matrix_initialized = false;
        bool is_b_initialized = false;
    };
}
