#include "Mixing_Gradients.h"

namespace USTC_CG
{
shared_ptr<Image> MixingGradients::solve()
{
    shared_ptr<Image> result = make_shared<Image>(*get_Source());

    int W = mask_w;
    int H = mask_h;

    vector<unsigned char> values(W * H * 3);

    int W_tar = get_Target()->width(),
        H_tar = get_Target()->height();

    for (int channel = 0; channel < 3; ++channel)
    {

    VectorXd f;
    switch (channel)
    {
    case 0:
        f = solver.solve(B_r);
        break;
    case 1:
        f = solver.solve(B_g);
        break;
    case 2:
        f = solver.solve(B_b);
        break;
    default:
        break;
    }

    if (solver.info() != Success) 
    {
        throw runtime_error("Matrix solving failed");
    }

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            values[(y * W + x) * 3 + channel] = 
                static_cast<unsigned char>( clamp(f(y * W + x), 0.0, 255.0) );
        }
    }

    }

    int W_source = get_Source()->width();
    int H_source = get_Source()->height();
    for (int y = 0; y < H; ++y)
    {
        for (int x =0; x < W; ++x)
        {
            int x_source = x + Source_offset_x();
            int y_source = y + Source_offset_y();
            if (x_source >= 0 && x_source < W_tar && y_source >= 0 && y_source < H_tar)
            {
                int idx = (y * W + x) * 3;
                vector<unsigned char> pixel = {
                    values[idx],
                    values[idx + 1], 
                    values[idx + 2]
                };
                result->set_pixel(  min( max(x_source,0), W_source-1 ), 
                                    min( max(y_source,0), H_source-1 ), pixel);
            }
            
        }
    }

    return result;
};

void MixingGradients::get_mask_size()
{ 
    shared_ptr<Image> mask = get_mask();
    int min_x = mask->width(), min_y = mask->height();
    int max_x = 0, max_y = 0;

    for (int y = 0; y < mask->height(); ++y) {
        for (int x = 0; x < mask->width(); ++x) {
            if (mask->get_pixel(x, y)[0] == 255) {
                min_x = std::min(min_x, x);
                min_y = std::min(min_y, y);
                max_x = std::max(max_x, x);
                max_y = std::max(max_y, y);
            }
        }
    }

    mask_w = (max_x >= min_x) ? (max_x - min_x + 1) : 0;
    mask_h = (max_y >= min_y) ? (max_y - min_y + 1) : 0;
}

void MixingGradients::build_matrix()
{
    int W = mask_w;
    int H = mask_h;

    SparseMatrix<double> Matrix_A(W * H, W * H);

    vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(5 * W * H);

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            int idx = y * W + x;
            triplet_list.push_back(Triplet<double>(idx, idx, 4.0));
            if (x > 0)
            {
                triplet_list.push_back(Triplet<double>(idx, y * W + (x-1), -1.0));  // left
            }
            if (x < W-1)
            {
                triplet_list.push_back(Triplet<double>(idx, y * W + (x+1), -1.0));  // right
            }
            if (y > 0)
            {
                triplet_list.push_back(Triplet<double>(idx, (y-1) * W + x, -1.0));  // up    
            }
            if (y < H-1)
            {
                triplet_list.push_back(Triplet<double>(idx, (y+1) * W + x, -1.0));  // down
            }
                
        }
    }

    Matrix_A.setFromTriplets(triplet_list.begin(), triplet_list.end());
    solver.compute(Matrix_A);

    is_matrix_initialized = true;
}

void MixingGradients::build_B()
{
    int W = mask_w;
    int H = mask_h;

    for (int channel = 0; channel < 3; ++channel)
    {

    VectorXd B(W * H);

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            int idx = y * W + x;
            int g_diff, fstar_diff;
            B(idx) = 0;

            g_diff = get_source_pixel(x, y)[channel] - get_source_pixel(x, y - 1)[channel];
            fstar_diff = get_target_pixel(x, y)[channel] - get_target_pixel(x, y - 1)[channel];
            B(idx) += (abs(g_diff) > abs(fstar_diff) ? g_diff : fstar_diff);

            g_diff = get_source_pixel(x, y)[channel] - get_source_pixel(x, y + 1)[channel];
            fstar_diff = get_target_pixel(x, y)[channel] - get_target_pixel(x, y + 1)[channel];
            B(idx) += (abs(g_diff) > abs(fstar_diff) ? g_diff : fstar_diff);
            
            g_diff = get_source_pixel(x, y)[channel] - get_source_pixel(x - 1, y)[channel];
            fstar_diff = get_target_pixel(x, y)[channel] - get_target_pixel(x - 1, y)[channel];
            B(idx) += (abs(g_diff) > abs(fstar_diff) ? g_diff : fstar_diff);
            
            g_diff = get_source_pixel(x, y)[channel] - get_source_pixel(x + 1, y)[channel];
            fstar_diff = get_target_pixel(x, y)[channel] - get_target_pixel(x + 1, y)[channel];
            B(idx) += (abs(g_diff) > abs(fstar_diff) ? g_diff : fstar_diff);
            
            
            // meet edge of mask
            if (x == 0) //left edge
            {
                B(idx) += get_target_pixel(x - 1, y)[channel];
            }
            if (y == 0) //up edge
            {
                B(idx) += get_target_pixel(x, y - 1)[channel];
            }
            if (x == W-1) //right edge
            {
                B(idx) += get_target_pixel(x+1, y)[channel];
            }
            if (y == H-1) //down edge
            {
                B(idx) += get_target_pixel(x, y+1)[channel];
            }
                
        }
    }

    switch (channel)
    {
    case 0:
        B_r = B;
        break;
    case 1:
        B_g = B;
        break;
    case 2:
        B_b = B;
        break;
    default:
        break;
    }
    }
    is_b_initialized = true;
}

vector<unsigned char> MixingGradients::get_source_pixel(int x, int y)
{
    int real_x = min(max(x + Source_offset_x(), 0), get_Source()->width());
    int real_y = min(max(y + Source_offset_y(), 0), get_Source()->width());

    if (real_x < 0 || real_x >= get_Source()->width() || 
        real_y < 0 || real_y >= get_Source()->height()) 
    {
        return vector<unsigned char>(3, 0); // default
    }

    return get_Source()->get_pixel(real_x, real_y);
}

vector<unsigned char> MixingGradients::get_target_pixel(int x, int y)
{
    int real_x = x + Target_offset_x();
    int real_y = y + Target_offset_y();

    if (real_x < 0 || real_x >= get_Target()->width() || 
        real_y < 0 || real_y >= get_Target()->height()) 
    {
        return vector<unsigned char>(3, 0); // default
    }

    return get_Target()->get_pixel(real_x, real_y);
}

bool MixingGradients::in_Mask(int x, int y)
{
    auto mask = get_mask();

    if (!mask) 
    {
        return true;
    }

    int x_src = x - Target_offset_x();
    int y_src = y - Target_offset_y();

    if (x_src < 0 || x_src >= mask->width() || 
        y_src < 0 || y_src >= mask->height()) {
        return false;
    }

    return true;
}

}
