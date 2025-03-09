#include <cmath>
#include <vector>

#include "RBF_warper.h"

namespace USTC_CG
{
    RBFWarper::RBFWarper( const vector<ImVec2>& start_points, 
        const vector<ImVec2>& end_points )
        :   Warper(start_points, end_points)
    {
        get_Start_vector();
        get_End_vector();
        get_r_list();
        Build_Matrix_M();
        Solve_Equation();
    }
    pair<int, int> RBFWarper::warp(int x, int y) const 
    {   
        Vector2d temp(x, y);
        temp = get_final_func(temp);

        pair<int, int> ret(static_cast<int>(temp(0)), static_cast<int>(temp(1)));

        return ret;
    }
    void RBFWarper::get_Start_vector()
    {
        start_list.resize(start_points_.size());
        for (int i = 0; i < start_points_.size(); ++i)
        {
            start_list[i] <<    start_points_[i].x, 
                                start_points_[i].y;
        }
    }
    void RBFWarper::get_End_vector()
    {
        end_list.resize(end_points_.size());
        for (int i = 0; i < end_points_.size(); ++i)
        {
            end_list[i] <<  end_points_[i].x, 
                            end_points_[i].y;
        }
    }
    void RBFWarper::get_r_list()
    {
        r_list.resize(start_points_.size());

        double min_distance = 1 / epsilon;
        double distance = 1 / epsilon;
        for (int i = 0; i < start_points_.size(); ++i)
        {
            min_distance = 1 / epsilon;

            for (int j = 0; j < start_points_.size(); ++j)
            {
                if (i == j) continue;

                distance = (start_list[i] - start_list[j]).norm();
                if (distance < min_distance)
                {
                    min_distance = distance;
                }
            }

            r_list[i] = min_distance;
        }
    }
    // To solve Mx = v, we can get matrix A, bias and alpha_list
    void RBFWarper::Build_Matrix_M()
    {
        size_t n = start_points_.size();

        Matrix_M.resize(2 * n + 6, 2 * n + 6);
        Matrix_M.setZero();

        v.resize(2 * n + 6);
        v.setZero();

        Block<MatrixXd> affine_block = Matrix_M.block(0, 0, 2 * n, 6);
        Block<MatrixXd> radial_block = Matrix_M.block(0, 6, 2 * n, 2 * n);
        Block<MatrixXd> constr_block = Matrix_M.block(2 * n, 6, 6, 2 * n);

        for (int i = 0; i < n; ++i)
        {
            double p_x = start_list[i].x();
            double p_y = start_list[i].y();
            double q_x = end_list[i].x();
            double q_y = end_list[i].y();

            // fill in affine_block part
            int row_x_num = 2 * i;

            affine_block(row_x_num, 0) = p_x;
            affine_block(row_x_num, 1) = p_y;
            affine_block(row_x_num, 4) = 1.0;
            v(row_x_num) = q_x;

            int row_y_num = 2 * i + 1;

            affine_block(row_y_num, 2) = p_x;
            affine_block(row_y_num, 3) = p_y;
            affine_block(row_y_num, 5) = 1.0;
            v(row_y_num) = q_y;

            //fill in radial_block part
            for (int j = 0; j < n; j++)
            {
                double g_j = get_g_d(j, (start_list[i] - start_list[j]).norm());

                radial_block(row_x_num, 2 * j) = g_j;
                radial_block(row_y_num, 2 * j + 1) = g_j;
            }
        }

        //fill in constr_block part
        for (int i = 0; i < n; ++i)
        {
            double p_x = start_list[i].x();
            double p_y = start_list[i].y();

            constr_block(0, 2 * i) = p_x;
            constr_block(1, 2 * i) = p_y;
            constr_block(2, 2 * i) = 1.0;

            constr_block(3, 2 * i + 1) = p_x;
            constr_block(4, 2 * i + 1) = p_y;
            constr_block(5, 2 * i + 1) = 1.0;
        }
    }
    void RBFWarper::Solve_Equation()
    {
        VectorXd solution = Matrix_M.householderQr().solve(v);

        alpha_list.resize(start_points_.size());

        Matrix_A << solution(0), solution(1), 
                    solution(2), solution(3);

        bias << solution(4), solution(5);

        for (int i = 0; i < start_points_.size(); ++i)
        {
            alpha_list[i].x() = solution(6 + 2 * i);
            alpha_list[i].y() = solution(7 + 2 * i);
        }
    }
    double RBFWarper::get_g_d(const int& i, const double& d) const
    {
        return pow( ( d * d + r_list[i] * r_list[i]) , mu_RBF / 2.0);
    }
    Vector2d RBFWarper::get_final_func(const Vector2d& p) const
    {
        Vector2d ret = Vector2d::Zero();

        for (int i = 0; i < start_points_.size(); ++i)
        {
            ret += alpha_list[i] * get_g_d( i, (p - start_list[i]).norm() );
        }

        ret += Matrix_A * p + bias;
        
        return ret;
    }
}
