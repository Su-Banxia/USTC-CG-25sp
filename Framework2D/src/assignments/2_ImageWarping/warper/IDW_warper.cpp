#include <cmath>
#include <vector>

#include "IDW_warper.h"

namespace USTC_CG
{
    IDWWarper::IDWWarper( const vector<ImVec2>& start_points, 
        const vector<ImVec2>& end_points )
        :   Warper(start_points, end_points)
    {
        get_Start_vector();
        get_End_vector();
        get_Matrix_T_list();
    }
    pair<int, int> IDWWarper::warp(int x, int y) const 
    {   
        Vector2d temp(x, y);
        temp = get_final_func(temp);

        pair<int, int> ret(static_cast<int>(temp(0)), static_cast<int>(temp(1)));

        return ret;
    }
    void IDWWarper::get_Start_vector()
    {
        start_list.resize(start_points_.size());
        for (int i = 0; i < start_points_.size(); ++i)
        {
            start_list[i] <<    start_points_[i].x, 
                                start_points_[i].y;
        }
    }
    void IDWWarper::get_End_vector()
    {
        end_list.resize(end_points_.size());
        for (int i = 0; i < end_points_.size(); ++i)
        {
            end_list[i] <<  end_points_[i].x, 
                            end_points_[i].y;
        }
    }
    void IDWWarper::get_Matrix_T_list()
    {
        T_list.resize(start_points_.size());

        Matrix2d A, B, T;
        A = B = T = Matrix2d::Zero();

        double lambda = 1e-5;   //regularization

        for (int i = 0; i < start_points_.size(); ++i)
        {
            A = get_Matrix_A(i) + lambda * Matrix2d::Identity();
            B = get_Matrix_B(i);
            
            T_list[i] = B * A.inverse();
        }
    }
    Matrix2d IDWWarper::get_Matrix_A(int i) const
    {
        Matrix2d ret = Matrix2d::Zero();
        for (int j = 0; j < start_points_.size(); ++j)
        {
            if (i != j)
            {
                double sigma_i = get_sigma_i(i, start_list[j]);
                Matrix2d temp = (start_list[j] - start_list[i]) * (start_list[j] - start_list[i]).transpose();

                ret += get_sigma_i(i, start_list[j]) * 
                    ((start_list[j] - start_list[i]) * (start_list[j] - start_list[i]).transpose());
            }
        }
        return ret;
    }
    Matrix2d IDWWarper::get_Matrix_B(int i) const
    {
        Matrix2d ret = Matrix2d::Zero();
        for (int j = 0; j < start_points_.size(); ++j)
        {
            if (i != j)
            {
                double test = get_sigma_i(i, start_list[j]);
                ret += get_sigma_i(i, start_list[j]) *
                    ((end_list[j] - end_list[i]) * 
                    (start_list[j] - start_list[i]).transpose());
            }
        }
        return ret;
    }
    double IDWWarper::get_sigma_i(int i, const Eigen::Vector2d& p) const
    {
        double distance = max((p - start_list[i]).norm(), epsilon); //avoid divided by 0
        return 1.0 / pow(distance, mu_IDW);
    }
    double IDWWarper::get_omega_i(int i, const Vector2d& p) const
    {
        if (p == start_list[i])
        {
            return 1.0;
        }

        double sum = 0.0;

        for (int j = 0; j < start_points_.size(); ++j)
        {
            sum += get_sigma_i(j, p);
        }

        return get_sigma_i(i, p) / sum;
    }
    Vector2d IDWWarper::get_f_i(int i, const Vector2d& p) const
    {
        return end_list[i] + T_list[i] * (p - start_list[i]);
    }
    Vector2d IDWWarper::get_final_func(const Vector2d& p) const
    {
        Vector2d ret = Vector2d::Zero();

        for (int i = 0; i < start_points_.size(); i++)
        {
            double omega_i = get_omega_i(i, p);
            Vector2d f_i = get_f_i(i, p);

            // avoid pollution
            if (isnan(omega_i) || omega_i <= 0.0) continue;
            if (f_i.array().isNaN().any() || f_i.array().isInf().any()) continue;
            
            ret += omega_i * f_i;
        }
        
        return ret;
    }
}
