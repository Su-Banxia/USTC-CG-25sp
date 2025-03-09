// HW2_TODO: Implement the IDWWarper class
#pragma once

#include <Eigen/Dense>

#include "warper.h"

using namespace Eigen;
using namespace std;

#define epsilon 1e-10
#define mu_IDW 2

namespace USTC_CG
{
class IDWWarper : public Warper
{
   public:
    IDWWarper(  const vector<ImVec2>& start_points, 
                const vector<ImVec2>& end_points );

    virtual ~IDWWarper() = default;

    pair<int, int> warp(int x, int y) const override;

   private:
    //some help function to compute the final function
    void get_Start_vector();
    void get_End_vector();
    void get_Matrix_T_list();

    Matrix2d get_Matrix_A(int i) const;
    Matrix2d get_Matrix_B(int i) const;
    double get_sigma_i(int i, const Vector2d& p) const;
    double get_omega_i(int i, const Vector2d& p) const;
    Vector2d get_f_i(int i, const Vector2d& p) const;
    Vector2d get_final_func(const Vector2d& p) const;

   private:
    vector<Matrix2d> T_list;
    vector<Vector2d> start_list, end_list;
};
}  // namespace USTC_CG