// HW2_TODO: Implement the RBFWarper class
#pragma once

#include <Eigen/Dense>

#include "warper.h"

using namespace Eigen;
using namespace std;

#define epsilon 1e-10
#define mu_RBF 1


namespace USTC_CG
{
// We enforce orthogonality between 
// the radial component and the affine component.
class RBFWarper : public Warper
{
   public:
    RBFWarper(  const vector<ImVec2>& start_points, 
                const vector<ImVec2>& end_points );

    virtual ~RBFWarper() = default;

    pair<int, int> warp(int x, int y) const override;

   private:
    //some help function to compute the final function
    void get_Start_vector();
    void get_End_vector();
    void get_r_list();

    void Build_Matrix_M();
    void Solve_Equation();

    double get_g_d(const int& i, const double& d) const;

    Vector2d get_final_func(const Vector2d& p) const;

   private:
    vector<Vector2d> start_list, end_list;
    vector<double> r_list;

    MatrixXd Matrix_M;
    VectorXd v;
    Matrix2d Matrix_A;
    Vector2d bias;
    vector<Vector2d> alpha_list;
};
}  // namespace USTC_CG