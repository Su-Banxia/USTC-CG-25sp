#pragma once
#include "WeightCalculator.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

namespace USTC_CG
{

class CotangentWeights : public WeightCalculator
{
public:
    CotangentWeights(
        const USTC_CG::PolyMesh &MyMesh,
        const OpenMesh::SmartVertexHandle &V_i,
        const OpenMesh::SmartVertexHandle &V_j):
        WeightCalculator(MyMesh, V_i, V_j)
    {
        find_alpha_beta_edge();
    };

    virtual double calculator_weight() override;
private:
    OpenMesh::SmartHalfedgeHandle alpha_edge1, alpha_edge2;
    OpenMesh::SmartHalfedgeHandle beta_edge1, beta_edge2;

    void find_alpha_beta_edge();
};

}