#pragma once

#include "GCore/Components/MeshOperand.h"
#include "../../../source/Editor/geometry_nodes/geom_node_base.h"
// #include "geom_node_base.h"
#include "GCore/util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <vector>

namespace USTC_CG
{

class WeightCalculator
{

public:
    WeightCalculator(
        const USTC_CG::PolyMesh &MyMesh,
        const OpenMesh::SmartVertexHandle &V_i,
        const OpenMesh::SmartVertexHandle &V_j):
        Mesh(MyMesh), v_i(V_i), v_j(V_j){};
    virtual ~WeightCalculator() = default;

    virtual double calculator_weight() = 0;
protected:
    const USTC_CG::PolyMesh &Mesh;
    const OpenMesh::SmartVertexHandle &v_i;
    const OpenMesh::SmartVertexHandle &v_j;

};


}

