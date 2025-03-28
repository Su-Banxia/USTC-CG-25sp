#include "CotangentWeights.h"

namespace USTC_CG
{

double CotangentWeights::calculator_weight()
{
    const OpenMesh::SmartVertexHandle &vertex_a = alpha_edge2.from(),
        &vertex_a1 = alpha_edge1.to(), &vertex_a2 = alpha_edge2.to();

    const auto &tempA = Mesh.point(vertex_a),
               &tempA1 = Mesh.point(vertex_a1),
               &tempA2 = Mesh.point(vertex_a2);
    Vector3d vec_a1 = { tempA1[0] - tempA[0], 
                        tempA1[1] - tempA[1],
                        tempA1[2] - tempA[2] };
    Vector3d vec_a2 = { tempA2[0] - tempA[0], 
                        tempA2[1] - tempA[1],
                        tempA2[2] - tempA[2] };

    double cot_alpha = (vec_a1.dot(vec_a2)) / vec_a1.cross(vec_a2).norm();

    const OpenMesh::SmartVertexHandle &vertex_b = beta_edge2.from(),
        &vertex_b1 = beta_edge1.to(), &vertex_b2 = beta_edge2.to();

    const auto &tempB = Mesh.point(vertex_b),
               &tempB1 = Mesh.point(vertex_b1),
               &tempB2 = Mesh.point(vertex_b2);
    Vector3d vec_b1 = { tempB1[0] - tempB[0], 
                        tempB1[1] - tempB[1],
                        tempB1[2] - tempB[2] };
    Vector3d vec_b2 = { tempB2[0] - tempB[0], 
                        tempB2[1] - tempB[1],
                        tempB2[2] - tempB[2] };

    double cot_beta = (vec_b1.dot(vec_b2)) / vec_b1.cross(vec_b2).norm();

    return cot_alpha + cot_beta;
}

void CotangentWeights::find_alpha_beta_edge()
{
    OpenMesh::SmartHalfedgeHandle link_edge1, link_edge2;
    for (const auto& edge_out : v_i.outgoing_halfedges())
    {
        if (edge_out.to() == v_j)
        {
            link_edge1 = edge_out;
            link_edge2 = edge_out.opp();
            break;
        }
    }
    alpha_edge1 = link_edge1.next().opp();
    alpha_edge2 = link_edge1.next().next();
    beta_edge1 = link_edge2.next().opp();
    beta_edge2 = link_edge2.next().next();
}


}