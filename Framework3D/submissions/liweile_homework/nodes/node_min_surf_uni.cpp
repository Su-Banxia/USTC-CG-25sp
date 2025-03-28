#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h" 
#include "UniformWeights.h"
#include "CotangentWeights.h"

#include <cmath>
#include <time.h>
#include <Eigen/Sparse>
#include <vector>

using namespace Eigen;
using namespace std;

    /*
    ** @brief HW4_TutteParameterization
    **
    ** This file presents the basic framework of a "node", which processes inputs
    ** received from the left and outputs specific variables for downstream nodes to
    ** use.
    ** - In the first function, node_declare, you can set up the node's input and
    ** output variables.
    ** - The second function, node_exec is the execution part of the node, where we
    ** need to implement the node's functionality.
    ** Your task is to fill in the required logic at the specified locations
    ** within this template, especially in node_exec.
    */

NODE_DEF_OPEN_SCOPE
NODE_DECLARATION_FUNCTION(min_surf_uniform_weights)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");

    b.add_input<Geometry>("Reference_mesh");

    //b.add_input<std::vector<double>>("Weights");

    /*
    ** NOTE: You can add more inputs or outputs if necessary. For example, in
    *some cases,
    ** additional information (e.g. other mesh geometry, other parameters) is
    *required to perform
    ** the computation.
    **
    ** Be sure that the input/outputs do not share the same name. You can add
    *one geometry as
    **
    **                b.add_input<Geometry>("Input");
    **
    ** Or maybe you need a value buffer like:
    **
    **                b.add_input<float1Buffer>("Weights");
    */

    // Output-1: Minimal surface with fixed boundary
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(min_surf_uniform_weights)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");
    auto weight_input = params.get_input<Geometry>("Reference_mesh");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }

    auto halfedge_mesh = operand_to_openmesh(&input);
    auto weight_halfedge_mesh = operand_to_openmesh(&weight_input);

    // store idx of the vertex, which is not on boundary
    vector<unsigned int> NotBnd;
    // sotre every vertex's idx in NotBnd
    // -1 means on boundary
    vector<int> Mirror(halfedge_mesh->n_vertices(), -1);

    for (const auto& vertex_now : halfedge_mesh->vertices()) 
    {
        unsigned int idx = vertex_now.idx();
        if (!vertex_now.is_boundary())
        {
            NotBnd.push_back(idx);
            Mirror[idx] = NotBnd.size() - 1;
        }
    }
    int dim = NotBnd.size();

    vector<Triplet<double>> triplet_list;
    VectorXd B_x(dim), B_y(dim), B_z(dim);
    B_x.setZero(); B_y.setZero(); B_z.setZero();

    unique_ptr<WeightCalculator> calculator;

    for (int i = 0; i < dim; ++i)
    {
        const auto& vertex_now = halfedge_mesh->
                make_smart(halfedge_mesh->vertex_handle(NotBnd[i]));

        const auto& weight_vertex_now = weight_halfedge_mesh->
                make_smart(weight_halfedge_mesh->vertex_handle(NotBnd[i]));

        double weight = 0.0;

        triplet_list.push_back(Triplet<double>(i, i, 1.0));

        double total_weight = 0.0;
        for (const auto& weight_eighbor : weight_vertex_now.vertices())
        {
            calculator = make_unique<UniformWeights>(*weight_halfedge_mesh, weight_vertex_now, weight_eighbor);
            //calculator = make_unique<CotangentWeights>(*weight_halfedge_mesh, weight_vertex_now, weight_eighbor);
            total_weight += calculator->calculator_weight();
        }
        auto it = vertex_now.vertices().begin();
        auto weight_it = weight_vertex_now.vertices().begin();
        for (;  it != vertex_now.vertices().end() &&
                weight_it != weight_vertex_now.vertices().end(); 
                ++it, ++weight_it)//const auto& neighbor : vertex_now.vertices()
        {
            const auto& neighbor = *it;
            const auto& weight_neighbor = *weight_it;
            // todo: change ways
            calculator = make_unique<UniformWeights>(*weight_halfedge_mesh, weight_vertex_now, weight_neighbor);
            //calculator = make_unique<CotangentWeights>(*weight_halfedge_mesh, weight_vertex_now, weight_neighbor);
            weight = calculator->calculator_weight() / total_weight;
            // weight = calculator->calculator_weight();
            if (neighbor.is_boundary())
            {
                const auto& position = halfedge_mesh->point(neighbor);
                B_x(i) += weight * position[0];
                B_y(i) += weight * position[1];
                B_z(i) += weight * position[2];
            }
            else
            {
                if (Mirror[neighbor.idx()] == -1) 
                {
                    throw std::runtime_error("Neighbor should be non-boundary.");
                }
                triplet_list.push_back(Triplet<double>(i, Mirror[neighbor.idx()], -weight));
            }
        }
    }

    // Build Matrix and solve
    SparseMatrix<double> Matrix_A(dim, dim);
    BiCGSTAB<SparseMatrix<double>> solver;

    Matrix_A.setFromTriplets(triplet_list.begin(), triplet_list.end());
    solver.compute(Matrix_A);

    VectorXd new_x = solver.solve(B_x);
    VectorXd new_y = solver.solve(B_y);
    VectorXd new_z = solver.solve(B_z);

    // Push back
    for (int i = 0; i < dim; ++i)
    {
        auto vertex_handle = halfedge_mesh->vertex_handle(NotBnd[i]);
        auto& point = halfedge_mesh->point(vertex_handle);
        point[0] = new_x[i];
        point[1] = new_y[i]; 
        point[2] = new_z[i]; 
    }

    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_UI(min_surf_uniform_weights);
NODE_DEF_CLOSE_SCOPE
