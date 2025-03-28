#include "GCore/Components/MeshOperand.h"
#include "geom_node_base.h"
#include "GCore/util_openmesh_bind.h"
#include "UniformWeights.h"

#include <Eigen/Sparse>
#include <vector>

using namespace Eigen;
using namespace std;

const double pi = 3.14159265358979323846;

    /*
    ** @brief HW4_TutteParameterization
    **
    ** This file contains two nodes whose primary function is to map the boundary of
    *a mesh to a plain
    ** convex closed curve (circle of square), setting the stage for subsequent
    *Laplacian equation
    ** solution and mesh parameterization tasks.
    **
    ** Key to this node's implementation is the adept manipulation of half-edge data
    *structures
    ** to identify and modify the boundary of the mesh.
    **
    ** Task Overview:
    ** - The two execution functions (node_map_boundary_to_square_exec,
    ** node_map_boundary_to_circle_exec) require an update to accurately map the
    *mesh boundary to a and
    ** circles. This entails identifying the boundary edges, evenly distributing
    *boundary vertices along
    ** the square's perimeter, and ensuring the internal vertices' positions remain
    *unchanged.
    ** - A focus on half-edge data structures to efficiently traverse and modify
    *mesh boundaries.
    */

NODE_DEF_OPEN_SCOPE

    /*
    ** HW4_TODO: Node to map the mesh boundary to a circle.
    */

NODE_DECLARATION_FUNCTION(circle_boundary_mapping)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");
    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the
    // interior vertices remains the same
    b.add_output<Geometry>("Output");
}

OpenMesh::SmartVertexHandle 
    NextBoundaryVertex(
            const std::shared_ptr<USTC_CG::PolyMesh> &halfedge_mesh,
            const OpenMesh::SmartVertexHandle &LastVertex)
{
    OpenMesh::SmartVertexHandle NextVertex;
    for (const auto& edge_out : LastVertex.outgoing_halfedges())
    {
        if (edge_out.is_boundary())
        {
            NextVertex = edge_out.to();
            break;
        }
    }
    return NextVertex;
}

double GetLength(
    const std::shared_ptr<USTC_CG::PolyMesh> &halfedge_mesh,
    const OpenMesh::SmartVertexHandle &Vertex1,
    const OpenMesh::SmartVertexHandle &Vertex2 )
{
    const auto& position1 = halfedge_mesh->point(Vertex1);
    const auto& position2 = halfedge_mesh->point(Vertex2);

    double delta_x = position1[0] - position2[0];
    double delta_y = position1[1] - position2[1];
    double delta_z = position1[2] - position2[2];

    double ret = pow(
                        delta_x * delta_x + 
                        delta_y * delta_y + 
                        delta_z * delta_z , 0.5);
    return ret;
}

// double GetTheta()

NODE_EXECUTION_FUNCTION(circle_boundary_mapping)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }
    // throw std::runtime_error("Not implemented");
    auto halfedge_mesh = operand_to_openmesh(&input);

    vector<unsigned int> BndList;

    for (const auto& vertex_now : halfedge_mesh->vertices()) 
    {
        if (vertex_now.is_boundary())
        {
            BndList.push_back(vertex_now.idx());
            break;
        }
    }

    const auto& StartVertex = halfedge_mesh->
            make_smart(halfedge_mesh->vertex_handle(BndList[0]));
    auto VertexNow = NextBoundaryVertex(halfedge_mesh, StartVertex);
    while (StartVertex.idx() != VertexNow.idx())
    {
        if (!VertexNow.is_boundary())
        {
            throw std::runtime_error("VertexNow should be on boundary");
            return false;
        }
        BndList.push_back(VertexNow.idx());
        VertexNow = NextBoundaryVertex(halfedge_mesh, VertexNow);
    }
    
    double perimeter = 0.0;
    for (int i = 0; i < BndList.size(); ++i)
    {
        perimeter += GetLength(  halfedge_mesh, 
                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[i])),
                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[(i+1) % BndList.size()])));
    }
    
    vector<double> x_list, y_list;
    double total_length = 0.0;
    
    for (int i = 0; i < BndList.size(); ++i)
    {
        double Theta = pi * 2.0 * (total_length / perimeter);
        x_list.push_back(cos(Theta));
        y_list.push_back(sin(Theta));
        total_length += GetLength(    halfedge_mesh, 
                                    halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[i])),
                                    halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[(i+1) % BndList.size()])));
    }

    // Push back
    for (int i = 0; i < BndList.size(); ++i)
    {
        auto vertex_handle = halfedge_mesh->vertex_handle(BndList[i]);
        auto& point = halfedge_mesh->point(vertex_handle);
        point[0] = x_list[i];
        point[1] = y_list[i]; 
        point[2] = 0.0; 
    }

    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
	return true;
}

    /*
    ** HW4_TODO: Node to map the mesh boundary to a square.
    */

NODE_DECLARATION_FUNCTION(square_boundary_mapping)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the
    // interior vertices remains the same
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(square_boundary_mapping)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Input does not contain a mesh");
    }
    //throw std::runtime_error("Not implemented");

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    if (halfedge_mesh->n_vertices() <= 3)
    {
        throw std::runtime_error("Num of Vertex is not enough");
    }

    vector<unsigned int> BndList;

    for (const auto& vertex_now : halfedge_mesh->vertices()) 
    {
        if (vertex_now.is_boundary())
        {
            BndList.push_back(vertex_now.idx());
            break;
        }
    }

    const auto& StartVertex = halfedge_mesh->
            make_smart(halfedge_mesh->vertex_handle(BndList[0]));
    auto VertexNow = NextBoundaryVertex(halfedge_mesh, StartVertex);
    while (StartVertex.idx() != VertexNow.idx())
    {
        if (!VertexNow.is_boundary())
        {
            throw std::runtime_error("VertexNow should be on boundary");
            return false;
        }
        BndList.push_back(VertexNow.idx());
        VertexNow = NextBoundaryVertex(halfedge_mesh, VertexNow);
    }
    
    double perimeter = 0.0;
    for (int i = 0; i < BndList.size(); ++i)
    {
        perimeter += GetLength(  halfedge_mesh, 
                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[i])),
                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[(i+1) % BndList.size()])));
    }
    
    vector<double> x_list, y_list;
    double side_length = perimeter / 4.0;

    for (int i = 0; i < 4; ++i)
    {
        double total_length = 0.0;
        for (int j = 0; j < BndList.size() / 4; ++j)
        {
            int idx = i * (BndList.size() / 4 ) + j;

            double temp = min(total_length / side_length, 1.0);
            switch (i)
            {
            case 0:
                x_list.push_back(0.0);
                y_list.push_back(temp);
                break;
            case 1:
                x_list.push_back(temp);
                y_list.push_back(1.0);
                break;
            case 2:
                x_list.push_back(1.0);
                y_list.push_back(1.0 - temp);
                break;
            case 3:
                x_list.push_back(1.0 - temp);
                y_list.push_back(0);
                break;
            default:
                break;
            }
            total_length += GetLength(  halfedge_mesh, 
                                        halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[idx])),
                                        halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[(idx+1) % BndList.size()])));

            if (i == 3 && j == (BndList.size() / 4) - 1)
            {
                while((i * (BndList.size() / 4) + j) < BndList.size())
                {
                    int idx = i * (BndList.size() / 4) + j;
                    double temp = min(total_length / side_length, 1.0);
                    
                    x_list.push_back(1 - temp);
                    y_list.push_back(0);

                    total_length += GetLength(  halfedge_mesh, 
                                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[idx])),
                                                halfedge_mesh->make_smart(halfedge_mesh->vertex_handle(BndList[(idx+1) % BndList.size()])));
                    ++j;
                }
            }
        }
    }

    // Push back
    for (int i = 0; i < BndList.size(); ++i)
    {
        auto vertex_handle = halfedge_mesh->vertex_handle(BndList[i]);
        auto& point = halfedge_mesh->point(vertex_handle);
        point[0] = x_list[i];
        point[1] = y_list[i]; 
        point[2] = 0.0; 
    }
   
    auto geometry = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}



NODE_DECLARATION_UI(boundary_mapping);
NODE_DEF_CLOSE_SCOPE
