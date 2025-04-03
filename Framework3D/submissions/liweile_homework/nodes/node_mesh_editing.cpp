#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <vector>
#include "CotangentWeights.h"
#include "UniformWeights.h"

#include <chrono>

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

using namespace Eigen;
using namespace std;

enum class WeightType
{
	Uniform,
	Cotangent
};

struct Matrix_Solver_storage
{
	// We want to solve (ATA+BTB)x = ATb_1 + BTb_2
	// where A is the matrix of the error
	// and B is the matrix of the constraints.
	// A, B and b_1 can be directly computed from the mesh.
	// only b_2 is the input of the control nodes.

	constexpr static bool has_storage = false;

	shared_ptr<SimplicialLDLT<SparseMatrix<double>>> solver_storage;

	SparseMatrix<double> B, Matrix_Final;
	VectorXd ATb_1x, ATb_1y, ATb_1z;

	int rows = 0;
	int cols = 0;
};

void build_Matrix_Solver_storage( const USTC_CG::PolyMesh &MyMesh,
								  Matrix_Solver_storage &storage, 
								  const vector<size_t> &control_points_indices,
								  enum class WeightType weight_type)
{
	int dim = MyMesh.n_vertices();
	unordered_set<size_t> control_set(control_points_indices.begin(), control_points_indices.end());

	vector<Triplet<double>> triplet_list_A, triplet_list_B;
    VectorXd b_1x(dim), b_1y(dim), b_1z(dim);
    b_1x.setZero(); b_1y.setZero(); b_1z.setZero();

    std::unique_ptr<USTC_CG::WeightCalculator> calculator;

	for (int i = 0; i < dim; ++i)
	{
		const auto& vertex_now = MyMesh.make_smart(MyMesh.vertex_handle(i));

		triplet_list_A.push_back(Triplet<double>(i, i, 1.0));

		if (control_set.count(i)) {
			triplet_list_B.push_back(Triplet<double>(i, i, 1.0));
		}

		double total_weight = 0.0;
        for (const auto& neighbor : vertex_now.vertices())
        {
			if (WeightType::Uniform == weight_type) {
				calculator = make_unique<USTC_CG::UniformWeights>(MyMesh, vertex_now, neighbor);
			}
			else if(WeightType::Cotangent == weight_type) {
				calculator = make_unique<USTC_CG::CotangentWeights>(MyMesh, vertex_now, neighbor);
			}
            total_weight += calculator->calculator_weight();
        }
		

		// TODO: Check the numbers' order of b_1
		b_1x(i) = MyMesh.point(vertex_now)[0];
		b_1y(i) = MyMesh.point(vertex_now)[1];
		b_1z(i) = MyMesh.point(vertex_now)[2];
		b_1x(i) = MyMesh.point(MyMesh.make_smart(MyMesh.vertex_handle(i)))[0];
		b_1y(i) = MyMesh.point(MyMesh.make_smart(MyMesh.vertex_handle(i)))[1];
		b_1z(i) = MyMesh.point(MyMesh.make_smart(MyMesh.vertex_handle(i)))[2];
        for (	auto it = vertex_now.vertices().begin();  
				it != vertex_now.vertices().end(); ++it)
		{
			const auto& neighbor = *it;
			if (WeightType::Uniform == weight_type) {
				calculator = make_unique<USTC_CG::UniformWeights>(MyMesh, vertex_now, neighbor);
			}
			else if(WeightType::Cotangent == weight_type) {
				calculator = make_unique<USTC_CG::CotangentWeights>(MyMesh, vertex_now, neighbor);
			}
            double weight = calculator->calculator_weight() / total_weight;
			triplet_list_A.push_back(Triplet<double>(i, neighbor.idx(), -weight));

			b_1x(i) -= weight * MyMesh.point(neighbor)[0];
			b_1y(i) -= weight * MyMesh.point(neighbor)[1];
			b_1z(i) -= weight * MyMesh.point(neighbor)[2];
		}
	}

	SparseMatrix<double> Matrix_A(dim, dim), Matrix_B(dim, dim), Matrix_AT(dim, dim);

    Matrix_A.setFromTriplets(triplet_list_A.begin(), triplet_list_A.end());
	Matrix_B.setFromTriplets(triplet_list_B.begin(), triplet_list_B.end());
	Matrix_AT = Matrix_A.transpose();
	// TODO: Check Matrix_B's dim 
	
	SparseMatrix<double> Matrix_Final = Matrix_AT * Matrix_A + Matrix_B * Matrix_B;
	storage.Matrix_Final = Matrix_Final;
	storage.solver_storage = make_shared<SimplicialLDLT<SparseMatrix<double>>>();
	storage.solver_storage->compute(storage.Matrix_Final);
	storage.ATb_1x = Matrix_AT * b_1x;
	storage.ATb_1y = Matrix_AT * b_1y;
	storage.ATb_1z = Matrix_AT * b_1z;
	storage.B = Matrix_B;
	storage.rows = dim;
	storage.cols = dim;
	// storage.has_storage = true;
}

NODE_DEF_OPEN_SCOPE
NODE_DECLARATION_FUNCTION(mesh_editing_CotangentWeights)
{
    // Input-1: Original 3D mesh with boundary
	// Input-2: Position of all points after change
	// Input-3: Indices of control points

    b.add_input<Geometry>("Original mesh");
    b.add_input<pxr::VtVec3fArray>("Changed vertices");
    b.add_input<std::vector<size_t>>("Control Points Indices");

	// Output: New positions of all points or changed mesh
    b.add_output<pxr::VtVec3fArray>("New vertices");
}

NODE_EXECUTION_FUNCTION(mesh_editing_CotangentWeights)
{
	// Get the input from params
	auto input = params.get_input<Geometry>("Original mesh");
	pxr::VtVec3fArray changed_vertices = params.get_input<pxr::VtVec3fArray>("Changed vertices");
	std::vector<size_t> control_points_indices = params.get_input<std::vector<size_t>>("Control Points Indices");

	auto MyMesh = operand_to_openmesh(&input);

	// Avoid processing the node when there is no input
	if (!input.get_component<MeshComponent>()) {
		throw std::runtime_error("Mesh Editing: Need Geometry Input.");
		return false;
	}
	if (control_points_indices.empty()) {
		throw std::runtime_error("at least one point");
	}


	auto &storage = params.get_storage<Matrix_Solver_storage&>();
	if (storage.rows != MyMesh->n_vertices() || storage.cols != MyMesh->n_vertices()) 
	{
		// The size of the matrix is changed, we need to rebuild the solver storage.
		build_Matrix_Solver_storage(*MyMesh, storage, control_points_indices, WeightType::Cotangent);
	}

	VectorXd b_2x(MyMesh->n_vertices()), b_2y(MyMesh->n_vertices()), b_2z(MyMesh->n_vertices());
	b_2x.setZero(); b_2y.setZero(); b_2z.setZero();

	// using control_points_indices to set b_2
	for (size_t i = 0; i < control_points_indices.size(); ++i) 
	{
		b_2x(control_points_indices[i]) = changed_vertices[control_points_indices[i]][0];
		b_2y(control_points_indices[i]) = changed_vertices[control_points_indices[i]][1];
		b_2z(control_points_indices[i]) = changed_vertices[control_points_indices[i]][2];
	}
	VectorXd b_final_x = storage.ATb_1x + storage.B * b_2x;
	VectorXd b_final_y = storage.ATb_1y + storage.B * b_2y;
	VectorXd b_final_z = storage.ATb_1z + storage.B * b_2z;

	VectorXd new_x = storage.solver_storage->solve(b_final_x);
	VectorXd new_y = storage.solver_storage->solve(b_final_y);
	VectorXd new_z = storage.solver_storage->solve(b_final_z);

	// Output the new positions of all points or you can output the changed mesh
	pxr::VtVec3fArray new_positions(MyMesh->n_vertices());
	for (size_t i = 0; i < MyMesh->n_vertices(); ++i) 
	{
		new_positions[i] = pxr::GfVec3f(new_x(i), new_y(i), new_z(i));
	}


	params.set_output("New vertices", std::move(new_positions));

    return true;
}


NODE_DECLARATION_FUNCTION(mesh_editing_UniformWeights)
{
    // Input-1: Original 3D mesh with boundary
	// Input-2: Position of all points after change
	// Input-3: Indices of control points

    b.add_input<Geometry>("Original mesh");
    b.add_input<pxr::VtVec3fArray>("Changed vertices");
    b.add_input<std::vector<size_t>>("Control Points Indices");

	// Output: New positions of all points or changed mesh
    b.add_output<pxr::VtVec3fArray>("New vertices");
}

NODE_EXECUTION_FUNCTION(mesh_editing_UniformWeights)
{
	// Get the input from params
	auto input = params.get_input<Geometry>("Original mesh");
	pxr::VtVec3fArray changed_vertices = params.get_input<pxr::VtVec3fArray>("Changed vertices");
	std::vector<size_t> control_points_indices = params.get_input<std::vector<size_t>>("Control Points Indices");

	auto MyMesh = operand_to_openmesh(&input);

	// Avoid processing the node when there is no input
	if (!input.get_component<MeshComponent>()) {
		throw std::runtime_error("Mesh Editing: Need Geometry Input.");
		return false;
	}
	if (control_points_indices.empty()) {
		throw std::runtime_error("at least one point");
	}


	auto &storage = params.get_storage<Matrix_Solver_storage&>();
	if (storage.rows != MyMesh->n_vertices() || storage.cols != MyMesh->n_vertices()) 
	{
		// The size of the matrix is changed, we need to rebuild the solver storage.
		build_Matrix_Solver_storage(*MyMesh, storage, control_points_indices, WeightType::Uniform);
	}

	auto start1 = std::chrono::high_resolution_clock::now();
	VectorXd b_2x(MyMesh->n_vertices()), b_2y(MyMesh->n_vertices()), b_2z(MyMesh->n_vertices());
	b_2x.setZero(); b_2y.setZero(); b_2z.setZero();

	// using control_points_indices to set b_2
	for (size_t i = 0; i < control_points_indices.size(); ++i) 
	{
		b_2x(control_points_indices[i]) = changed_vertices[control_points_indices[i]][0];
		b_2y(control_points_indices[i]) = changed_vertices[control_points_indices[i]][1];
		b_2z(control_points_indices[i]) = changed_vertices[control_points_indices[i]][2];
	}
	VectorXd b_final_x = storage.ATb_1x + storage.B * b_2x;
	VectorXd b_final_y = storage.ATb_1y + storage.B * b_2y;
	VectorXd b_final_z = storage.ATb_1z + storage.B * b_2z;

	VectorXd new_x = storage.solver_storage->solve(b_final_x);
	VectorXd new_y = storage.solver_storage->solve(b_final_y);
	VectorXd new_z = storage.solver_storage->solve(b_final_z);

	// Output the new positions of all points or you can output the changed mesh
	pxr::VtVec3fArray new_positions(MyMesh->n_vertices());
	for (size_t i = 0; i < MyMesh->n_vertices(); ++i) 
	{
		new_positions[i] = pxr::GfVec3f(new_x(i), new_y(i), new_z(i));
	}

	params.set_output("New vertices", std::move(new_positions));

    return true;
}

NODE_DECLARATION_UI(mesh_editing);
NODE_DEF_CLOSE_SCOPE

