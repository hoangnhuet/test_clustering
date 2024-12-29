// #include <iostream>
// #include <Eigen/Core>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>

// // Define the vertex: a single variable x
// class VertexX : public g2o::BaseVertex<1, double> {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     // Reset to zero
//     void setToOriginImpl() override {
//         _estimate = 0.0;
//     }

//     // Update rule
//     void oplusImpl(const double *update) override {
//         _estimate += update[0];
//     }

//     // Read and write functions (not used here)
//     bool read(std::istream &) override {
//         return false;
//     }
//     bool write(std::ostream &) const override {
//         return false;
//     }
// };

// // Define the edge: the error function (x - 5)^2
// class EdgeCostFunction : public g2o::BaseUnaryEdge<1, double, VertexX> {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     EdgeCostFunction(double target) {
//         _measurement = target;  // Target value
//     }

//     // Compute the error: error = x - target
//     void computeError() override {
//         const VertexX *v = static_cast<const VertexX *>(_vertices[0]);
//         const double x = v->estimate();
//         _error[0] = x - _measurement;
//     }

//     // Read and write functions (not used here)
//     bool read(std::istream &) override {
//         return false;
//     }
//     bool write(std::ostream &) const override {
//         return false;
//     }
// };

// int main() {
//     // Create optimizer
//     g2o::SparseOptimizer optimizer;

//     // Use a dense linear solver
//     auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
//     auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
//     auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
//     optimizer.setAlgorithm(solver);

//     // Add vertex
//     VertexX *v = new VertexX();
//     v->setId(0);
//     v->setEstimate(0.0);  // Initial guess
//     optimizer.addVertex(v);

//     // Add edge
//     EdgeCostFunction *edge = new EdgeCostFunction(5.0);  // Target value is 5.0
//     edge->setId(0);
//     edge->setVertex(0, v);
//     edge->setMeasurement(5.0);
//     edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());  // Weight of the error
//     optimizer.addEdge(edge);

//     // Perform optimization
//     std::cout << "Initial estimate: " << v->estimate() << std::endl;
//     optimizer.initializeOptimization();
//     optimizer.optimize(10);  // Number of iterations
//     std::cout << "Optimized estimate: " << v->estimate() << std::endl;

//     return 0;
// }
#include <iostream>
#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

// Define the vertex: a single variable x
class VertexX : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Reset to zero
    void setToOriginImpl() override {
        _estimate = 0.0;
    }

    // Update rule
    void oplusImpl(const double* update) override {
        _estimate += update[0];
    }

    // Read and write functions (not used here)
    bool read(std::istream&) override {
        return false;
    }

    bool write(std::ostream&) const override {
        return false;
    }
};

// Define the edge: the error function (x - target)
class EdgeCostFunction : public g2o::BaseUnaryEdge<1, double, VertexX> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeCostFunction() {
        // Constructor doesn't need to take target value anymore
        // as we'll use setMeasurement() later
    }

    // Compute the error: error = x - target
    void computeError() override {
        const VertexX* v = static_cast<const VertexX*>(_vertices[0]);
        const double x = v->estimate();
        _error[0] = x - _measurement;  // Fixed: Use _error instead of error
    }

    // Read and write functions (not used here)
    bool read(std::istream&) override {
        return false;
    }

    bool write(std::ostream&) const override {
        return false;
    }
};

int main() {
    // Create optimizer
    g2o::SparseOptimizer optimizer;

    // Use a dense linear solver
    std::unique_ptr<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>> linearSolver =
        std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

    std::unique_ptr<g2o::BlockSolverX> blockSolver =
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    optimizer.setAlgorithm(solver);

    // Add vertex
    auto* v = new VertexX();
    v->setId(0);
    v->setEstimate(0.0);  // Initial guess
    optimizer.addVertex(v);

    // Add edge
    auto* edge = new EdgeCostFunction();
    edge->setId(0);
    edge->setVertex(0, v);
    edge->setMeasurement(5.0);  // Target value is 5.0
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());  // Weight of the error
    optimizer.addEdge(edge);

    // Perform optimization
    std::cout << "Initial estimate: " << v->estimate() << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);  // Number of iterations
    std::cout << "Optimized estimate: " << v->estimate() << std::endl;

    return 0;
}
