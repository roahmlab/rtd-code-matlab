#include "mex.h"
#include "robust_controller.hpp"
#include "robot_models.hpp"

void copyPointerToEigenVector(Eigen::VectorXd& a, double* b, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
        a(i) = b[i];
    }
}

void copyEigenVectorToPointer(double* a, Eigen::VectorXd& b, unsigned int n) {
    for (unsigned int i = 0; i < n; i++) {
        a[i] = b(i);
    }
}

// C-style interface (for some reason?)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Temporaries
    Robot* Kinova = nullptr;
    RobustController* c = nullptr;

    // Get the values
    uint64_T* robot_ptr = static_cast<uint64_T*>(mxGetData(prhs[0]));
    uint64_T* controller_ptr = static_cast<uint64_T*>(mxGetData(prhs[1]));

    // Recast them
    Kinova = reinterpret_cast<Robot*>(*robot_ptr);
    c = reinterpret_cast<RobustController*>(*controller_ptr);

    int numJoints = Kinova->numJoints;

    // Begin to solve
    // Consider using Eigen Map?
    double* q_input = (double*)mxGetData(prhs[2]);
    Eigen::VectorXd q(numJoints);
    copyPointerToEigenVector(q, q_input, numJoints);

    double* q_d_input = (double*)mxGetData(prhs[3]);
    Eigen::VectorXd q_d(numJoints);
    copyPointerToEigenVector(q_d, q_d_input, numJoints);

    double* qd_input = (double*)mxGetData(prhs[4]);
    Eigen::VectorXd qd(numJoints);
    copyPointerToEigenVector(qd, qd_input, numJoints);

    double* qd_d_input = (double*)mxGetData(prhs[5]);
    Eigen::VectorXd qd_d(numJoints);
    copyPointerToEigenVector(qd_d, qd_d_input, numJoints);

    double* qd_dd_input = (double*)mxGetData(prhs[6]);
    Eigen::VectorXd qd_dd(numJoints);
    copyPointerToEigenVector(qd_dd, qd_dd_input, numJoints);

    Eigen::VectorXd u = c->update(Kinova, q, q_d, qd, qd_d, qd_dd);

    plhs[0] = mxCreateNumericMatrix(numJoints, 1, mxDOUBLE_CLASS, mxREAL);
    copyEigenVectorToPointer((double*)mxGetData(plhs[0]), u, numJoints);

    plhs[1] = mxCreateNumericMatrix(numJoints, 1, mxDOUBLE_CLASS, mxREAL);
    copyEigenVectorToPointer((double*)mxGetData(plhs[1]), c->u_nominal, numJoints);

    plhs[2] = mxCreateNumericMatrix(numJoints, 1, mxDOUBLE_CLASS, mxREAL);
    copyEigenVectorToPointer((double*)mxGetData(plhs[2]), c->v, numJoints);

    nlhs = 3;
}
