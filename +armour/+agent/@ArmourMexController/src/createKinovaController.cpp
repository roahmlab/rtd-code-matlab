#include "mex.h"
#include "robust_controller.hpp"
#include "robot_models.hpp"

// C-style interface (for some reason?)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // create a left pinned model
    // Use C pointers so we can wrap this in matlab
    Robot* Kinova = nullptr;
    // Get the model file
    // input path must be a string
    if (mxIsChar(prhs[0]) != 1) {
        mexErrMsgTxt("Model path must be a string.");
    }

    // input path must be a row vector
    if (mxGetM(prhs[0]) != 1) {
        mexErrMsgTxt("Model path must be a row vector.");
    }

    // Make sure the output is reasonable
    if (nlhs != 2) {
        mexErrMsgTxt("Must return to two types, one pointer for robot, one pointer for controller!");
    }

    // Attempt to load the file.
    char* robot_filepath = mxArrayToString(prhs[0]);
    double* ModelUncertainty = (double*)mxGetData(prhs[1]);
    try {
        Kinova = new Robot(robot_filepath, *ModelUncertainty);
    }
    catch (int error_code) {
        mexErrMsgTxt("Wrong model path! Check your first argument");
    }

    int numJoints = Kinova->numJoints;

    double* Kr_input = (double*)mxGetData(prhs[2]);
    Eigen::MatrixXd Kr = Eigen::MatrixXd::Identity(numJoints, numJoints);
    for (int i = 0; i < numJoints; i++) {
        Kr(i,i) = Kr_input[i];
    }

    double* alpha_input = (double*)mxGetData(prhs[3]);

    double* V_max_input = (double*)mxGetData(prhs[4]);

    double* r_norm_threshold_input = (double*)mxGetData(prhs[5]);
    
    // create a robust controller
    RobustController* c = new RobustController(Kr, *alpha_input, *V_max_input, *r_norm_threshold_input);
    c->applyFriction = false;
    
    // store both pointers to the left
    plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    plhs[1] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    uint64_T* robot_ptr = static_cast<uint64_T*>(mxGetData(plhs[0]));
    uint64_T* controller_ptr = static_cast<uint64_T*>(mxGetData(plhs[1]));
    *robot_ptr = reinterpret_cast<uint64_T>(Kinova);
    *controller_ptr = reinterpret_cast<uint64_T>(c);

    // free string
    mxFree(robot_filepath);
}
