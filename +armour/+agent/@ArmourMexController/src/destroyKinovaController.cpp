#include "mex.h"
#include "robust_controller.hpp"
#include "robot_models.hpp"

// C-style interface (for some reason?)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Make sure we have the stuff to delete
    if (nrhs != 2) {
        mexErrMsgTxt("Invalid use of destroy!");
    }

    // Temporaries
    Robot* Kinova = nullptr;
    RobustController* c = nullptr;

    // Get the values
    uint64_T* robot_ptr = static_cast<uint64_T*>(mxGetData(prhs[0]));
    uint64_T* controller_ptr = static_cast<uint64_T*>(mxGetData(prhs[1]));

    // Recast them
    Kinova = reinterpret_cast<Robot*>(*robot_ptr);
    c = reinterpret_cast<RobustController*>(*controller_ptr);

    // DELETE
    delete Kinova;
    delete c;
}
