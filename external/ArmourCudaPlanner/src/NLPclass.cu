#ifndef NLP_CLASS_CU
#define NLP_CLASS_CU

#include "NLPclass.h"

// constructor
armtd_NLP::armtd_NLP()
{
}


// destructor
armtd_NLP::~armtd_NLP()
{
    delete[] g_copy;
}


bool armtd_NLP::set_parameters(
    Eigen::VectorXd& q_des_input,
    double t_plan_input,
    const BezierCurve* desired_trajectory_input,
    KinematicsDynamics* kinematics_dynamics_result_input,
    const Eigen::MatrixXd* torque_radius_input,
    Obstacles* obstacles_input
 ) 
 {
    q_des = q_des_input;
    t_plan = t_plan_input;
    desired_trajectory = desired_trajectory_input;
    kinematics_dynamics_result = kinematics_dynamics_result_input;
    torque_radius = torque_radius_input;
    obstacles = obstacles_input;

    constraint_number = NUM_FACTORS * NUM_TIME_STEPS +
                        NUM_JOINTS * NUM_TIME_STEPS * obstacles->num_obstacles + 
                        NUM_FACTORS * 4;

    g_copy = new Number[constraint_number];

    return true;
}


bool armtd_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
    // The problem described NUM_FACTORS variables, x[NUM_FACTORS] through x[NUM_FACTORS] for each joint
    n = NUM_FACTORS;

    // number of inequality constraint
    m = constraint_number;

    nnz_jac_g = m * n;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}
// [TNLP_get_nlp_info]

// [TNLP_get_bounds_info]
// returns the variable bounds
bool armtd_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
    // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
    // If desired, we could assert to make sure they are what we think they are.
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in get_bounds_info!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in get_bounds_info!");
    }

    // lower bounds
    for( Index i = 0; i < n; i++ ) {
        x_l[i] = -1.0;
    }

    // upper bounds  
    for( Index i = 0; i < n; i++ ) {
        x_u[i] = 1.0;
    }

    // control input constraints
    for( Index i = 0; i < NUM_TIME_STEPS; i++ ) {
        for( Index j = 0; j < NUM_FACTORS; j++ ) {
            g_l[i * NUM_FACTORS + j] = -torque_limits[j] + (*torque_radius)(j, i);
            g_u[i * NUM_FACTORS + j] = torque_limits[j] - (*torque_radius)(j, i);
        }
    }    

    // collision avoidance constraints
    Index offset = NUM_FACTORS * NUM_TIME_STEPS;
    for( Index i = offset; i < offset + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles; i++ ) {
        g_l[i] = -1e19;
        g_u[i] = 0;
    }
    offset += NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles;

    // state limit constraints
    //     minimum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = state_limits_lb[i - offset] + qe;
        g_u[i] = state_limits_ub[i - offset] - qe;
    }
    offset += NUM_FACTORS;

    //     maximum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = state_limits_lb[i - offset] + qe;
        g_u[i] = state_limits_ub[i - offset] - qe;
    }
    offset += NUM_FACTORS;

    //     minimum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = -speed_limits[i - offset] + qde;
        g_u[i] = speed_limits[i - offset] - qde;
    }
    offset += NUM_FACTORS;

    //     maximum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        g_l[i] = -speed_limits[i - offset] + qde;
        g_u[i] = speed_limits[i - offset] - qde;
    }

    return true;
}
// [TNLP_get_bounds_info]

// [TNLP_get_starting_point]
// returns the initial point for the problem
bool armtd_NLP::get_starting_point(
    Index   n,
    bool    init_x,
    Number* x,
    bool    init_z,
    Number* z_L,
    Number* z_U,
    Index   m,
    bool    init_lambda,
    Number* lambda
)
{
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    if(init_x == false || init_z == true || init_lambda == true){
        WARNING_PRINT("*** Error wrong value of init in get_starting_point!");
    }

    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in get_starting_point!");
    }

    for( Index i = 0; i < n; i++ ) {
        // initialize to zero
        x[i] = 0.0;

        // try to avoid local minimum
        // x[i] = min(max((q_des[i] - desired_trajectory->q0[i]) / k_range[i], -0.5), 0.5);
    }

    return true;
}
// [TNLP_get_starting_point]

// [TNLP_eval_f]
// returns the value of the objective function
bool armtd_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
    if(n != NUM_FACTORS){
       WARNING_PRINT("*** Error wrong value of n in eval_f!");
    }

    // obj_value = sum((q_plan - q_des).^2);
    obj_value = 0; 
    for(Index i = 0; i < n; i++){
        double q_plan = q_des_func(desired_trajectory->q0[i], desired_trajectory->qd0[i], desired_trajectory->qdd0[i], k_range[i] * x[i], t_plan);
        obj_value += pow(q_plan - q_des[i], 2);
    }

    obj_value *= COST_FUNCTION_OPTIMALITY_SCALE;

    return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool armtd_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_grad_f!");
    }

    for(Index i = 0; i < n; i++){
        double q_plan = q_des_func(desired_trajectory->q0[i], desired_trajectory->qd0[i], desired_trajectory->qdd0[i], k_range[i] * x[i], t_plan);
        double dk_q_plan = pow(t_plan,3) * (6 * pow(t_plan,2) - 15 * t_plan + 10);
        grad_f[i] = (2 * (q_plan - q_des[i]) * dk_q_plan * k_range[i]) * COST_FUNCTION_OPTIMALITY_SCALE;
    }

    return true;
}
// [TNLP_eval_grad_f]

// [TNLP_eval_g]
// return the value of the constraints: g(x)
bool armtd_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_g!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in eval_g!");
    }

    Index i;
    #pragma omp parallel for shared(kinematics_dynamics_result, x, g, link_sliced_center) private(i) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
    for(i = 0; i < NUM_TIME_STEPS; i++) {
        for (int k = 0; k < NUM_FACTORS; k++) {
            MatrixXInt res = kinematics_dynamics_result->u_nom(k, i).slice(x);
            g[i * NUM_FACTORS + k] = getCenter(res(0));
        }

        for (int l = 0; l < NUM_JOINTS; l++) {
            MatrixXInt res = kinematics_dynamics_result->links(l, i).slice(x);
            link_sliced_center[i * NUM_JOINTS + l] = getCenter(res);
        }
    }

    // Part 3. check collision between joint position reachable set and obstacles (in gpu)
    obstacles->linkFRSConstraints(link_sliced_center, nullptr, g + NUM_TIME_STEPS * NUM_FACTORS, nullptr);

    // Part 4. (position & velocity) state limit constraints
    desired_trajectory->returnJointPositionExtremum(g + NUM_TIME_STEPS * NUM_FACTORS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles, x);
    desired_trajectory->returnJointVelocityExtremum(g + NUM_TIME_STEPS * NUM_FACTORS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles + NUM_FACTORS * 2, x);

    return true;
}
// [TNLP_eval_g]


// [TNLP_eval_jac_g]
// return the structure or values of the Jacobian
bool armtd_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
    if(n != NUM_FACTORS){
        WARNING_PRINT("*** Error wrong value of n in eval_g!");
    }
    if(m != constraint_number){
        WARNING_PRINT("*** Error wrong value of m in eval_g!");
    }
        
    if( values == NULL ) {
       // return the structure of the Jacobian
       // this particular Jacobian is dense
        for(Index i = 0; i < m; i++){
            for(Index j = 0; j < n; j++){
                iRow[i * n + j] = i;
                jCol[i * n + j] = j;
            }
        }
    }
    else {
        Index i;
        #pragma omp parallel for shared(kinematics_dynamics_result, x, values, link_sliced_center, dk_link_sliced_center) private(i) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(i = 0; i < NUM_TIME_STEPS; i++) {
            for (int k = 0; k < NUM_FACTORS; k++) {
                kinematics_dynamics_result->u_nom(k, i).slice(values + (i * NUM_FACTORS + k) * NUM_FACTORS, x);
            }

            for (int l = 0; l < NUM_JOINTS; l++) {
                link_sliced_center[i * NUM_JOINTS + l] = getCenter(kinematics_dynamics_result->links(l, i).slice(x));
                kinematics_dynamics_result->links(l, i).slice(dk_link_sliced_center + (i * NUM_JOINTS + l) * NUM_FACTORS, x);
            }
        }

        // Part 3. check collision between joint position reachable set and obstacles (in gpu)
        obstacles->linkFRSConstraints(link_sliced_center, dk_link_sliced_center, nullptr, values + NUM_TIME_STEPS * NUM_FACTORS * NUM_FACTORS);

        // Part 4. (position & velocity) state limit constraints
        desired_trajectory->returnJointPositionExtremumGradient(values + (NUM_TIME_STEPS * NUM_FACTORS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles) * NUM_FACTORS, x);
        desired_trajectory->returnJointVelocityExtremumGradient(values + (NUM_TIME_STEPS * NUM_FACTORS + NUM_TIME_STEPS * NUM_JOINTS * obstacles->num_obstacles + NUM_FACTORS * 2) * NUM_FACTORS, x);
    }

    return true;
}
// [TNLP_eval_jac_g]


// [TNLP_eval_h]
//return the structure or values of the Hessian
bool armtd_NLP::eval_h(
   Index         n,
   const Number* x,
   bool          new_x,
   Number        obj_factor,
   Index         m,
   const Number* lambda,
   bool          new_lambda,
   Index         nele_hess,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
    return false;
}
// [TNLP_eval_h]


// [TNLP_finalize_solution]
void armtd_NLP::finalize_solution(
    SolverReturn               status,
    Index                      n,
    const Number*              x,
    const Number*              z_L,
    const Number*              z_U,
    Index                      m,
    const Number*              g,
    const Number*              lambda,
    Number                     obj_value,
    const IpoptData*           ip_data,
    IpoptCalculatedQuantities* ip_cq
)
{
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // store the solution
    for( Index i = 0; i < n; i++ ) {
        solution[i] = (double)x[i];
    }

    // check constraint violation manually for Maximum_CpuTime_Exceeded case
    memcpy(g_copy, g, m * sizeof(Number));

    feasible = true;

    // control input constraints
    for( Index i = 0; i < NUM_TIME_STEPS; i++ ) {
        for( Index j = 0; j < NUM_FACTORS; j++ ) {
            if (g_copy[i * NUM_FACTORS + j] < -torque_limits[j] + (*torque_radius)(j, i) - TORQUE_INPUT_CONSTRAINT_VIOLATION_THRESHOLD || 
                g_copy[i * NUM_FACTORS + j] > torque_limits[j] - (*torque_radius)(j, i) + TORQUE_INPUT_CONSTRAINT_VIOLATION_THRESHOLD) {
                feasible = false;
                cout << "        CUDA & C++: Ipopt: Control torque of joint " << j << " at time interval " << i << " exceeds limit!\n";
                cout << "                        value: " << g_copy[i * NUM_FACTORS + j] << "\n";
                cout << "                        range: [ " << -torque_limits[j] + (*torque_radius)(j, i) << ", "
                                                            << torque_limits[j] - (*torque_radius)(j, i) << " ]\n";
                return;
            }
        }
    }    

    // collision avoidance constraints
    Index offset = NUM_FACTORS * NUM_TIME_STEPS;
    for( Index i = 0; i < NUM_JOINTS; i++ ) {
        for( Index j = 0; j < NUM_TIME_STEPS; j++ ) {
            for( Index h = 0; h < obstacles->num_obstacles; h++ ) {
                if (g_copy[(i * NUM_TIME_STEPS + j) * obstacles->num_obstacles + h + offset] > COLLISION_AVOIDANCE_CONSTRAINT_VIOLATION_THRESHOLD) {
                    feasible = false;
                    cout << "        CUDA & C++: Ipopt: Collision between link " << i + 1 << " and obstacle " << h << " at time interval " << j << "!\n";
                    cout << "                        value: " << g_copy[(i * NUM_TIME_STEPS + j) * obstacles->num_obstacles + h + offset] << "\n";
                    return;
                }
            }
        }
    }
    offset += NUM_JOINTS * NUM_TIME_STEPS * obstacles->num_obstacles;

    // state limit constraints
    //     minimum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < state_limits_lb[i - offset] + qe || g_copy[i] > state_limits_ub[i - offset] - qe) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds position limit when it reaches minimum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << state_limits_lb[i - offset] + qe << ", "
                                                        << state_limits_ub[i - offset] - qe << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     maximum joint position
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < state_limits_lb[i - offset] + qe || g_copy[i] > state_limits_ub[i - offset] - qe) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds position limit when it reaches maximum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << state_limits_lb[i - offset] + qe << ", "
                                                        << state_limits_ub[i - offset] - qe << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     minimum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < -speed_limits[i - offset] + qde || g_copy[i] > speed_limits[i - offset] - qde) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds velocity limit when it reaches minimum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << -speed_limits[i - offset] + qde << ", "
                                                        << speed_limits[i - offset] - qde << " ]\n";
            return;
        }
    }
    offset += NUM_FACTORS;

    //     maximum joint velocity
    for( Index i = offset; i < offset + NUM_FACTORS; i++ ) {
        if (g_copy[i] < -speed_limits[i - offset] + qde || g_copy[i] > speed_limits[i - offset] - qde) {
            feasible = false;
            cout << "        CUDA & C++: Ipopt: joint " << i - offset << " exceeds velocity limit when it reaches maximum!\n";
            cout << "                        value: " << g_copy[i] << "\n";
            cout << "                        range: [ " << -speed_limits[i - offset] + qde << ", "
                                                        << speed_limits[i - offset] - qde << " ]\n";
            return;
        }
    }
}
// [TNLP_finalize_solution]


#endif
