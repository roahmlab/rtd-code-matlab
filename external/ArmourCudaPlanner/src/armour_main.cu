#include "NLPclass.h"
#include "BufferPath.h"

const std::string inputext = ".in";
const std::string outputext1 = ".out";
const std::string outputext2 = ".joint_position_center";
const std::string outputext3 = ".joint_position_radius";
const std::string outputext4 = ".control_input_radius";
const std::string outputext5 = ".constraints";

int main(int argc, char **argv) {
/*
Section I:
    Parse input
    There is no check and warning, so be careful!
*/
    // Here is an example of required input
    // double q0[NUM_FACTORS] = {0.6543, -0.0876, -0.4837, -1.2278, -1.5735, -1.0720, 0};
    // double qd0[NUM_FACTORS] = {0, 0, 0, 0, 0, 0, 0};
    // double qdd0[NUM_FACTORS] = {0, 0, 0, 0, 0, 0, 0};
    // double q_des[NUM_FACTORS] = {0.6831, 0.009488, -0.2471, -0.9777, -1.414, -0.9958, 0};

    // const int num_obstacles = 10;
    // const double obstacles[num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {-0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             -0.28239,  -0.33281, 0.88069, 0.069825, 0, 0, 0,  0.09508, 0, 0, 0, 0.016624,
    //                                                                             -0.19033,  0.035391,  1.3032,  0.11024, 0, 0, 0, 0.025188, 0, 0, 0, 0.014342,
    //                                                                             0.67593, -0.085841, 0.43572,  0.17408, 0, 0, 0,  0.07951, 0, 0, 0,  0.18012,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981,
    //                                                                             0.75382,   0.51895,  0.4731, 0.030969, 0, 0, 0,  0.22312, 0, 0, 0,  0.22981};

    // Parse input
    if(argc != 2) {
        WARNING_PRINT("        CUDA & C++: Missing input argument !\n");
        throw;
    }
    std::string filename = std::string(argv[1]);

    // declare this first and make sure we always have a new output
    std::ofstream outputstream1(pathname + filename + outputext1);

    Eigen::VectorXd q0(NUM_FACTORS); q0.setZero();
    Eigen::VectorXd qd0(NUM_FACTORS); qd0.setZero();
    Eigen::VectorXd qdd0(NUM_FACTORS); qdd0.setZero();
    Eigen::VectorXd q_des(NUM_FACTORS); q_des.setZero();

    int num_obstacles = 0;
    double obstacles[MAX_OBSTACLE_NUM * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3] = {0.0};

    std::ifstream inputstream(pathname + filename + inputext);
    if (!inputstream.is_open()) {
        WARNING_PRINT("        CUDA & C++: Error reading input files !\n");
        outputstream1 << -1;
        outputstream1.close();
        throw;
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> q0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> qd0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> qdd0[i];
    }
    for (int i = 0; i < NUM_FACTORS; i++) {
        inputstream >> q_des[i];
    }
    inputstream >> num_obstacles;
    if (num_obstacles > MAX_OBSTACLE_NUM || num_obstacles < 0) {
        WARNING_PRINT("Number of obstacles larger than MAX_OBSTACLE_NUM !\n");
        outputstream1 << -1;
        outputstream1.close();
        throw;
    }
    if (num_obstacles > 0) {
        for (int i = 0; i < num_obstacles * (MAX_OBSTACLE_GENERATOR_NUM + 1) * 3; i++) {
            inputstream >> obstacles[i];
        }
    }

    inputstream.close();

    double t_plan = 1.0; // optimize the distance between q_des and the desired trajectories at t_plan
     
    /*
Section II:
    Initialize all polynomial zonotopes, including links and torques
*/
    Obstacles O(obstacles, num_obstacles); 

    auto start1 = std::chrono::high_resolution_clock::now();

    omp_set_num_threads(NUM_THREADS);
    int openmp_s_ind = 0; // openmp loop index

    /*
    Section II.A: Create JRS online
    */
    BezierCurve traj(q0, qd0, qdd0);

    try {
        #pragma omp parallel for shared(traj) private(openmp_s_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(openmp_s_ind = 0; openmp_s_ind < NUM_TIME_STEPS; openmp_s_ind++) {
            traj.makePolyZono(openmp_s_ind);
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error creating JRS! Check previous error message!");
        return -1;
    }

    /*
    Section II.B: Compute link PZs and nominal torque PZs
    */
    KinematicsDynamics kd(&traj);
    Eigen::Matrix<double, 3, 3 + 3> link_independent_generators[NUM_TIME_STEPS * NUM_JOINTS];

    try {
        #pragma omp parallel for shared(kd, link_independent_generators) private(openmp_s_ind) schedule(static, NUM_TIME_STEPS / NUM_THREADS)
        for(openmp_s_ind = 0; openmp_s_ind < NUM_TIME_STEPS; openmp_s_ind++) {
            // compute link PZs through forward kinematics
            kd.fk(openmp_s_ind);

            // reduce non-only-k-dependent generators so that slice takes less time
            for (int i = 0; i < NUM_JOINTS; i++) {
                link_independent_generators[openmp_s_ind * NUM_JOINTS + i] = kd.links(i, openmp_s_ind).reduce_link_PZ();
            }

            // compute nominal torque
            kd.rnea_nominal(openmp_s_ind);

            // compute interval torque
            kd.rnea_interval(openmp_s_ind);

            // compute max disturbance (stored in u_nom_int)
            for (int i = 0; i < NUM_FACTORS; i++) {
                kd.u_nom_int(i, openmp_s_ind) = kd.u_nom_int(i, openmp_s_ind) - kd.u_nom(i, openmp_s_ind);
            }

            // reduce non-only-k-dependent generators so that slice takes less time
            for (int i = 0; i < NUM_FACTORS; i++) {
                kd.u_nom(i, openmp_s_ind).reduce();
            }
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error computing link PZs and nominal torque PZs! Check previous error message!");
        return -1;
    }

    /*
    Section II.C: Compute robust input bound
    */
    // the radius of the torque PZs
    Eigen::MatrixXd torque_radius(NUM_FACTORS, NUM_TIME_STEPS);
    torque_radius.setZero();

    try {
        for(int t_ind = 0; t_ind < NUM_TIME_STEPS; t_ind++) {
            // (1) add the bound of robust input (||v||)
            Interval rho_max_temp = Interval(0.0);
            for (int i = 0; i < NUM_FACTORS; i++) {
                // compute norm of disturbance
                MatrixXInt temp = kd.u_nom_int(i, t_ind).toInterval(); // should be a 1-dim Interval
                rho_max_temp += temp(0) * temp(0);

                torque_radius(i, t_ind) = alpha * (M_max - M_min) * eps + 0.5 * max(abs(temp(0).lower()), abs(temp(0).upper()));
            }
            rho_max_temp = sqrt(rho_max_temp);
            
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += 0.5 * rho_max_temp.upper();
            }

            // (2) add the radius of the nominal input PZ (after reducing)
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += kd.u_nom(i, t_ind).independent(0);
            }

            // (3) add friction
            for (int i = 0; i < NUM_FACTORS; i++) {
                torque_radius(i, t_ind) += friction[i];
            }

            // so that torque_radius would be the radius of the total control input PZ from now
        }
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error computing torque PZs! Check previous error message!");
        return -1;
    }

    /*
    Section II.D: Buffer obstacles and initialize collision checking hyperplanes
    */
    try {
        O.initializeHyperPlane(link_independent_generators);
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error initializing collision checking hyperplanes! Check previous error message!");
        return -1;
    }

    auto stop1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    cout << "        CUDA & C++: Time taken by generating reachable sets: " << duration1.count() << " milliseconds" << endl;

/*
Section III:
    Solve the optimization problem using IPOPT
*/
    auto start2 = std::chrono::high_resolution_clock::now();

    SmartPtr<armtd_NLP> mynlp = new armtd_NLP();
    try {
	    mynlp->set_parameters(q_des, t_plan, &traj, &kd, &torque_radius, &O);
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error initializing Ipopt! Check previous error message!");
        return -1;
    }

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", IPOPT_OPTIMIZATION_TOLERANCE);
	app->Options()->SetNumericValue("max_cpu_time", IPOPT_MAX_CPU_TIME);
	app->Options()->SetIntegerValue("print_level", IPOPT_PRINT_LEVEL);
    app->Options()->SetStringValue("mu_strategy", IPOPT_MU_STRATEGY);
    app->Options()->SetStringValue("linear_solver", IPOPT_LINEAR_SOLVER);
	app->Options()->SetStringValue("hessian_approximation", "limited-memory");

    // For gradient checking
    // app->Options()->SetStringValue("output_file", "ipopt.out");
    // app->Options()->SetStringValue("derivative_test", "first-order");
    // app->Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
    // app->Options()->SetNumericValue("derivative_test_tol", 1e-6);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if( status != Solve_Succeeded ) {
		WARNING_PRINT("Error during initialization!");
        outputstream1 << -1 << '\n';
        outputstream1.close();
        throw;
    }

    try {
        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);
    }
    catch (int errorCode) {
        WARNING_PRINT("        CUDA & C++: Error solving optimization problem! Check previous error message!");
        return -1;
    }
	
    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start2);

    if (status == Maximum_CpuTime_Exceeded) {
        cout << "        CUDA & C++: Ipopt maximum CPU time exceeded!\n";
    }
    
    if (status == Invalid_Option) {
        cout << "        CUDA & C++: Cannot find HSL library! Need to put libcoinhsl.so in proper path!\n";
    }
    else {
        cout << "        CUDA & C++: Time taken by Ipopt: " << duration2.count() << " milliseconds" << endl;
    }

/*
Section IV:
    Prepare output
*/
    // set precision to 10 decimal digits
    outputstream1 << std::setprecision(10);

    // output k_opt
    if (mynlp->feasible) {
        for (int i = 0; i < NUM_FACTORS; i++) {
            outputstream1 << mynlp->solution[i] << '\n';
        }
    }
    else {
        outputstream1 << -1 << '\n';
    }

    // output time cost (in milliseconds) in C++
    outputstream1 << duration1.count() + duration2.count();
    outputstream1.close();

    // output FRS and other information, you can comment them if they are unnecessary
    std::ofstream outputstream2(pathname + filename + outputext2);
    outputstream2 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            for (int l = 0; l < 3; l++) {
                outputstream2 << mynlp->link_sliced_center[i * NUM_JOINTS + j](l) << ' ';
            }
            outputstream2 << '\n';
        }
    }
    outputstream2.close();

    std::ofstream outputstream3(pathname + filename + outputext3);
    outputstream3 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3 + 3; l++) {
                    outputstream3 << link_independent_generators[i * NUM_JOINTS + j](k, l) << ' ';
                }
                outputstream3 << '\n';
            }
        }
    }
    outputstream3.close();

    std::ofstream outputstream4(pathname + filename + outputext4);
    outputstream4 << std::setprecision(10);
    for (int i = 0; i < NUM_TIME_STEPS; i++) {
        for (int j = 0; j < NUM_FACTORS; j++) {
            outputstream4 << torque_radius(j, i) << ' '; // this is radius of final control input
        }
        outputstream4 << '\n';
    }
    outputstream4.close();

    std::ofstream outputstream5(pathname + filename + outputext5);
    outputstream5 << std::setprecision(6);
    for (int i = 0; i < mynlp->constraint_number; i++) {
        outputstream5 << mynlp->g_copy[i] << '\n';
    }
    outputstream5.close();

    return 0;
}
