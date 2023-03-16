#ifndef DYNAMICS_CPP
#define DYNAMICS_CPP

#include "Dynamics.h"

KinematicsDynamics::KinematicsDynamics(BezierCurve* traj_input) {
    traj = traj_input;

    // pre-allocate memory
    links = PZsparseArray(NUM_FACTORS * 3, NUM_TIME_STEPS);
    mass_nominal_arr = PZsparseArray(NUM_JOINTS, 1);
    mass_uncertain_arr = PZsparseArray(NUM_JOINTS, 1);
    I_nominal_arr = PZsparseArray(NUM_JOINTS, 1);
    I_uncertain_arr = PZsparseArray(NUM_JOINTS, 1);
    u_nom = PZsparseArray(NUM_FACTORS, NUM_TIME_STEPS);
    u_nom_int = PZsparseArray(NUM_FACTORS, NUM_TIME_STEPS);
    f_c_int = PZsparseArray(1,NUM_TIME_STEPS);
    n_c_int = PZsparseArray(1,NUM_TIME_STEPS);
    f_c_nom = PZsparseArray(1,NUM_TIME_STEPS);
    n_c_nom = PZsparseArray(1,NUM_TIME_STEPS);
    r = PZsparseArray(NUM_FACTORS, 1);
    Mr = PZsparseArray(NUM_FACTORS, NUM_TIME_STEPS);

    // initialize robot properties
    for (int i = 0; i < NUM_JOINTS; i++) {
        trans_matrix(i, 0) = Eigen::MatrixXd::Zero(3, 1);
        trans_matrix(i, 0)(0) = trans[3 * i];
        trans_matrix(i, 0)(1) = trans[3 * i + 1];
        trans_matrix(i, 0)(2) = trans[3 * i + 2];

        com_matrix(i, 0) = Eigen::MatrixXd::Zero(3, 1);
        com_matrix(i, 0)(0) = com[3 * i];
        com_matrix(i, 0)(1) = com[3 * i + 1];
        com_matrix(i, 0)(2) = com[3 * i + 2];

        Eigen::MatrixXd mass_matrix(1, 1);
        mass_matrix(0) = mass[i];
        mass_nominal_arr(i) = PZsparse(mass_matrix);
        mass_uncertain_arr(i) = PZsparse(mass_matrix, mass_uncertainty);

        Eigen::Matrix3d inertia_matrix;
        for (int j = 0; j < 9; j++) {
            inertia_matrix(j) = inertia[i * 9 + j]; // This may not be right...
        }
        I_nominal_arr(i) = PZsparse(inertia_matrix);
        I_uncertain_arr(i) = PZsparse(inertia_matrix, inertia_uncertainty);

        if (i < NUM_FACTORS) {
            r(i) = PZsparse(0, Interval(-eps, eps));
        }
    }

    trans_matrix(NUM_JOINTS, 0) = Eigen::MatrixXd::Zero(3, 1);
    trans_matrix(NUM_JOINTS, 0)(0) = trans[3 * NUM_JOINTS];
    trans_matrix(NUM_JOINTS, 0)(1) = trans[3 * NUM_JOINTS + 1];
    trans_matrix(NUM_JOINTS, 0)(2) = trans[3 * NUM_JOINTS + 2];

    // define original link PZs
    links = PZsparseArray(NUM_JOINTS, NUM_TIME_STEPS);

    for (int i = 0; i < NUM_JOINTS; i++) {
        PZsparseArray link(3, 1);

        for (int j = 0; j < 3; j++) {
            uint64_t degree[1][NUM_FACTORS * 6] = {0};
            degree[0][NUM_FACTORS * (j + 1)] = 1; // use qde, qdae, qdde for x, y, z generator
            double temp = link_zonotope_generators[i][j];
            link(j, 0) = PZsparse(link_zonotope_center[i][j], &temp, degree, 1);
        }

        links(i, 0) = stack(link);

        for (int j = 1; j < NUM_TIME_STEPS; j++) {
            links(i, j) = links(i, 0);
        }
    }
}

void KinematicsDynamics::fk(uint t_ind) {
    PZsparse FK_R = PZsparse(0, 0, 0); // identity matrix
    PZsparse FK_T(3, 1);
    int j = 0;

    for (int i = 0; i < NUM_JOINTS; i++) {
        PZsparse P(trans_matrix(i, 0));
        
        FK_T = FK_T + FK_R * P;
        FK_R = FK_R * traj->R(i, t_ind);
        
        links(i, t_ind) = FK_R * links(i, t_ind) + FK_T;
    }
}

void KinematicsDynamics::rnea(uint t_ind,
                              PZsparseArray& mass_arr,
                              PZsparseArray& I_arr,
                              PZsparseArray& u,
                              PZsparseArray& f_c,
                              PZsparseArray& n_c,
                              bool setGravity) {
    PZsparse w(3, 1);
    PZsparse wdot(3, 1);
    PZsparse w_aux(3, 1);
    PZsparse linear_acc(3, 1);

    PZsparseArray F(NUM_JOINTS, 1);
    PZsparseArray N(NUM_JOINTS, 1);

    if (setGravity) { // set gravity
        // directly modify the center of the PZ instance
        linear_acc.center(2) = gravity;
    }

    // RNEA forward recursion
    for (int i = 0; i < NUM_JOINTS; i++) {
        // NOTE:
        // This is just a simplified implementation!!!
        // We assume all fixed joints are at the end and the revolute joints are consecutive
        if (axes[i] != 0) { // revolute joints
            // line 16
            linear_acc = traj->R_t(i, t_ind) * (linear_acc 
                                                 + cross(wdot, trans_matrix(i, 0)) 
                                                 + cross(w, cross(w_aux, trans_matrix(i, 0))));

            // line 13
            w = traj->R_t(i, t_ind) * w;
            w.addOneDimPZ(traj->qd_des(i, t_ind), abs(axes[i]) - 1, 0);

            // line 14
            w_aux = traj->R_t(i, t_ind) * w_aux;

            // line 15
            wdot = traj->R_t(i, t_ind) * wdot;

            PZsparse temp(3, 1); // temp = joint_vel(robot_params.q_index(i))*z(:,i)
            temp.addOneDimPZ(traj->qd_des(i, t_ind), abs(axes[i]) - 1, 0);

            wdot = wdot + cross(w_aux, temp);

            wdot.addOneDimPZ(traj->qdda_des(i, t_ind), abs(axes[i]) - 1, 0);

            // line 14
            w_aux.addOneDimPZ(traj->qda_des(i, t_ind), abs(axes[i]) - 1, 0);
        }
        else { // fixed joints
            // line 16

            // PZsparse test1 = (linear_acc 
            //                                      + cross(wdot, trans_matrix(i, 0)) 
            //                                      + cross(w, cross(w_aux, trans_matrix(i, 0))));
            // cout << "PZ1" << endl << test1 << endl;
            // PZsparse test2 = traj->R_t(i, t_ind);
            // cout << "PZ2" << endl << test2 << endl;
            
            linear_acc = traj->R_t(i, t_ind) * (linear_acc 
                                                 + cross(wdot, trans_matrix(i, 0)) 
                                                 + cross(w, cross(w_aux, trans_matrix(i, 0))));

            // line 13
            w = traj->R_t(i, t_ind) * w;

            // line 14
            w_aux = traj->R_t(i, t_ind) * w_aux;

            // line 15
            wdot = traj->R_t(i, t_ind) * wdot;
        }

        // line 23 & 27
        F(i, 0) = mass_arr(i, 0) * (linear_acc
                                     + cross(wdot, com_matrix(i, 0))
                                     + cross(w, cross(w_aux, com_matrix(i, 0))));

        // line 29
        N(i, 0) = I_arr(i, 0) * wdot + cross(w_aux, (I_arr(i, 0) * w));
    }

    PZsparse f(3, 1);
    PZsparse n(3, 1);

    // RNEA reverse recursion
    for (int i = NUM_JOINTS - 1; i >= 0; i--) {
        // line 29
        n = N(i, 0)
            + traj->R(i + 1, t_ind) * n
            + cross(com_matrix(i, 0), F(i, 0))
            + cross(trans_matrix(i + 1, 0), traj->R(i + 1, t_ind) * f);

        // line 28
        f = traj->R(i + 1, t_ind) * f + F(i, 0);

        if (axes[i] != 0) {
            u(i, t_ind) = n(abs(axes[i]) - 1, 0);

            u(i, t_ind) = u(i, t_ind) + armature[i] * traj->qdda_des(i, t_ind);

            u(i, t_ind) = u(i, t_ind) + damping[i] * traj->qd_des(i, t_ind);

            // friction is directly cut on the torque limits
        }

        if (i == NUM_JOINTS - 1) {
            f_c(0,t_ind) = f; // not sure how to assign these
            n_c(0,t_ind) = n; // not sure how to assign these
            // note: should change this at some point to be 
            // specifically for a specified list of contact 
            // joints and not just the last joint.
        }
    }
}

#endif