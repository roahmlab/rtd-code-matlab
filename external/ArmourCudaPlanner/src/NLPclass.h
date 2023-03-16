#ifndef NLP_CLASS_H
#define NLP_CLASS_H

#include "Dynamics.h"
#include "CollisionChecking.h"
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

using namespace Ipopt;

class armtd_NLP: public TNLP
{
public:
    /** Default constructor */
    armtd_NLP();

    /** Default destructor */
    virtual ~armtd_NLP();

    // [set_parameters]
    bool set_parameters(
        Eigen::VectorXd& q_des_input,
        double t_plan_input,
        const BezierCurve* desired_trajectory_input,
        KinematicsDynamics* kinematics_dynamics_result_input,
        const Eigen::MatrixXd* torque_radius_input,
        Obstacles* obstacles_input,
        double u_s_input,
        double surf_rad_input
    );

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the NLP */
    virtual bool get_nlp_info(
        Index&          n,
        Index&          m,
        Index&          nnz_jac_g,
        Index&          nnz_h_lag,
        IndexStyleEnum& index_style
    );

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(
        Index   n,
        Number* x_l,
        Number* x_u,
        Index   m,
        Number* g_l,
        Number* g_u
    );

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(
        Index   n,
        bool    init_x,
        Number* x,
        bool    init_z,
        Number* z_L,
        Number* z_U,
        Index   m,
        bool    init_lambda,
        Number* lambda
    );

    virtual void compute(
        bool new_x,
        const Number* x
    );

    /** Method to return the objective value */
    virtual bool eval_f(
        Index         n,
        const Number* x,
        bool          new_x,
        Number&       obj_value
    );

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(
        Index         n,
        const Number* x,
        bool          new_x,
        Number*       grad_f
    );

    /** Method to return the constraint residuals */
    virtual bool eval_g(
        Index         n,
        const Number* x,
        bool          new_x,
        Index         m,
        Number*       g
    );

    /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
    virtual bool eval_jac_g(
        Index         n,
        const Number* x,
        bool          new_x,
        Index         m,
        Index         nele_jac,
        Index*        iRow,
        Index*        jCol,
        Number*       values
    );

    /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
    virtual bool eval_h(
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
    );

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(
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
    );
    //@}

    double t_plan = DURATION;

    double solution[NUM_FACTORS];

    bool ifFeasible = true;

    Index constraint_number = 0;

    Number* g_copy = nullptr;

    bool feasible = false;

    Eigen::Vector3d link_sliced_center[NUM_TIME_STEPS * NUM_JOINTS];
    Eigen::Vector3d dk_link_sliced_center[NUM_TIME_STEPS * NUM_JOINTS * NUM_FACTORS];

    // force constraint specific
    
    Eigen::MatrixXd force_value_center = Eigen::MatrixXd(3,NUM_TIME_STEPS);
    Eigen::MatrixXd force_value_radii = Eigen::MatrixXd(3,NUM_TIME_STEPS);
    Eigen::MatrixXd moment_value_center = Eigen::MatrixXd(3,NUM_TIME_STEPS);
    Eigen::MatrixXd moment_value_radii = Eigen::MatrixXd(3,NUM_TIME_STEPS);

    double force_constraint_ub[3*NUM_TIME_STEPS];
    double force_constraint_lb[3*NUM_TIME_STEPS];
    double force_constraint_gradient[3*NUM_TIME_STEPS*NUM_FACTORS];

private:
    /**@name Methods to block default compiler methods.
    *kinematics_dynamics_results generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
    //@{
    armtd_NLP(
       const armtd_NLP&
    );

    armtd_NLP& operator=(
       const armtd_NLP&
    );

    Eigen::VectorXd q_des;

    const BezierCurve* desired_trajectory = nullptr;

    KinematicsDynamics* kinematics_dynamics_result = nullptr;

    const Eigen::MatrixXd* torque_radius = nullptr;

    Obstacles* obstacles = nullptr;

    double u_s = 0; // is this correct or needed?

    double surf_rad = 0; // is this correct or needed?

    //@}
};

#endif