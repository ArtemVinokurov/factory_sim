#include <kdl_parser/kdl_parser.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <eigen3/Eigen/Core>



class ikSolver
{

    private:
        KDL::Tree tree;
        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive *fk_solver;
        KDL::ChainIkSolverVel_wdls *ik_solver;
        KDL::JntArray jnt_min, jnt_max;

        KDL::ChainJntToJacSolver *jnt2jac;
        KDL::Jacobian  jac;
        Eigen::MatrixXd jacTop;
        Eigen::MatrixXd U;
        Eigen::VectorXd S;
        Eigen::MatrixXd V;
        Eigen::VectorXd tmp;

        unsigned int svd_iter;
        unsigned int max_iter;
        unsigned int nj;
        unsigned int ns;
        double eps;

    public:
        ikSolver();
        ~ikSolver();

        void getJointLimits(std::string filename, KDL::JntArray &jnt_min, KDL::JntArray &jnt_max);	
        int getNumJoints();
        void getJointLimits(KDL::JntArray & j_min, KDL::JntArray & j_max);
        void randInit(KDL::JntArray &randJnt);
        void solveFkDouble(std::vector<double> &q_in, std::vector<double> &p_out);
        void solveFkJntArray(KDL::JntArray &q_in, KDL::Frame &p_out);
        bool solvePoseIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose);
        bool solvePosOnlyIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose);
        void solveHybridIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, double threshold, bool verbose);
        bool clamp(std::vector<double> &q_targ);
        int myCartToJntVel(const KDL::JntArray& q_init_jnt_arr, const KDL::Twist& p_delta, KDL::JntArray& q_delta);
        bool reject(std::vector<double> q_init, std::vector<double> q_targ, double threshold);
        
}