//
// Created by bsaund on 3/31/21.
//

// Bring in my package's API, which is what I'm testing
#include <jacobian_follower/jacobian_follower.hpp>
// Bring in gtest
#include <gtest/gtest.h>
#include "../include/jacobian_follower/jacobian_follower.hpp"

// Declare a test
TEST(JacobianFollolwer, testJacobianFollowerIK) {
    JacobianFollower jf("victor", 0.005, true);
    std::string group_name = "right_arm";
    auto const jmg = jf.model_->getJointModelGroup(group_name);
    const kinematics::KinematicsBaseConstPtr &solver = jmg->getSolverInstance();
    std::cout << "Sovler IK Frame: " << solver->getBaseFrame() << "\n";
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}