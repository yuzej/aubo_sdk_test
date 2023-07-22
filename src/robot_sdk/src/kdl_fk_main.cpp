#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <urdf/model.h>

using namespace KDL;
using namespace std;

int main(int argc,char** argv){
    Tree my_tree;
    kdl_parser::treeFromFile("/home/amax/aubo_sdk_test/src/robot_sdk/src/robot/aubo_i5_ft_peg.urdf",my_tree);


    // std::string urdf_file = "/home/amax/aubo_sdk_test/src/robot_sdk/src/robot/aubo_i5_ft_peg.urdf";
    // urdf::Model model;
    // if (!model.initFile(urdf_file)){
    //     printf("Failed to parse urdf file");
    //     return false;
    // }
    // if (!kdl_parser::treeFromUrdfModel(model,my_tree)){
    //   printf("Failed to construct kdl tree");
    //   return false;
    // }
    bool exit_value;
    Chain chain;
    //printf(my_tree.getNrOfJoints());
    exit_value = my_tree.getChain("base_Link","peg_end",chain);
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();
    int a = chain.getNrOfJoints();
    printf("%d",a);
    //printf(nj);
    JntArray jointpositions = JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf("Enter the position of joint %i: ",i);
        scanf("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }

    Frame cartpos;
    //printf(jointpositions);
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos << std::endl;
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate forward kinematics : ");
    }
}
