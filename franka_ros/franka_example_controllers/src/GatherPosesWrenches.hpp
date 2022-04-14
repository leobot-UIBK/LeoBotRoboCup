//
// Created by paddy on 25.09.20.
//

#ifndef ROBOCUPATWORK_GATHERPOSESWRENCHES_HPP
#define ROBOCUPATWORK_GATHERPOSESWRENCHES_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <fstream>
#include <franka_msgs/FrankaState.h>

void saveVectorMatrixToFile (std::string fileName, std::vector< std::vector<double> > outMat) {
    std::ofstream out(fileName.data());
    if (!out) {
        std::cout << "No file found: " << fileName << std::endl;
        return;
    }
    int rows = (int)outMat.size();
    int cols = (int)outMat[0].size();
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            out << outMat[i][j] << "\t";
        }
        out << std::endl;
    }
    out.close();
    return;
}

//ros::Duration gatherTime(30);
std::vector< std::vector<double> > eePoses;
std::vector< std::vector<double> > externalWrenches;
//std::vector< std::vector<double> > times;
size_t counter;
size_t max_iter = 30000;
//ros::Time startTime(0.0);


void gather_poses_wrenches(const franka::RobotState &state){
    // init start time once
    //if(startTime.isZero()){
    //    startTime = state.header.stamp;
    //}
    // set times
    //ros::Time stampTime(state.header.stamp);
    //ros::Duration timeStamp = stampTime - startTime;
    //std::cout << timeStamp << std::endl;

    //times.push_back(std::vector<double>());
    //times[counter].push_back(timeStamp.toSec());

    eePoses.push_back( std::vector <double>() );
    for(int i=0; i<16; ++i) {
        eePoses[counter].push_back(state.O_T_EE[i]);
    }
    externalWrenches.push_back( std::vector <double>() );
    for(int i=0; i<6; ++i) {
        externalWrenches[counter].push_back(state.O_F_ext_hat_K[i]);
    }
    counter++;

    // define shutdown
    if(counter == max_iter){
        // Save demonstration to file
        std::cout << std::endl << "Demonstration finished. Save to file..." << std::endl;
        std::string packPath = ros::package::getPath("precision_placement");
        std::cout << "Provide the demonstration number" << std::endl;
        std::string demoNum;
        std::cin >> demoNum;
        //saveVectorMatrixToFile(packPath+"/resources/raw_demo_data/Times_"+demoNum+".txt", times);
        saveVectorMatrixToFile(packPath+"/resources/raw_demo_data/EndEffectorPose_"+demoNum+".txt", eePoses);
        saveVectorMatrixToFile(packPath+"/resources/raw_demo_data/EndEffectorWrench_"+demoNum+".txt", externalWrenches);
        ROS_INFO("All done. Terminate node with control-c");
    }
}


#endif //ROBOCUPATWORK_GATHERPOSESWRENCHES_HPP
