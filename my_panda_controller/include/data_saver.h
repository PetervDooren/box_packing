#pragma once

#include <iostream>
#include <fstream>

#include <franka/robot.h>
#include <controller.h>

class DataSaver {
public:
    // write data to file.
    void openfile(ConstraintController& controller);
    void write(franka::RobotState& state, ConstraintController& controller);
    void closefile();
private:
    std::ofstream myfile;
};
