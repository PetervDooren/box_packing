#pragma once

#include <iostream>
#include <fstream>

#include <franka/robot.h>

class DataSaver {
public:
    // write data to file.
    void openfile();
    void write(franka::RobotState state);
    void closefile();
private:
    std::ofstream myfile;
};
