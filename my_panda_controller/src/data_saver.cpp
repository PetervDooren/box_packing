#include "data_saver.h"

void DataSaver::openfile()
{
    myfile.open ("data.csv");
    // time
    myfile << "timestamp" << ", ";
    // joint positions
    for (int i = 0; i< 7; i++)
    {
        myfile << "q" << i << ", ";
    }
    // joint velocities
    for (int i = 0; i< 7; i++)
    {
        myfile << "dq" << i << ", ";
    }
    // joint torques
    for (int i = 0; i< 7; i++)
    {
        myfile << "tau" << i << ", ";
    }
    // EE position
    for (int i = 0; i< 16; i++)
    {
        myfile << "OTEE" << i << ", ";
    }
    myfile << "\n";
}

void DataSaver::write(franka::RobotState state)
{
    // time
    myfile << state.time.toSec() << ", ";
    // joint positions
    for (int i = 0; i< 7; i++)
    {
        myfile << state.q[i] << ", ";
    }
    // joint velocities
    for (int i = 0; i< 7; i++)
    {
        myfile << state.dq[i] << ", ";
    }
    // joint torques
    for (int i = 0; i< 7; i++)
    {
        myfile << state.tau_ext_hat_filtered[i] << i << ", ";
    }
    // EE position
    for (int i = 0; i< 16; i++)
    {
        myfile << state.O_T_EE[i] << ", ";
    }
    myfile << "\n";
}

void DataSaver::closefile()
{
    myfile.close();
}
