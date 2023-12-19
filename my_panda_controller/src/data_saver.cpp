#include "data_saver.h"

#include <string.h>
#include <time.h>


// Get current date/time, format is YYYY-MM-DD-HH-mm-ss
std::string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}

void DataSaver::openfile()
{
    // generate filepath
    std::string homedir = getenv("HOME");
    std::string filename = currentDateTime();
    std::string path = homedir + "/" + filename + ".csv";

    myfile.open(path);
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
