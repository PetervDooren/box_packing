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

void DataSaver::openfile(ConstraintController& controller)
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


    // reference point
    myfile << "point_dx, point_dy, point_dz, ";

    // constraint values + ref
    std::vector<Constraint> constraints = controller.getconstraints();
    for (int i = 0; i<constraints.size(); i++)
    {
        myfile << "constraint_" << constraints[i].id << "_value, ";
    }
    for (int i = 0; i<constraints.size(); i++)
    {
        myfile << "constraint_" << constraints[i].id << "_velocity_reference, ";
    }

    // dq_desired
    for (int i = 0; i< 7; i++)
    {
        myfile << "dq_d" << i << ", ";
    }

    // Jacobian
    for (int i = 0; i< 42; i++)
    {
        myfile << "J" << i << ", ";
    }

    for (int i= 0; i < constraints.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            myfile << "M" << i << j << ", ";
        }
    }

    myfile << "\n";
}

void DataSaver::write(franka::RobotState& state, ConstraintController& controller)
{
    // ROBOT STATE
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

    // CONTROLLER STATE
    // reference position
    Eigen::Vector3d position_d = controller.getDesiredPosition();
    myfile << position_d.x() << ", " << position_d.y() << ", " << position_d.z() << ", ";

    // constraint values + ref
    std::vector<double> constraint_values = controller.getconstraintValues();
    for (int i = 0; i<constraint_values.size(); i++)
    {
        myfile << constraint_values[i] << ", ";
    }
    std::vector<double> constraint_vel_refs = controller.getconstraintVelReferences();
    for (int i = 0; i<constraint_vel_refs.size(); i++)
    {
        myfile << constraint_vel_refs[i] << ", ";
    }
    std::array<double, 7> dq_d = controller.getDqLog();
    // dq_desired
    for (int i = 0; i< 7; i++)
    {
        myfile << dq_d[i] << ", ";
    }
    
    // Jacobian
    std::array<double, 42> J = controller.getJacobian();
    for (int i = 0; i< 42; i++)
    {
        myfile << J[i] << ", ";
    }

    // interaction matrix
    Eigen::MatrixXd M = controller.getInteractionMatrix();
    int nrows = static_cast<int>(M.rows()); // should be equal to nr of constraints
    int ncols = static_cast<int>(M.cols()); // should be equal to 7
    for (int i= 0; i < nrows; i++)
    {
        for (int j = 0; j < ncols; j++)
        {
            myfile << M(i,j) << ", ";
        }
    }

    // ENDLINE
    myfile << "\n";
}

void DataSaver::closefile()
{
    myfile.close();
}
