#ifndef _MANIPULATOR_PACKET_H_
#define _MANIPULATOR_PACKET_H_

#include <sys/types.h>

namespace packet
{
namespace manipulator
{
////////////////////////////////////////////////////////////////////////////////////////////
/// ToM2slave
////////////////////////////////////////////////////////////////////////////////////////////
struct ToM2slave
{
    /// packet information
    float timestamp_master;             // time stamp of this packet
    float timestamp_slave;
    int packet_id;                      // packet id

    /// Cartesian Space EE Feedback
    double position_ee[3];              // end-effector position of the Teleop-Man: frame{teleop_link5}
    double quaternion_ee[4];            // end-effector orientation of the Teleop-Man. expressed in quaternion

    /// Joint Space Feedback
    double q[5];                        // joint position of the Teleop-Man
    double torque[5];                   // joint torque of the Teleop-Man

    /// Hand information
    double closure_cmd_fingers[4];      // how do we control the fingers?
};

////////////////////////////////////////////////////////////////////////////////////////////
/// slave2ToM
////////////////////////////////////////////////////////////////////////////////////////////
struct slave2ToM
{
    /// packet information
    float timestamp_master;             // time stamp of this packet
    float timestamp_slave;
    int packet_id;                    // packet id

    /// control mode
    int teleopman_mode;                 // the control mode of the teleop-man: 1 = cartesian impedance control, 2 = joint impedance control, 3. cartesian position/velocity control ....
    int hand_mode;

    /// Cartesian Space EE Feedback
    double position_ee[3];              // current end-effector position of the Teleop-Man: frame{teleop_link5}
    double quaternion_ee[4];            // current end-effector orientation (expressed by quaternion) of the Teleop-Man: frame{teleop_link5}
    double cmd_position_ee[3];          // last commanded position for the IK controller (input to IK solver)
    double cmd_quaternion_ee[4];        // last commanded orientation for the IK controller (input to IK solver)

    /// Joint Space Feedback
    double q[5];                        // joint position of the Teleop-Man
    double torque[5];                   // joint torque of the Teleop-Man

    /// F/T wrist sensor information
    double force_ee[3];                 // the force (Cartesian) sensed by the F/T sensor installed on the wrist
    double torque_ee[3];                // the torque (Cartesian) sensed by the F/T sensor installed on the wrist

    /// Hand information
    double i_fingers[4];                // the actuator current (in Amper) for each finger
    double q_fingers[4];                // the actuator position for each finger.

    /// Pressure hand sensor information
    double pressure_fingers[12];        // the pressure sensor data installed on each phalanx
};

}
}

#endif //_MANIPULATOR_PACKET_H_
