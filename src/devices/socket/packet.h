#ifndef JS_PACKET_H
#define JS_PACKET_H
#include <iostream>
// Data Packet description
namespace packet
{
    namespace joystick
    {
        struct ToM2slave
        {
            unsigned int id;
            double time_stamp;
            float force[2];
        };

        struct slave2ToM
        {
            unsigned int id;
            double time_stamp;
            int axis[4];
            unsigned short buttonBits;
        };
        inline std::ostream& operator<<(std::ostream& os, const ToM2slave& obj)
        {
            os << obj.force[0] << "\t" << obj.force[1];
            return os;
        }
        inline std::ostream& operator<<(std::ostream& os, const slave2ToM& obj)
        {
            os << obj.axis[0] << ", " << obj.axis[1]  << ", "<< obj.axis[2] << ", " << obj.axis[3];
            os << ", " << obj.buttonBits;
            return os;
        }
    }
}
#endif