//------------------------------------------------------------------------------
#include "tools/CToolTrioGripper.h"
#include "graphics/CTriangleArray.h"
//------------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cToolTrioGripper.

    \param  a_parentWorld  World in which the tool will operate.
*/
//==============================================================================
cToolTrioGripper::cToolTrioGripper(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
{
    // default gripper workspace
    m_gripperWorkspaceScale = 1.0;

    // create a haptic point contact for the thumb
    m_hapticPointThumb = new cHapticPoint(this);
    m_hapticPoints.push_back(m_hapticPointThumb);

    // create a haptic point contact for the finger0
    m_hapticPointIndexFinger = new cHapticPoint(this);
    m_hapticPoints.push_back(m_hapticPointIndexFinger);

    // create a haptic point contact for the finger1
    m_hapticPointMiddleFinger = new cHapticPoint(this);
    m_hapticPoints.push_back(m_hapticPointMiddleFinger);

    // show proxy spheres only
    setShowContactPoints(true, true);
}


//==============================================================================
/*!
    Destructor of cToolTrioGripper.
*/
//==============================================================================
cToolTrioGripper::~cToolTrioGripper()
{
    delete  m_hapticPointThumb;
    delete  m_hapticPointIndexFinger;
    delete  m_hapticPointMiddleFinger;
}


//==============================================================================
/*!
    This method computes the interaction forces between the tool and all
    objects located inside the virtual world.
*/
//==============================================================================
void cToolTrioGripper::computeInteractionForces()
{
    // convert the angle of the gripper into a position in device coordinates. 
    // this value is device dependent.
    static double gripperPositionFinger0; // index fingure position
    static double gripperPositionFinger1; // middle fingure position
    static double gripperPositionThumb;  // thumb position

    gripperPositionFinger0 = 0.05 * (1.0-m_gripperAngle);//cSinRad( 1 - m_gripperAngle + cDegToRad( 0.0));
    gripperPositionFinger1 = 0.05 * (1.0-m_gripperAngle);//cSinRad( 1 - m_gripperAngle + cDegToRad( 0.0));
    gripperPositionThumb  = 0.05  * (1.0-m_gripperAngle);//cSinRad( 1 - m_gripperAngle + cDegToRad( 0.0));

    // compute new position of thumb and finger 
    cVector3d dir_thumb = cVector3d(1,0,0);
    cVector3d dir_fingure_0 = cVector3d(-0.5, 0.8660254037844387,0.0);
    cVector3d dir_fingure_1 = cVector3d(-0.5,-0.8660254037844387,0.0);

    cVector3d pFinger0 = m_gripperWorkspaceScale * m_workspaceScaleFactor * gripperPositionFinger0 * dir_thumb;
    cVector3d pFinger1 = m_gripperWorkspaceScale * m_workspaceScaleFactor * gripperPositionFinger1 * dir_fingure_0;
    cVector3d pThumb  = m_gripperWorkspaceScale * m_workspaceScaleFactor * gripperPositionThumb  * dir_fingure_1;

    cVector3d posFinger0, posFinger1, posThumb;
 
    posFinger0 = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (1.0 * pFinger0));
    posFinger1 = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (1.0 * pFinger1));
    posThumb = m_deviceGlobalPos + cMul(m_deviceGlobalRot, (1.0 * pThumb));

    // compute forces
    cVector3d forceThumb = m_hapticPointThumb->computeInteractionForces(posThumb, 
                                                                        m_deviceGlobalRot, 
                                                                        m_deviceGlobalLinVel, 
                                                                        m_deviceGlobalAngVel);

    cVector3d forceFinger0 = m_hapticPointIndexFinger->computeInteractionForces(posFinger0, 
                                                                          m_deviceGlobalRot, 
                                                                          m_deviceGlobalLinVel, 
                                                                          m_deviceGlobalAngVel);

    cVector3d forceFinger1 = m_hapticPointMiddleFinger->computeInteractionForces(posFinger1, 
                                                                          m_deviceGlobalRot, 
                                                                          m_deviceGlobalLinVel, 
                                                                          m_deviceGlobalAngVel);

    // compute torques
    double scl = 0.0;
    double factor = m_gripperWorkspaceScale * m_workspaceScaleFactor;
    if (factor > 0.0)
    {
        scl = 1.0 / factor;
    }
    cVector3d torque = scl * cAdd(cCross(cSub(posThumb, m_deviceGlobalPos), forceThumb), cCross(cSub(posFinger0, m_deviceGlobalPos), forceFinger0), cCross(cSub(posFinger1, m_deviceGlobalPos), forceFinger1));

    // compute gripper force
    double gripperForce = 0.0;

    if ((m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_OMEGA_7) ||
        (m_hapticDevice->m_specifications.m_model == C_HAPTIC_DEVICE_SIGMA_7))
    {
        cVector3d dir = posFinger0 - posThumb;
        if (dir.length() > 0.00001) 
        {
            dir.normalize ();
            cVector3d force = cProject (forceFinger0, dir);
            gripperForce = force.length();
            if (force.length() > 0.001) 
            {
                double angle = cAngle(dir, force);
                if ((angle > C_PI/2.0) || (angle < -C_PI/2.0)) gripperForce = -gripperForce;
            }
        }
    }

    // gripper damping
    double gripperAngularVelocity = 0.0;
    m_hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);
    double gripperDamping = -0.1 * m_hapticDevice->m_specifications.m_maxGripperAngularDamping * gripperAngularVelocity;

    // finalize forces, torques and gripper force
    cVector3d globalForce = forceThumb + forceFinger0 + forceFinger1;
    cVector3d globalTorque = torque;
    gripperForce = gripperForce + gripperDamping;

    // update computed forces to tool
    setDeviceGlobalForce(globalForce);
    setDeviceGlobalTorque(globalTorque);
    setGripperForce(gripperForce);
}


//==============================================================================
/*!
    This method renders the current tool using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cToolTrioGripper::render(cRenderOptions& a_options)
{
    ///////////////////////////////////////////////////////////////////////
    // render haptic points
    ///////////////////////////////////////////////////////////////////////
    int numContactPoint = (int)(m_hapticPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next haptic point
        cHapticPoint* nextContactPoint = m_hapticPoints[i];

        // render tool
        nextContactPoint->render(a_options);
    }

    ///////////////////////////////////////////////////////////////////////
    // render mesh image
    ///////////////////////////////////////////////////////////////////////
    if (m_image != NULL)
    {
        m_image->renderSceneGraph(a_options);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
