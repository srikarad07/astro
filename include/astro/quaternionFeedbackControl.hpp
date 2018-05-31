/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#ifndef QUATERNION_FEEDBACK_CONTROL_HPP
#define QUATERNION_FEEDBACK_CONTROL_HPP

#include <Eigen/Eigen>

namespace astro 
{
//  Compute the control authority on the attitude representation quaternion.  
/*  Computes the control required on the quaternion based on the error. Error is defined as the 
*   difference between commanded quaternion (reference quaternion) and the current quaternion. 
*   The expression for the quaternion feedback controller is given as per Spacecraft Dynamics 
*   and Control (Bong Wie):
*   /f[
*           u = -K q_e  -C \omega
*   \f] 
*   where, $u = (u_1, u_2, u_3)$ is the control torque vector, $q_e = (q_{1e}, q_{2e}, q_{3e}$ 
*   is the attitude error in quaternion representation, $\omega = (\omega_1, \omega_2, \omega_3)$
*   is the angular velocity vector and $K$ & $C$ are the control gain matrices.  
* 
*   @tparam                     Vector3                          3-vector type 
*   @tparam                     Vector4                          4-vector type
*   @param[in]                  quaternionReference              Desired or commanded quaternion vector 
*   @param[in]                  quaternionCurrent                Current quaternion vector 
*   @param[in]                  quaternionControlGainMatrix      Control gain vector for the quaternion 
*   @param[in]                  angularVelocityControlGainMatrix Control gain vector for the angularVelocity 
*   @return                     controlTorque                    Control input torque    
*
*/

template < typename Vector3, typename Vector4, typename Real >
Vector3 computeQuaternionControlTorque( const Vector4 quaternionReference, 
                                        const Vector4 quaternionCurrent, 
                                        const Vector3 angularVelocity, 
                                        const Real quaternionControlGain, 
                                        const Vector3 angularVelocityControlGainMatrix )    
{
    Vector4 quaternionError; 
    quaternionError[0]          =   quaternionReference[3]*quaternionCurrent[0] + 
                                    quaternionReference[2]*quaternionCurrent[1] - 
                                    quaternionReference[1]*quaternionCurrent[2] - 
                                    quaternionReference[0]*quaternionCurrent[3]; 
    quaternionError[1]          =  -quaternionReference[2]*quaternionCurrent[0] + 
                                    quaternionReference[3]*quaternionCurrent[1] + 
                                    quaternionReference[0]*quaternionCurrent[2] - 
                                    quaternionReference[1]*quaternionCurrent[3];
    quaternionError[2]          =   quaternionReference[1]*quaternionCurrent[0] - 
                                    quaternionReference[0]*quaternionCurrent[1] + 
                                    quaternionReference[3]*quaternionCurrent[2] - 
                                    quaternionReference[2]*quaternionCurrent[3];  
    quaternionError[3]          =   quaternionReference[0]*quaternionCurrent[0] + 
                                    quaternionReference[1]*quaternionCurrent[1] + 
                                    quaternionReference[2]*quaternionCurrent[2] + 
                                    quaternionReference[3]*quaternionCurrent[3];

    Vector3 controlTorque; 
    controlTorque[0]            =  - quaternionControlGain*quaternionError[0]  
                                        - angularVelocityControlGainMatrix[0]*angularVelocity[0];
    controlTorque[1]            =  - quaternionControlGain*quaternionError[1]  
                                        - angularVelocityControlGainMatrix[1]*angularVelocity[1]; 
    controlTorque[2]            =  - quaternionControlGain*quaternionError[2]  
                                        - angularVelocityControlGainMatrix[2]*angularVelocity[2];
    
    return controlTorque; 
} // template

} // namespace astro 

#endif 