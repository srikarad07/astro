/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef QUATERNION_DIFFERENTIAL_HPP
#define QUATERNION_DIFFERENTIAL_HPP

#include <Eigen/Eigen>

namespace astro
{
//  Compute the time dependence of the two reference frames for the attitude representation
//  in quaternion vector. 
/*  Computes the derivative of the quaternion vector to capture the time dependent relationship
*   of two frames when represented in quaterions. 
*   The expression for the quaternion kinematic differential is given as per Spacecraft vehicles 
*   dynamics and control:
*   /f[
*       \dot{q}     = \frac{1}{2}(q_4\omega - \omega \times q) \\
*       \dot{q_4}   = -\frac{1}{2}\omega_T q
*   /f]
*   where $q = [q_1, q_2, q_3]$, $ \omega = [\omega_1, \omega_2, \omega_3 ]$ is the angular 
*   velocity, $q = [q_1, q_2, q_3, q_4]$ is the quaternion vector, $\omega \times q$ is $ 
*   \widetilde{\omega}q$ where $\widetilde{\omega}$ is the skew symetric matrix. 
*   
*   @tparam             Vector3                             3-vector
*   @tparam             Quaternion                          4-vector
*   @tparam             Matrix33                            3x3 matrix 
*   @param[in]          quaternion                          Quaternion vector
*   @param[in]          angularVelocity                     Angular velocity vector
*   @return             quaternionDerivative                Derivative of quaternion  
*
*/   

template < typename Vector3, typename Vector4 >
Vector4 computeQuaternionDerivative(  const Vector4 quaternion, 
                                      const Vector3 angularVelocity )
{
    Vector4 quaternionDerivative; 
    quaternionDerivative[0]     = ( angularVelocity[2] * quaternion[1] - angularVelocity[1] * 
                                        quaternion[2] + angularVelocity[0] * quaternion[3] ) / 2.0;
    quaternionDerivative[1]     = ( - angularVelocity[2] * quaternion[0] + angularVelocity[0] * 
                                        quaternion[2] + angularVelocity[1] * quaternion[3] ) / 2.0;
    quaternionDerivative[2]     = ( angularVelocity[1] * quaternion[0] - angularVelocity[0] * 
                                        quaternion[1] + angularVelocity[2] * quaternion[3] ) / 2.0;
    quaternionDerivative[3]     = ( - angularVelocity[0] * quaternion[0] - angularVelocity[1] * 
                                        quaternion[1] - angularVelocity[2] * quaternion[2] ) / 2.0;

    return quaternionDerivative;
}   

} // astro 

#endif 