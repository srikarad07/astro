/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef EULER_KINEMATIC_DIFFERENTIAL_HPP
#define EULER_KINEMATIC_DIFFERENTIAL_HPP

/* The time dependent relatinoship between two reference frames is described by the 
*  kinematic differntial equations. This particular case deals with the kinematic 
*  differential equation for a sequence of 3->2->1 euler angle rotation.  
*/

#include <iostream>

#include <Eigen/Eigen>

namespace astro
{

//  Computes the euler rates to capture the time dependent relationship between two frames.
/*  Computes the euler rates for the time dependence between two reference frames for a 
*   rigid body rotating around a central body with a constant mean angular motion. 
*   The expression is defined as per 'Spacecraft Vehicle Dynamics and Control Second Edition' 
*   by Bong Wie (Page-389). 
*   
*   The expression describes the vector angular velocity expressed in  
*   body reference frame relative to newtonian inertial frame. 
*   /f[
*       \omega_{BN} = \omega_{BA} + \omega_{AN}
*   /f] 
*   where $\omega_{BN}$ is the angular velocity of body fixed reference frame relative to  
*   inertial frame, $\omega_{BA}$ is the angular velocity of body fixed reference frame 
*   relative to Local Veritcal Local Horizontal (LCLH) reference frame and $\omega_{AN}$
*   is angular velocity of LVLH reference frame relative to inertial frame (which is the
*   orbital angular velocity or the constant mean angular motion). $\omega_{BN}$ is the    
*   angular rate measured onboard the spaceraft $[\omega_1, \omega_2, \omega_3]$. 
*   Substituting the values of $\omega_{BA}$ and \omega_{AN} expressed in respective
*   reference frames desribes the kinematics. The inverse kinematics describes the 
*   reltionship of euler rates with the angular velocity. 
*   
*   @tparam         Real                        Real type
*   @tparam         Vector3                     3-vector type
*   @tparam         Matrix33                    3x3 matrix type
*   @param[in]      rotationSequence            Matrix33 type. Represents the axis of rotation
*                                               in vector type. For ex: For a 3 -> 2 -> 1 
*                                               rotation the rotational sequence will be:
*                                               [  0, 0, 1 
*                                                  0, 1, 0 
*                                                  1, 0, 0 ] 
*   @param[in]      eulerAngles                 Vector3 type. Rotational euler angles in the 
*                                               same sequence as above. [rad] 
*   @param[in]      angularRates                Calculated angular rates. 
*   @param[in]      meanMotion                  Angular speed required by the orbit to complete
*                                               one orbit. 
*       
*/

template <typename Vector3, typename Real, typename Matrix33>
Vector3 eulerKinematicDifferential(Matrix33 &rotationSequence,
                                   Vector3 &eulerAngles,
                                   const Vector3 &angularRates,
                                   const Real meanMotion)
{
    // Matrix33 directionCosineMatrix, tempDirectionCosineMatrix, tempRotationSequence;
    Matrix33 directionCosineMatrix, tempDirectionCosineMatrix;
    Eigen::Matrix< double, 3, 2 > tempRotationSequence;

    // tempDirectionCosineMatrix is the intermediary rotation matrix. Let's say the rotation from A -> B 
    // happens, with Intermediary A' such that A -> A' -> A'' -> B. This rotation sequence is from
    // A' -> B. 
    tempRotationSequence        <<  rotationSequence.col(1),
                                    rotationSequence.col(2);

    directionCosineMatrix       = 
                astro::computeEulerAngleToDcmConversionMatrix(rotationSequence, eulerAngles);
    
    // Eigen::Matrix< double, 2, 1 > tempRotationAngles( eulerAngles[1], eulerAngles[2] ); 
    tempDirectionCosineMatrix   =   Eigen::AngleAxisd( -eulerAngles[2], rotationSequence.col(2) )*
                                    Eigen::AngleAxisd( -eulerAngles[1], rotationSequence.col(1) );

    // std::cout << "The DCM is : \n" << directionCosineMatrix << std::endl; 
    // The rotation of the spacecraft around the central body happens around this axis. 
    Vector3 a(directionCosineMatrix.col(2));
    // a2 ?= ;
    // std::cout << "The value of a is: \n" << a << std::endl; 
    Matrix33 angularVelocityRotationMatrix;

    // A - A' happens around this axis. (First rotation)
    angularVelocityRotationMatrix.col(0) = directionCosineMatrix.col(2);
    // A' - B happens aroud this axis. (Second rotation)
    angularVelocityRotationMatrix.col(1) = tempDirectionCosineMatrix.col(1);
    // A'' - B happens around this axis. (Third rotation)
    angularVelocityRotationMatrix.col(2) = rotationSequence.row(2).transpose();
    
    Vector3 multiplier( angularRates[0] + meanMotion * a[0], 
                        angularRates[1] + meanMotion * a[1],
                        angularRates[2] + meanMotion * a[2] );
    // std::cout << "The multiplier is: \n " << multiplier << std::endl;     
    // Vector3 attitudeDerivative = angularVelocityRotationMatrix.jacobiSvd().solve(multiplier);
    Vector3 attitudeDerivative = angularVelocityRotationMatrix.colPivHouseholderQr().solve(multiplier);
    return attitudeDerivative;
}

}

#endif // EULER_KINEMATIC_DIFFERENTIAL_HPP