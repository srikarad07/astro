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

// TO DO : 
// Create a generic differential equation function. The lectures by hanspeter schaub 
// give a good insight into the problem. 
// The function needs to be tested thoroughly. For now it is has been tested only fotr
// a off case! theta_2 = 0 deg. 

#include <Eigen/Eigen>

namespace astro
{

//  Computes the euler rates to capture the time dependent relationship between two frames.
/*  Computes the euler rates for the time dependence between two reference frames. 
*   The expression is defined as per 'Spacecraft Vehicle Dynamics and Control Second Edition' 
*   by Bong Wie (Page-389). The expression describes the relationship of euler rates with the 
*   angular velocity expresses in body reference frame relative to newtonian inertial frame. 
*    
*/

// template< typename Vector3, typename Matrix33 > 
template< typename Vector3 > 
Vector3 eulerKinematicDifferential( const Vector3& rotationSequence, 
                                    const Vector3& eulerAngles,
                                    const Vector3& angularRates )
                                    // const Vector3 orbitalAngularVelocity )
{
    // Vector3 rotation321(3, 2, 1);
    typedef Eigen::Matrix< double, 3, 3 > Matrix33;  
    Matrix33 directionCosineMatrix, tempDirectionCosineMatrix;

    // if ( true == rotationSequence.isapprox(rotation321) )
    if ( 0 == 0 )
    {
        directionCosineMatrix       =   Eigen::AngleAxisd(sml::convertDegreesToRadians(-90.0), 
                                                            Eigen::Vector3d::UnitX() )*
                                        Eigen::AngleAxisd(sml::convertDegreesToRadians(0.0), 
                                                            Eigen::Vector3d::UnitY() )*
                                        Eigen::AngleAxisd(sml::convertDegreesToRadians(-90.0), 
                                                            Eigen::Vector3d::UnitZ() );
        tempDirectionCosineMatrix   =   Eigen::AngleAxisd(sml::convertDegreesToRadians(-90.0),
                                                            Eigen::Vector3d::UnitX() )*
                                        Eigen::AngleAxisd(sml::convertDegreesToRadians(0.0),
                                                            Eigen::Vector3d::UnitY() );
    }
     

    // k = Eigen::AngleAxisd(sml::convertDegreesToRadians(-90.0), Eigen::Vector3d::UnitX() )*
    //     Eigen::AngleAxisd(sml::convertDegreesToRadians(0.0), Eigen::Vector3d::UnitY() );
    // Eigen::Matrix3d temp; 
    // temp = k.transpose();

    // Vector3 col1 = m.col(2);   
    // Vector3 col2 = k.col(1); 
    // Vector3 col3 = Eigen::Vector3d::UnitX(); 

    Matrix33 angularVelocityRotationMatrix;  
    angularVelocityRotationMatrix.col(0) = directionCosineMatrix.col(2); 
    angularVelocityRotationMatrix.col(1) = tempDirectionCosineMatrix.col(1);
    angularVelocityRotationMatrix.col(2) = Eigen::Vector3d::UnitX(); 

    Vector3 attitudeDerivative = angularVelocityRotationMatrix * angularRates; 
    return attitudeDerivative; 
}

}

#endif // EULER_KINEMATIC_DIFFERENTIAL_HPP 