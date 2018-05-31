/*
* Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
* Distributed under the MIT License.
* See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#ifndef SOLAR_RADIATION_DISTURBANCE_TORQUE_HPP
#define SOLAR_RADIATION_DISTURBANCE_TORQUE_HPP

#include <iostream>

#include "astro/solarRadiationPressureAccelerationModel.hpp"

/*  Solar radiation pressure is experienced by the central body due to the difference in the 
*   locations of the center of solar pressure and the center of gravity. The torque is 
*   generated at the center of mass of the spacecraft due to the reflected solar radiation 
*   off the satellite in parts of it's orbit. 
*/

namespace astro 
{

//  Compute the solar radiation pressure torque experienced by an orbiting body. 
/*  Computes the solar radiation pressure torque experienced by an orbiting body. 
*   The expression for the torque is defined as per 'Spacecraft Dynamics and Control: 
*   A practical approach' (By Marc J. Sidl). It is computed as follows: 
*   \f[
*           M_{sp} = F(c_{ps} - c_{g})  \\
*           F = \frac{F_s A_s (1+q) cosi}{c}   
*   \f]    
*   where, $M_{sp}$ is the solar radiation pressure disturbance torque, expereinced at 
*   the center of mass of the orbiting body, $c_{ps}$ is the location of the center of 
*   solar pressure, $c_g$ is the location of the center of gravity, $F_s$ is the solar 
*   constant ($1,367 W/m^2$), c is the speed of light, $A_s$ is the spacecraft surface 
*   area projected towards the sun, $q$ is the reflectance factor (rangine from 0-1)
*   and i is the solar incidence angle of the Sun.  
*/


template < typename Real, typename Vector3 >
Vector3 computeSolarRadiationDisturbanceTorque (    const Real      centerOfSolarRadiationPressure, 
                                                    const Real      centerOfGravity, 
                                                    const Real      radiationPressure, 
                                                    const Real      radiationPressureCoefficient,
                                                    const Vector3&  vectorToSource, 
                                                    const Real      area, 
                                                    const Real      mass )

{     
    Vector3 solarRadiationPressureAcceleration  = astro::computeSolarRadiationPressureAcceleration( 
                                                        radiationPressure,
                                                        radiationPressureCoefficient, 
                                                        vectorToSource, 
                                                        area, 
                                                        mass );
    Vector3 solarRadiationPressure; 
    solarRadiationPressure[0]      = solarRadiationPressureAcceleration[0] * mass; 
    solarRadiationPressure[1]      = solarRadiationPressureAcceleration[1] * mass; 
    solarRadiationPressure[2]      = solarRadiationPressureAcceleration[2] * mass; 

    std::cout << solarRadiationPressure[0] << std::endl;    
    std::cout << solarRadiationPressure[1] << std::endl;
    std::cout << solarRadiationPressure[2] << std::endl;
    
    Vector3 solarRadiationPressureTorque;   
    solarRadiationPressureTorque[0] = solarRadiationPressure[0] * (centerOfSolarRadiationPressure - centerOfGravity ); 
    solarRadiationPressureTorque[1] = solarRadiationPressure[1] * (centerOfSolarRadiationPressure - centerOfGravity );
    solarRadiationPressureTorque[2] = solarRadiationPressure[2] * (centerOfSolarRadiationPressure - centerOfGravity ); 
    
    return solarRadiationPressureTorque; 
}

} // namespace astro 
 
 #endif //SOLAR_RADIATION_DISTURBANCE_TORQUE_HPP