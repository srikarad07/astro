/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef ASTRO_QUATERNION_TO_EULER_ANGLE_TRANSFORMATION_HPP
#define ASTRO_QUATERNION_TO_EULER_ANGLE_TRANSFORMATION_HPP

#include <iostream>
#include <stdexcept>

#include <Eigen/Eigen>

namespace astro
{

// typedef double Real; 

typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 

inline Vector3 transformQuaternionToEulerAngles( const Vector4 quaternion )
{
    /*  The euler angles will be converted only in the range [-180, 180] for the roll and yaw (1st and 3rd)
    *   angles and 80 deg for pitch (2nd) angle. This is the sequence where the euler angles acheive a 
    *   singularity and hence the conversion sequence fails! 
    */
    // <<<<<<<<<<<<<<<<<<<<< TO DO >>>>>>>>>>>>>>>>>>>>>>>>>>> // 
    // Convert to euler angles without range obligations. Use a different 
    // conversion technique during similarities or something? 
    // <<<<<<<<<<<<<<<<<<<<<       >>>>>>>>>>>>>>>>>>>>>>>>>>> //
     
    Eigen::Quaterniond quaternionNew(   quaternion[3], 
                                        quaternion[0], 
                                        quaternion[1], 
                                        quaternion[2] );

    // Use 321 rotation for the euler angle sequence.    
    const Vector3 euler = quaternionNew.toRotationMatrix().eulerAngles(2, 1, 0);

    return euler;
}

} // astro 

#endif 