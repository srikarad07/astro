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
    Eigen::Quaterniond quaternionNew(   quaternion[3], 
                                        quaternion[0], 
                                        quaternion[1], 
                                        quaternion[2] );
    
    const Vector3 euler = quaternionNew.toRotationMatrix().eulerAngles(0, 1, 2);

    return euler;
}

} // astro 

#endif 