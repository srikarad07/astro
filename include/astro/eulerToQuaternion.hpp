/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef ASTRO_EULER_TO_QUATERNION_HPP
#define ASTRO_EULER_TO_QUATERNION_HPP

#include <iostream>
#include <math.h>
#include <stdexcept>

#include <Eigen/Eigen>

namespace astro
{

typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 
typedef Eigen::Quaterniond Quaternion; 

inline Vector4 transformEulerToQuaternion( const Vector3 euler )
{
    Quaternion tempQuaternion;
    tempQuaternion  =     Eigen::AngleAxisd( euler[0], Eigen::Vector3d::UnitX() )
                        * Eigen::AngleAxisd( euler[1], Eigen::Vector3d::UnitY() )
                        * Eigen::AngleAxisd( euler[2], Eigen::Vector3d::UnitZ() );
    
    const Vector4 quaternion( tempQuaternion.coeffs() ); 
    
    return quaternion; 
}

} // namespace astro 

#endif 