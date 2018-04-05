/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

 #ifndef QUATERNION_TO_EULER_ANGLE_TRANSFORMATION_HPP
 #define QUATERNION_TO_EULER_ANGLE_TRANSFORMATION_HPP

#include <Eigen/Eigen>

namespace astro
{

template < typename Vector3, typename Vector4 > 
Vector3 transformQuaternionToEulerAngles( Vector4 quaternions )
{
    auto euler = quaternions.toRotationMatrix().eulerAngles(0, 1, 2);
}

} // astro 
 #endif 