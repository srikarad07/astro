/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef EULER_ANGLE_TO_DCM_TRANSFORMATION
#define EULER_ANGLE_TO_DCM_TRANSFORMATION

/*  Direction Cosine Matrices (DCM's) are universal translators for attitude representations. 
*   In this class any sequence of euler angles will be converted into DCM's using the inbuilt
*   in eigen library for rotation transformations. 
*/

#include <Eigen/Eigen>

namespace astro
{
    
//  Transformation of euler angle representation to the DCM representation for attitude. 
/*  Computes the direction cosine matrix for any sequence of euler angle rotation. 
*   The expression is defined for a three euler angle rotation (yaw, pitch, roll) and the rotation
*   matrix is given as: 
*   \f[
*        DCM = C(\theta_k)*C(\theta_j)*C(\theta_i)
*   \f]
*   where $DCM$ is the rotation matrix for a sequential rotation of (i -> j -> k), $i, j, k$ are the 
*   axes of rotation and C(\theta) is the rotation matrix for a rotation of \theta around a particular 
*   axis.
*   
*   @tparam         Vector3                             3 vector type            
*   @tparam         Matrix33                            3x3 matrix type
*   @param[in]      rotationSequence                    Matrix33 type. Represents the axis of rotation
*                                                       in vector type. For ex: For a 3 -> 2 -> 1 
*                                                       rotation the rotational sequence will be:
*                                                       [  0, 0, 1 
*                                                          0, 1, 0 
*                                                          1, 0, 0 ] 
*   @param[in]      eulerAngles                         Vector3 type. Rotational euler angles in the 
*                                                       same sequence as above. [rad] 
*/

template < typename Vector3, typename Matrix33 >
Matrix33 computeEulerAngleToDcmConversionMatrix( Matrix33 rotationSequence,
                                                 Vector3 eulerAngles )
{
    Vector3 rotationAxis1( rotationSequence.row(0) );              
    Vector3 rotationAxis2( rotationSequence.row(1) ); 
    Vector3 rotationAxis3( rotationSequence.row(2) );

    Matrix33 directionCosineMatrix;
    directionCosineMatrix       =   Eigen::AngleAxisd( -eulerAngles[2], rotationAxis3 )*
                                    Eigen::AngleAxisd( -eulerAngles[1], rotationAxis2 )*
                                    Eigen::AngleAxisd( -eulerAngles[0], rotationAxis1 );
    return directionCosineMatrix; 
}
     
}

#endif // EULER_ANGLE_TO_DCM_TRANSFORMATION