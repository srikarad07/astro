/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#ifndef CASCADE_SATURATION_CONTROLLER_HPP
#define CASCADE_SATURATION_CONTROLLER_HPP

#include <math.h>  
#include <numeric> 

#include <Eigen/Dense>

namespace astro 
{

typedef double Real; 
typedef Eigen::Matrix< double, 3, 3 > Matrix33; 
typedef Eigen::Matrix< double, 4, 1 > Vector4;
typedef Eigen::Matrix< double, 3, 1 > Vector3;
typedef Eigen::Matrix< double, Eigen::Dynamic, 1> VectorXd;

inline std::tuple< Matrix33, Matrix33, Matrix33 > computeControlGainsCascadeSaturationLogic( const Real        naturalFrequency,
                                                      const Real        dampingRatio,
                                                      const Real        slewRateSaturation, 
                                                      const Vector3     principleInertiaVector, 
                                                      const Vector4     initialQuaternion )
{   
    //! Make a diagnal matrix out of the principle inertia vector. 
    const Matrix33 principleInertiaMatrix           = principleInertiaVector.asDiagonal(); 

    //! Compute the gain matrices. 
    const Real angularVelocityControlGain           = 2 * naturalFrequency * dampingRatio;
    const Matrix33 angularVelocityControlGainMatrix = angularVelocityControlGain * principleInertiaMatrix;  
    const Matrix33 tempControlGainMatrixQuaternion  = 2* naturalFrequency * naturalFrequency * principleInertiaMatrix; 

    //! Compute the attitude gain matrices. 
    const Vector3 directionalQuaternion( initialQuaternion[0], initialQuaternion[1], initialQuaternion[2] ); 
    const Real directionalQuaternionNorm = directionalQuaternion.norm(); 
    
    //! Calculate the quaternion gain vector as per the formula. 
    Vector3 quaternionControlGainVector; 
    quaternionControlGainVector[0] = ( angularVelocityControlGain * fabs(directionalQuaternion[0]) * slewRateSaturation ) / directionalQuaternionNorm;
    quaternionControlGainVector[1] = ( angularVelocityControlGain * fabs(directionalQuaternion[1]) * slewRateSaturation ) / directionalQuaternionNorm;
    quaternionControlGainVector[2] = ( angularVelocityControlGain * fabs(directionalQuaternion[2]) * slewRateSaturation ) / directionalQuaternionNorm;

    const Matrix33 attitudeControlGainMatrix = quaternionControlGainVector.asDiagonal() * principleInertiaMatrix;

    const Matrix33 attitudeControlGainMatrix2 = attitudeControlGainMatrix.inverse() * tempControlGainMatrixQuaternion; 

    return std::make_tuple( attitudeControlGainMatrix, attitudeControlGainMatrix2, angularVelocityControlGainMatrix ); 
}

inline Vector3 saturationFunction( const Vector3 vectorToBeSaturated, 
                    const Real    upperBound )
                    // const Real    lowerBound = -upperBound )
{
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TO DO >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> // 
    //! Update this function to take multiple upperbounds and be able to default the value to the only given 
    //! upper bound if other are not mentioned (default parameter case!). Also the function should be update 
    //! saturate for any given vector of any size. 
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> //   

    Vector3 vectorAfterSaturation; 
    const Real lowerBound = - upperBound; 
    for ( unsigned int iterator = 0; iterator < vectorToBeSaturated.size(); ++iterator )
    {
        if ( vectorToBeSaturated[iterator] >= upperBound )
        {
            vectorAfterSaturation[iterator] = upperBound;     
        }
        else if ( vectorToBeSaturated.array().abs()[iterator] < upperBound )
        {
            vectorAfterSaturation[iterator] = vectorToBeSaturated[iterator]; 
        }
        else if ( vectorToBeSaturated[iterator] <= lowerBound )
        {
            vectorAfterSaturation[iterator] = lowerBound; 
        }
    }

    return vectorAfterSaturation; 
}

inline Vector3 minimumFunction( const Real      value1, 
                                const Vector3   value2 )
                    // const Real    lowerBound = -upperBound )
{
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TO DO >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> // 
    //! Update this function to take multiple upperbounds and be able to default the value to the only given 
    //! upper bound if other are not mentioned (default parameter case!). Also the function should be update 
    //! saturate for any given vector of any size. 
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> //   

    Vector3 vectorAfterSaturation; 
    // const Vector3 lowerBound = - upperBound; 
    for ( unsigned int iterator = 0; iterator < value2.size(); ++iterator )
    {
        if ( value1 >= value2[iterator] )
        {
            vectorAfterSaturation[iterator] = value2[iterator];     
        }
        else if ( value1 < value2[iterator] )
        {
            vectorAfterSaturation[iterator] = value1; 
        }
        // else if ( saturatedSlewRateSaturation[iterator] <= lowerBound[iterator] )
        // {
        //     vectorAfterSaturation[iterator] = lowerBound[iterator]; 
        // }
    }

    return vectorAfterSaturation; 
}

inline Vector3 normalizedSaturationFunction( const Vector3 vectorToBeSaturated, 
                                             const Vector3 maxVector )
{
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TO DO >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> // 
    //! Update this function to take multiple upperbounds and be able to default the value to the only given 
    //! upper bound if other are not mentioned (default parameter case!). Also the function should be update 
    //! saturate for any given vector of any size. 
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> //   

    const Vector3 tempvector = maxVector.array().inverse(); 
    const Matrix33 maxDiagonalMatrix = tempvector.asDiagonal(); 
    
    // std::cout << maxVector.array().inverse() << std::endl; 

    const Vector3 sigmaFunction      = (maxDiagonalMatrix * vectorToBeSaturated).array().abs();
    Vector3 vectorAfterSaturation; 

    for ( unsigned int iterator = 0; iterator < vectorToBeSaturated.size(); ++iterator )
    {
        if ( sigmaFunction[iterator] <= 1 )
        {   
            vectorAfterSaturation[iterator] = vectorToBeSaturated[iterator]; 
        }
        else if ( sigmaFunction[iterator] > 1)
        {
            vectorAfterSaturation[iterator] = vectorToBeSaturated[iterator] / sigmaFunction[iterator]; 
        }
    }
    // std::cout << "The sigma function: " << sigmaFunction << std::endl;     
    return vectorAfterSaturation; 
}

inline double closest(std::vector<double> const& vec, double value) 
{
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }

    return *it;
}

inline std::vector<double> linspace(double a, double c, double b) 
{
    std::vector<double> array;
    while(a <= c) {
        array.push_back(a);
        a += b;         
    }
    return array;
}

inline Vector3 computeControlTorqueWithSaturationCascadeController( const Real                  naturalFrequency,
                                                                    const Real                  dampingRatio,
                                                                    const Real                  slewRateSaturation, 
                                                                    const Vector3               principleInertiaVector, 
                                                                    const Vector4               initialQuaternion,
                                                                    const Vector4               referenceAttitude,
                                                                    const Vector4               currentAttitude,
                                                                    const Vector3               currentAttitudeRate )
{
    //! Calculate the maximum slew saturation rate. 
    // const Vector3 maximumAcceleration( maximumControlInput[0]/principleInertiaVector[0], maximumControlInput[1]/principleInertiaVector[1], maximumControlInput[2]/principleInertiaVector[2] ); 

    const Vector3 initialDirectionalQuaternion( initialQuaternion[0], initialQuaternion[1], initialQuaternion[2] );

    //! Compute the error in quaternion. 
    Vector3 quaternionError; 
    quaternionError[0]          =   referenceAttitude[3]*currentAttitude[0] + 
                                    referenceAttitude[2]*currentAttitude[1] - 
                                    referenceAttitude[1]*currentAttitude[2] - 
                                    referenceAttitude[0]*currentAttitude[3]; 
    quaternionError[1]          =  -referenceAttitude[2]*currentAttitude[0] + 
                                    referenceAttitude[3]*currentAttitude[1] + 
                                    referenceAttitude[0]*currentAttitude[2] - 
                                    referenceAttitude[1]*currentAttitude[3];
    quaternionError[2]          =   referenceAttitude[1]*currentAttitude[0] - 
                                    referenceAttitude[0]*currentAttitude[1] + 
                                    referenceAttitude[3]*currentAttitude[2] - 
                                    referenceAttitude[2]*currentAttitude[3];  

    // Vector3 slewRateSaturationLimiter; 

    // slewRateSaturationLimiter[0] = sqrt( fabs( 0.4 * maximumAcceleration[0] ) ); 
    // slewRateSaturationLimiter[1] = sqrt( fabs( 0.4 * maximumAcceleration[1] ) ); 
    // slewRateSaturationLimiter[2] = sqrt( fabs( 0.4 * maximumAcceleration[2] ) ); 

    // const Vector3 saturatedSlewRateSaturation = minimumFunction( slewRateSaturation, slewRateSaturationLimiter ); 

    //! Compute the control gain matrices. 
    std::tuple< Matrix33, Matrix33, Matrix33 > controlGainMatrices 
         = computeControlGainsCascadeSaturationLogic(   naturalFrequency,
                                                        dampingRatio,
                                                        slewRateSaturation, 
                                                        principleInertiaVector, 
                                                        initialQuaternion ); 

    const Matrix33 attitudeControlGainMatrix    = std::get<0>(controlGainMatrices); 
    const Matrix33 attitudeControlGainMatrix2   = std::get<1>(controlGainMatrices);
    const Matrix33 velocityControlGainMatrix    = std::get<2>(controlGainMatrices);
    const Matrix33 principleInertiaMatrix       = principleInertiaVector.asDiagonal(); 
    
    const Real k = 2 * naturalFrequency * naturalFrequency; 
    const Real c = 2 * naturalFrequency * dampingRatio; 
    
    const Vector3 commandedControlTorque = - ( attitudeControlGainMatrix * saturationFunction( attitudeControlGainMatrix2 * quaternionError, 1.0 ) + velocityControlGainMatrix * currentAttitudeRate );  
    
    return commandedControlTorque; 
}

// inline Vector3 computeControlTorqueWithSaturationCascadeControllerWithLimiter( 
//                                                                     const Real                  naturalFrequency,
//                                                                     const Real                  dampingRatio,
//                                                                     const Real                  slewRateSaturation, 
//                                                                     const Vector3               principleInertiaVector, 
//                                                                     const Vector4               initialQuaternion,
//                                                                     const Vector4               referenceAttitude,
//                                                                     const Vector4               currentAttitude,
//                                                                     const Vector3               currentAttitudeRate, 
//                                                                     const Vector3               maximumControlInput )
// {
//     //! Calculate the maximum slew saturation rate. 
//     const Vector3 maximumAcceleration( maximumControlInput[0]/principleInertiaVector[0], maximumControlInput[1]/principleInertiaVector[1], maximumControlInput[2]/principleInertiaVector[2] ); 

//     const Vector3 initialDirectionalQuaternion( initialQuaternion[0], initialQuaternion[1], initialQuaternion[2] );

//     //! Compute the error in quaternion. 
//     Vector3 quaternionError; 
//     quaternionError[0]          =   referenceAttitude[3]*currentAttitude[0] + 
//                                     referenceAttitude[2]*currentAttitude[1] - 
//                                     referenceAttitude[1]*currentAttitude[2] - 
//                                     referenceAttitude[0]*currentAttitude[3]; 
//     quaternionError[1]          =  -referenceAttitude[2]*currentAttitude[0] + 
//                                     referenceAttitude[3]*currentAttitude[1] + 
//                                     referenceAttitude[0]*currentAttitude[2] - 
//                                     referenceAttitude[1]*currentAttitude[3];
//     quaternionError[2]          =   referenceAttitude[1]*currentAttitude[0] - 
//                                     referenceAttitude[0]*currentAttitude[1] + 
//                                     referenceAttitude[3]*currentAttitude[2] - 
//                                     referenceAttitude[2]*currentAttitude[3];  

//     Vector3 slewRateSaturationLimiter; 

//     slewRateSaturationLimiter[0] = sqrt( fabs( 0.4 * maximumAcceleration[0] ) ); 
//     slewRateSaturationLimiter[1] = sqrt( fabs( 0.4 * maximumAcceleration[1] ) ); 
//     slewRateSaturationLimiter[2] = sqrt( fabs( 0.4 * maximumAcceleration[2] ) ); 

//     const Vector3 saturatedSlewRateSaturation = minimumFunction( slewRateSaturation, slewRateSaturationLimiter ); 

//     //! Compute the control gain matrices. 
//     std::tuple< Matrix33, Matrix33, Matrix33 > controlGainMatrices 
//          = computeControlGainsCascadeSaturationLogic(   naturalFrequency,
//                                                         dampingRatio,
//                                                         saturatedSlewRateSaturation, 
//                                                         principleInertiaVector, 
//                                                         initialQuaternion ); 

//     const Matrix33 attitudeControlGainMatrix    = std::get<0>(controlGainMatrices); 
//     const Matrix33 attitudeControlGainMatrix2   = std::get<1>(controlGainMatrices);
//     const Matrix33 velocityControlGainMatrix    = std::get<2>(controlGainMatrices);
//     const Matrix33 principleInertiaMatrix       = principleInertiaVector.asDiagonal(); 
    
//     const Real k = 2 * naturalFrequency * naturalFrequency; 
//     const Real c = 2 * naturalFrequency * dampingRatio; 
    
//     const Vector3 commandedControlTorque = - ( attitudeControlGainMatrix * saturationFunction( attitudeControlGainMatrix2 * quaternionError, 1.0 ) + velocityControlGainMatrix * currentAttitudeRate );  
    
//     return commandedControlTorque; 
// }

} // namespace 

#endif // CASCADE_SATURATION_CONTROLLER_HPP