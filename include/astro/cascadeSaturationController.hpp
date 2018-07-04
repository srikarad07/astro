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
                                                      const Vector4   initialQuaternion )
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
    const Vector3 quaternionControlGainVector = angularVelocityControlGain * directionalQuaternion.array().abs() * slewRateSaturation / directionalQuaternionNorm;

    const Matrix33 attitudeControlGainMatrix = quaternionControlGainVector.asDiagonal() * principleInertiaMatrix;

    const Matrix33 attitudeControlGainMatrix2 = attitudeControlGainMatrix.inverse() * tempControlGainMatrixQuaternion; 

    return std::make_tuple( attitudeControlGainMatrix, attitudeControlGainMatrix2, angularVelocityControlGainMatrix );
    // std::cout << "Damping ration and angular frequency: " << naturalFrequency << dampingRatio << std::endl; 
    // std::cout << "C matrix: " << angularVelocityControlGainMatrix << std::endl;
    // std::cout << "KP matrix: " << tempControlGainMatrixQuaternion << std::endl; 
    // std::cout << "K matrix: " << attitudeControlGainMatrix << std::endl; 
    // std::cout << "P matrix: " << attitudeControlGainMatrix2 << std::endl; 
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
                                                                    const Vector3               currentAttitudeRate, 
                                                                    const Real                  simulationTime )
{
    //! Compute the control gain matrices. 
    std::tuple< Matrix33, Matrix33, Matrix33 > controlGainMatrices 
         = computeControlGainsCascadeSaturationLogic(   naturalFrequency,
                                                        dampingRatio,
                                                        slewRateSaturation, 
                                                        principleInertiaVector, 
                                                        initialQuaternion ); 

    // const Matrix33 attitudeControlGainMatrix    = std::get<0>(controlGainMatrices); 
    const Matrix33 attitudeControlGainMatrix2   = std::get<1>(controlGainMatrices);
    // const Matrix33 velocityControlGainMatrix    = std::get<2>(controlGainMatrices);
    const Matrix33 principleInertiaMatrix           = principleInertiaVector.asDiagonal(); 
    
    const Real k = naturalFrequency * naturalFrequency; 
    const Real c = 2 * naturalFrequency * dampingRatio; 
    
    const Matrix33 attitudeControlGainMatrix    =  k * principleInertiaMatrix; 
    // const Matrix33 attitudeControlGainMatrix2   =  ;
    const Matrix33 velocityControlGainMatrix    =  c * principleInertiaMatrix;
    // std::cout << "K: " << attitudeControlGainMatrix  << std::endl; 
    // std::cout << "P: " << attitudeControlGainMatrix2 << std::endl; 
    // std::cout << "C: " << velocityControlGainMatrix << std::endl; 

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
    // quaternionError[3]          =   referenceAttitude[0]*currentAttitude[0] + 
    //                                 referenceAttitude[1]*currentAttitude[1] + 
    //                                 referenceAttitude[2]*currentAttitude[2] + 
    //                                 referenceAttitude[3]*currentAttitude[3];
    // const Real endOfAccelerationPhaseTimeCalculated = 4 / angularVelocityControlGain;  
    
    // std::vector<Real> timeRange = linspace( initialTime, finalTime, timeStep ); 
    // const Real endOfAccelerationPhaseTime = closest( timeRange, endOfAccelerationPhaseTimeCalculated ); 
    
    // std::cout << 
    // std::cout << "The start time and end time is: " << timeRange[0] << " and " << timeRange[1] << std::endl;
    // std::cout << "The number of times are: " << timeRange.size() << std::endl; 
    // std::cout << "End of acceleration phase: " << endOfAccelerationPhaseTime << std::endl; 
    // std::cout << "Simulation time: " << simulationTime << std::endl; 

    // Vector4 quaternionAtEndOFAcceleration( 0.0, 0.0, 0.0, 0.0 ); 

    // std::cout << "Quaternion error: " << quaternionError << std::endl; 
    // std::cout << "P * q_e " << attitudeControlGainMatrix2 * quaternionError << std::endl; 
    // std::cout << "Saturation: " << saturationFunction( attitudeControlGainMatrix2 * quaternionError, slewRateSaturation ) << std::endl; 
    // Real endOfCoastingPhaseTime; 
    const Real L = ( c / k ) * 0.2;
    // std::cout << "L matrix: " << L << std::endl; 
    // std::cout << "Saturation function: " << saturationFunction( attitudeControlGainMatrix * quaternionError, L ) << std::endl; 
    // std::cout << "Control first term: " << attitudeControlGainMatrix * saturationFunction( attitudeControlGainMatrix * quaternionError, L )  << std::endl; 
    // std::cout << "Control second term: " << velocityControlGainMatrix * currentAttitudeRate << std::endl; 
    // std::cout << "Control first + second: " << ( attitudeControlGainMatrix * saturationFunction( quaternionError, L ) ) + ( velocityControlGainMatrix * currentAttitudeRate ) << std::endl; 
    // const Vector3 commandedControlTorque = - ( attitudeControlGainMatrix * saturationFunction( quaternionError, L ) + velocityControlGainMatrix * currentAttitudeRate );  
    const Vector3 commandedControlTorque = - ( attitudeControlGainMatrix * saturationFunction( quaternionError, L ) + velocityControlGainMatrix * currentAttitudeRate ); 

    // std::cout << "Commanded control torque: " << commandedControlTorque << std::endl; 
    //! Compute the control torque.
    // if ( simulationTime <= endOfAccelerationPhaseTime )
    // {
    //     commandedControlTorque = attitudeControlGainMatrix * saturationFunction( attitudeControlGainMatrix2 * quaternionError, slewRateSaturation ) + velocityControlGainMatrix * currentAttitudeRate;   
    //     if ( simulationTime == endOfAccelerationPhaseTime )
    //     {
    //         quaternionAtEndOFAcceleration = currentAttitude; 
    //         endOfCoastingPhaseTime = endOfAccelerationPhaseTime + 2 / slewRateSaturation * atan( initialDirectionalQuaternion.norm() / quaternionAtEndOFAcceleration[3] ); 
    //     }
    // }
    // else
    // {
    //     if ( simulationTime <= endOfAccelerationPhaseTime )
    //     {
    //         commandedControlTorque = {0.0, 0.0, 0.0}; 
    //     }
    //     else 
    //     {
    //         const Real k = 2 * naturalFrequency * naturalFrequency; 
    //         const Real c = 2 * naturalFrequency * dampingRatio; 
    //         // commandedControlTorque = astro::computeQuaternionControlTorque( ); 
    //         commandedControlTorque = -k*quaternionError - c*currentAttitudeRate;
    //     }
    //     // const Real endOfCoastingPhaseTime = endOfAccelerationPhaseTime + 2 / slewRateSaturation * atan( ); 
    // }
    
    // const Vector3 controlTorque = - normalizedSaturationFunction( commandedControlTorque, controlConstraint ); 
    // std::cout << "Control torque value " << controlTorque << " at simulation time " << simulationTime << std::endl; 
    
    // const Vector3 controlTorque( 0.0, 0.0, 0.0);
    // return controlTorque; 
    return commandedControlTorque; 
}

} // namespace 

#endif // CASCADE_SATURATION_CONTROLLER_HPP