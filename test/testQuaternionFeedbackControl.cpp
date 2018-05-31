/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#include <catch.hpp>

#include "astro/quaternionFeedbackControl.hpp"

namespace astro
{
namespace tests
{
typedef double Real;  
typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 

TEST_CASE( "Test Case 1: Test the quaternion feedback control for an ephemeris of reference quaternions" , "[quaternionReference, quaternionCurrent, angularVelocity ]")
{

    // const Vector4 quaternionCurrent; 
    // quaternionCurrent( 10.0, 10.0, 10.0, 1.0 );
    const Real tolerance = 1.0e-12;  

    const Vector4 quaternionCurrent( 10.0, 10.0, 10.0, 1.0 ); 

    const Vector4 quaternionReference( 0.0, 0.0, 0.0, 1.0 );

    const Vector3 angularVelocity( 0.1, 0.1, 0.1 );

    const Real quaternionControlGain( 10.0 );

    const Vector3 angularVelocityControlGain( 10.0, 10.0, 10.0 );

    Vector3 computedControlTorque   = astro::computeQuaternionControlTorque(    quaternionReference, 
                                                                                quaternionCurrent,
                                                                                angularVelocity,
                                                                                quaternionControlGain,
                                                                                angularVelocityControlGain );

    // Vector3 computedControlTorque( 9.0, 9.0, 9.0 );  
    // <<TO DO; Find a better way to get the value of expectedControlTorque. >> // 
    Vector3 expectedControlTorque; 
    expectedControlTorque[0]        = -101.0;
    expectedControlTorque[1]        = -101.0;
    expectedControlTorque[2]        = -101.0;

    // Check if computed value is equal to the expected value. 
    REQUIRE( computedControlTorque[0] == Approx( expectedControlTorque[0]).epsilon(tolerance) );             
    REQUIRE( computedControlTorque[1] == Approx( expectedControlTorque[1]).epsilon(tolerance) );             
    REQUIRE( computedControlTorque[2] == Approx( expectedControlTorque[2]).epsilon(tolerance) );                                                                              
} // testcase-1. 

} // tests  

} // astro 