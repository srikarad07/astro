/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */
#include <catch.hpp>

#include "astro/quaternionDifferential.hpp"

namespace astro
{
namespace tests
{

typedef double Real;  
typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 

TEST_CASE( "Test case 1: Test the quternion differential equations for a set of quaternions", "[quaternions, angularVelocity]" )
{
const Real tolerance = 1.0e-12;  

Vector4 quaternion; 
quaternion << 10.0, 10.0, 10.0, 10.0; 

Vector3 angularVelocity; 
angularVelocity << 0.0, 0.0, 0.0; 

Vector4 expectedQuaternionDerivative, computedQuaternionDerivative; 

expectedQuaternionDerivative << 0.0, 0.0, 0.0, 0.0; 

computedQuaternionDerivative    = astro::computeQuaternionDerivative( quaternion, angularVelocity ); 

// Check if computed value is equal to the expected value. 
REQUIRE( computedQuaternionDerivative[0] 
                                    == Approx(expectedQuaternionDerivative[0]).epsilon(tolerance) ); 
REQUIRE( computedQuaternionDerivative[1] 
                                    == Approx(expectedQuaternionDerivative[1]).epsilon(tolerance) );
REQUIRE( computedQuaternionDerivative[2] 
                                    == Approx(expectedQuaternionDerivative[2]).epsilon(tolerance) );
}

} // tests  

} // astro 