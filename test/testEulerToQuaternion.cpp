/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#include <catch.hpp>

#include <Eigen/Eigen>

#include <astro/eulerToQuaternion.hpp>

namespace astro 
{
namespace tests
{    

typedef double Real;  
typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 
typedef Eigen::Quaterniond Quaternion; 

TEST_CASE( "Test Case 1: Test the euler angle to quaternion function from Eigen." , "[quaternionState]" )
{
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( 0.0, 0.0, 0.0 ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    const Vector4 expectedQuaternion( 0.0, 0.0, 0.0, 1.0 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
} // test case 1


} // tests

} // astro 