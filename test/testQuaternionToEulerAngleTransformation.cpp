/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#include <catch.hpp>

#include <Eigen/Eigen>

#include <astro/quaternionToEulerAngleTransformation.hpp>

namespace astro 
{
namespace tests
{    
typedef double Real;  
typedef Eigen::Matrix< double, 4, 1 > Vector4; 
typedef Eigen::Matrix< double, 3, 1 > Vector3; 

TEST_CASE( "Test Case 1: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{

    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 0.0, 0.0, 0.0, 1.0 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    const Vector3 expectedEulerAngles( 0.0, 0.0, 0.0 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 1

TEST_CASE( "Test Case 2: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{

    const Real tolerance = 1.0e-5; 

    const Vector4 quaternion( 0.0905287, 0.0356992, 0.0905287, 0.991128 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 10 deg, 5 deg, 10 deg in radians.  
    const Vector3 expectedEulerAngles(  1.745329251994329e-01,  8.726646259971647e-02,  1.745329251994329e-01 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 2

} // tests

} // astro 

