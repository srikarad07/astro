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
    // The MATLAB function is used to compute the values of the expected euler angles. 
    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 1.767766952966369e-01, 3.061862178478972e-01, 1.767766952966369e-01, 9.185586535436919e-01 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 30 deg, 30 deg, 30 deg in radians.  
    const Vector3 expectedEulerAngles( 5.235987755982987e-01, 5.235987755982989e-01, 5.235987755982988e-01 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 2


TEST_CASE( "Test Case 3: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{
    // The MATLAB function is used to compute the values of the expected euler angles. 
    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 1.920882315810498e-01, 4.119345419740040e-01, 1.920882315810498e-01, 8.697782221375732e-01 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 40 deg, 40 deg, 40 deg in radians.  
    const Vector3 expectedEulerAngles( 6.981317007977320e-01, 6.981317007977319e-01, 6.981317007977320e-01 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 3

TEST_CASE( "Test Case 4: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{
    // The MATLAB function is used to compute the values of the expected euler angles. 
    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 4.657518160827175e-01, 3.253559543278105e-01, -9.860578031679113e-03, 8.228739379759037e-01 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 20.0, 33.0, 65.0 in radians.  
    const Vector3 expectedEulerAngles( 3.490658503988659e-01, 5.759586531581288e-01, 1.134464013796314e+00 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 4

TEST_CASE( "Test Case 5: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{
    // The MATLAB function is used to compute the values of the expected euler angles. 
    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 2.280970497069847e-01, 5.716378203564888e-01, -1.165743576888184e-02, 7.880774342945918e-01 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 20.0, 33.0, 65.0 in radians.  
    const Vector3 expectedEulerAngles( 6.108652381980152e-01, 1.134464013796315e+00, 9.599310885968811e-01 ); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 5

TEST_CASE( "Test Case 6: Test the quaternion to euler angle function from Eigen." , "[quaternionState]" )
{
    // The MATLAB function is used to compute the values of the expected euler angles. 
    const Real tolerance = 1.0e-12; 

    const Vector4 quaternion( 2.258941546823033e-01, 7.239915037281760e-01, -1.387384119346450e-01, 6.368357609805178e-01 ); 

    // computedEulerAngles; 
    const Vector3 computedEulerAngles = transformQuaternionToEulerAngles( quaternion );  
     
    // Euler angles 120, 80, 150 in radians.  
    const Vector3 expectedEulerAngles( 2.094395102393195e+00, 1.396263401595465e+00, 2.617993877991494e+00); 

    // Check if expected values are equal to the computed values. 
    REQUIRE( computedEulerAngles[0] == Approx( expectedEulerAngles[0]).epsilon(tolerance) ); 

    REQUIRE( computedEulerAngles[1] == Approx( expectedEulerAngles[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedEulerAngles[2] == Approx( expectedEulerAngles[2]).epsilon(tolerance) );  

} // test case 6

} // tests

} // astro 

