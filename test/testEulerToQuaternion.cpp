/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
*/

#include <catch.hpp>

#include <Eigen/Eigen>

#include <sml/sml.hpp>
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

TEST_CASE( "Test Case 2: Test the euler angle to quaternion function from Eigen." , "[quaternionState]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( sml::convertDegreesToRadians( 30.0 ), sml::convertDegreesToRadians(30.0), sml::convertDegreesToRadians(30.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion( 1.767766952966369e-01, 3.061862178478972e-01 ,  1.767766952966369e-01, 9.185586535436919e-01  );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

TEST_CASE( "Test 3: Test the euler angle to quaternion function from Eigen." , "[40.0, 40.0, 40.0]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( sml::convertDegreesToRadians( 40.0 ), sml::convertDegreesToRadians(40.0), sml::convertDegreesToRadians(40.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion(  1.920882315810498e-01, 4.119345419740040e-01, 1.920882315810498e-01, 8.697782221375732e-01    );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

TEST_CASE( "Test 4: Test the euler angle to quaternion function from Eigen." , "[20.0, 33.0, 65.0]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( sml::convertDegreesToRadians(20.0), sml::convertDegreesToRadians(33.0), sml::convertDegreesToRadians(65.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion( 4.657518160827175e-01, 3.253559543278105e-01, -9.860578031679113e-03, 8.228739379759037e-01 );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

TEST_CASE( "Test 5: Test the euler angle to quaternion function from Eigen." , "[35, 65, 55]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( sml::convertDegreesToRadians(35.0), sml::convertDegreesToRadians(65.0), sml::convertDegreesToRadians(55.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion( 2.280970497069847e-01, 5.716378203564888e-01, -1.165743576888184e-02, 7.880774342945918e-01 );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

TEST_CASE( "Test 6: Test the euler angle to quaternion function from Eigen." , "[120, 200, 20]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-12; 

    const Vector3 euler( sml::convertDegreesToRadians(120.0), sml::convertDegreesToRadians(200.0), sml::convertDegreesToRadians(20.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion( -8.549883873704290e-01, 4.588092939789447e-01, -2.336041021944291e-01, 6.259403053159474e-02 );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

TEST_CASE( "Test 7: Test the euler angle to quaternion function from Eigen." , "[5.0, 2.0, 300.0]" )
{
    /* The values for the expected quaternion are calculated with the use of MATLAB function eul2quat
    *  with the euler rotation sequence of '321'/'ZYX' 
    */
    const Real tolerance = 1.0e-15; 

    const Vector3 euler( sml::convertDegreesToRadians(5.0), sml::convertDegreesToRadians(2.0), sml::convertDegreesToRadians(300.0) ); 

    // computedEulerAngles; 
    const Vector4 computedQuaternion = transformEulerToQuaternion( euler );  
     
    // Expected values 
    const Vector4 expectedQuaternion( 5.001073041860721e-01, 6.706530024205272e-03, -4.648764197139339e-02, -8.646887335328902e-01 );
    
    // Check if expected values are equal to the computed values. 
    REQUIRE( computedQuaternion[0] == Approx( expectedQuaternion[0]).epsilon(tolerance) ); 

    REQUIRE( computedQuaternion[1] == Approx( expectedQuaternion[1]).epsilon(tolerance) ); 
    
    REQUIRE( computedQuaternion[2] == Approx( expectedQuaternion[2]).epsilon(tolerance) );  
    REQUIRE( computedQuaternion[3] == Approx( expectedQuaternion[3]).epsilon(tolerance) ); 
}

} // tests

} // astro 