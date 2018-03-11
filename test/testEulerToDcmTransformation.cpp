/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <catch.hpp>
#include <Eigen/Eigen>

#include <sml/sml.hpp>

#include "astro/eulerAngleToDcmTransformation.hpp"
#include "astro/loadCsvFile.hpp"

namespace astro
{
namespace tests
{

typedef double Real; 
typedef Eigen::Matrix< double, 3, 1 > Vector3;
typedef Eigen::Matrix< double, 3, 3 > Matrix33; 

TEST_CASE( "Test Case 1: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-12;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(45.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(60.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(45.0);

    Matrix33 expectedDirectionCosineMatrix; 
    expectedDirectionCosineMatrix <<    0.353553390593274, 0.353553390593274, -0.866025403784439, 
                                        -0.066987298107781, 0.933012701892219, 0.353553390593274, 
                                        0.933012701892219, -0.066987298107781, 0.353553390593274; 

    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            1, 0, 0;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}    

TEST_CASE( "Test Case 2: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-5;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(30.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(45.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(60.0);

    Matrix33 expectedDirectionCosineMatrix; 
    std::string testPath = "/home/superman/workspace/test/30_45_60_321_dcm_transformation.dat";  
    expectedDirectionCosineMatrix = astro::load_csv<Matrix33>(testPath);
    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            1, 0, 0;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}

TEST_CASE( "Test Case 3: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-5;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(60.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(30.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(45.0);

    Matrix33 expectedDirectionCosineMatrix; 
    std::string testPath = "/home/superman/workspace/test/60_30_45_321_dcm_transformation.dat";  
    expectedDirectionCosineMatrix = astro::load_csv<Matrix33>(testPath);
    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            1, 0, 0;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}

TEST_CASE( "Test Case 4: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-5;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(60.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(30.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(45.0);

    Matrix33 expectedDirectionCosineMatrix; 
    std::string testPath = "/home/superman/workspace/test/60_30_45_123_dcm_transformation.dat";  
    expectedDirectionCosineMatrix = astro::load_csv<Matrix33>(testPath);
    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     1, 0, 0, 
                            0, 1, 0, 
                            0, 0, 1;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}

TEST_CASE( "Test Case 5: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-4;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(60.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(30.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(45.0);

    Matrix33 expectedDirectionCosineMatrix; 
    std::string testPath = "/home/superman/workspace/test/60_30_45_323_dcm_transformation.dat";  
    expectedDirectionCosineMatrix = astro::load_csv<Matrix33>(testPath);
    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            0, 0, 1;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}

TEST_CASE( "Test Case 6: Transform from euler angle representation to direction cosine matrix represention", "[rotation_sequence, euler_angles]" )
{
    // Benchmark values for test case obtained using the angle2dcm() function in MATLAB. 

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-4;
    
    // Testing the function for values: 
    Vector3 eulerAngles; 
    eulerAngles[0]      = sml::convertDegreesToRadians(60.0); 
    eulerAngles[1]      = sml::convertDegreesToRadians(30.0);
    eulerAngles[2]      = sml::convertDegreesToRadians(45.0);

    Matrix33 expectedDirectionCosineMatrix; 
    std::string testPath = "/home/superman/workspace/test/60_30_45_121_dcm_transformation.dat";  
    expectedDirectionCosineMatrix = astro::load_csv<Matrix33>(testPath);
    Matrix33 computedDirectionCosineMatrix, rotationSequence; 
    rotationSequence <<     1, 0, 0, 
                            0, 1, 0, 
                            1, 0, 0;  

    computedDirectionCosineMatrix = astro::computeEulerAngleToDcmConversionMatrix( rotationSequence,
                                                                                   eulerAngles ); 

    // Check if computed matrix is the same as actual DCM. 
    REQUIRE( computedDirectionCosineMatrix(0,0) 
                    == Approx( expectedDirectionCosineMatrix(0,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,1) 
                    == Approx( expectedDirectionCosineMatrix(0,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(0,2) 
                    == Approx( expectedDirectionCosineMatrix(0,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,0) 
                    == Approx( expectedDirectionCosineMatrix(1,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,1) 
                    == Approx( expectedDirectionCosineMatrix(1,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(1,2) 
                    == Approx( expectedDirectionCosineMatrix(1,2) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,0) 
                    == Approx( expectedDirectionCosineMatrix(2,0) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,1) 
                    == Approx( expectedDirectionCosineMatrix(2,1) ).epsilon(tolerance) );
    REQUIRE( computedDirectionCosineMatrix(2,2) 
                    == Approx( expectedDirectionCosineMatrix(2,2) ).epsilon(tolerance) );
}

} // namespace tests

} // namespace astro 