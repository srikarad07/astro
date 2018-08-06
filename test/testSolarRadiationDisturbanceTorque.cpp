/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <catch.hpp>

#include "astro/solarRadiationDisturbanceTorqueModel.hpp"

namespace astro
{
namespace tests 
{

typedef double Real; 
typedef Eigen::Matrix< double, 3, 1 > Vector3;

TEST_CASE("Test 1: Compute solar radiation pressure torque for an arbitrary case", "[solar_radiation_pressure, acceleration, models]" )
{

    // Set expected solar radiation pressure torque vector [m/s^2]
    Vector3 expectedTorque; 
    expectedTorque[ 0 ]  = 0.0; 
    expectedTorque[ 1 ]  = 0.0;
    expectedTorque[ 2 ]  = 0.0;

    // Set the center of solar pressure. 
    const Real centerOfSolarPressure = 10.00;

    // Set the center of gravity. 
    const Real centerOfGravity = 10.0;  

    // Set 1 AU in metres [m].
    const double astronomicalUnitInMeters = 1.49598e11;

    // Set tolerance = error between expected value and computed value.
    const Real tolerance = 1.0e-15;

    // Solar Radiation Pressure at 1 AU [N m^-2].
    const Real solarRadiationPressure = 4.56e-6;

    // Set radiation pressure coefficient.
    const Real radiationPressureCoefficient = 1.0 + 0.3;

    // Set absorbing area [m^2].
    const Real area = 2.0;

    // Set mass [kg].
    const Real mass = 4.0;

    // Set unit vector pointing from S/C to the Sun.
    Vector3 vectorToSource;
    vectorToSource[ 0 ] = astronomicalUnitInMeters;
    vectorToSource[ 1 ] = 0.0;
    vectorToSource[ 2 ] = 0.0;

    // Compute the unit vector to the Sun.
    const Real normVectorToSource = std::sqrt( vectorToSource[ 0 ] * vectorToSource[ 0 ]
                                               + vectorToSource[ 1 ] * vectorToSource[ 1 ]
                                               + vectorToSource[ 2 ] * vectorToSource[ 2 ] );

    const Real squaredNormVectorToSource = normVectorToSource * normVectorToSource;
    
    Vector3 unitVectorToSource;
    unitVectorToSource[ 0 ] = vectorToSource[ 0 ] / normVectorToSource;
    unitVectorToSource[ 1 ] = vectorToSource[ 1 ] / normVectorToSource;
    unitVectorToSource[ 2 ] = vectorToSource[ 2 ] / normVectorToSource;

    // Compute the solar radiation pressure torque. 
    const Vector3 computedTorque = computeSolarRadiationDisturbanceTorque( centerOfSolarPressure, 
                                            centerOfGravity, 
                                            solarRadiationPressure, 
                                            radiationPressureCoefficient, 
                                            unitVectorToSource, 
                                            area, 
                                            mass );

    // Check if the computed values match the expected values. 
    REQUIRE(  computedTorque[0]
                        == Approx( expectedTorque[0] ).epsilon( tolerance ) ); 
    REQUIRE(  computedTorque[1]
                        == Approx( expectedTorque[1] ).epsilon( tolerance ) ); 
    REQUIRE(  computedTorque[2]
                        == Approx( expectedTorque[2] ).epsilon( tolerance ) ); 

}

TEST_CASE( "Test 2: Test the solar radiation torque model with Firesat (SMAD).", "[solar_radiation_pressure, acceleration, models]")
{
    /*  The function solar radiation disturbance torque is verified by using the values presented for 
    *   firesat & SCS spacecraft given in Space Mission Engineering - The New SMAD by James Wertz,  
    *   David Everett & Jeffery Puschell, Space Technology Library, Vol. 28, Microsom Press 2011 
    */
    
    // Set the tolerance for the tests. 
    const Real tolerance    = 1e-8; 

    // Set the area of the surface facing the s/c [m^2] 
    const Real area         = 1.2 * 1.1; 

    // Set the mass of the s/c [kg]
    const Real mass        = 215.0; 

    // Set the speed of light [m sec^-1]
    const Real speedOfLight   = 3e8;

    // Set the Solar constant [W m^-2]
    const Real solarConstant  = 1367.0; 

    // Set the radiation pressure coefficient [adim]
    const Real radiationPressureCoefficient = 1.0 + 0.6; // This is given in SMAD as (1+q)

    // Set the vector to source vector [adim]
    const Vector3 vectorToSource( 1.0, 0.0, 0.0 );

    // Calculate the radiation pressure value [N m^-2]   
    const Real radiationPressure    = solarConstant / speedOfLight;

    // Set the center of pressure.
    const Real centerOfSolarPressure    = 0.1;

    // Set the center of mass.
    const Real centerOfMass             = 0.0; 

    // Compute the solar radiation disturbance torque. 
    const Vector3 computedSolarRadiationTorque      = astro::computeSolarRadiationDisturbanceTorque( centerOfSolarPressure, 
                                                                                    centerOfMass, 
                                                                                    radiationPressure,
                                                                                    radiationPressureCoefficient, 
                                                                                    vectorToSource, 
                                                                                    area, 
                                                                                    mass ); 

    // Given in SMAD for Firesat example. 
    const Vector3 expectedSolarRadiationTorque( -9.6e-7, 0.0, 0.0 );

    // Compute the error in the solar radiation pressure torque.
    const Vector3 errorInSolarRadiationPressureTorque = expectedSolarRadiationTorque - computedSolarRadiationTorque; 

    REQUIRE( errorInSolarRadiationPressureTorque.array()[0] < tolerance ); 
    REQUIRE( errorInSolarRadiationPressureTorque.array()[1] < tolerance ); 
    REQUIRE( errorInSolarRadiationPressureTorque.array()[2] < tolerance ); 
}

} //namespace tests    

} // namespace astro 