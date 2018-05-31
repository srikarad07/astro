/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <cmath>
#include <vector>

#include <iostream> // remove later

#include <Eigen/Dense>
#include <catch.hpp>

#include "astro/solarRadiationDisturbanceTorqueModel.hpp"

namespace astro
{
namespace tests 
{

typedef double Real; 
typedef Eigen::Matrix< double, 3, 1 > Vector3;

TEST_CASE("Compute solar radiation pressure torque for an arbitrary case", "[solar_radiation_pressure, acceleration, models]" )
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


} //namespace tests    

} // namespace astro 