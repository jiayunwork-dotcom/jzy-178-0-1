// Package coords implements WGS-84 ECEF/geodetic conversions.
package coords

import "math"

// WGS84 is the WGS-84 reference ellipsoid.
const (
	WGS84A    = 6378137.0
	WGS84F    = 1.0 / 298.257223563
	WGS84B    = WGS84A * (1.0 - WGS84F)
	WGS84E2   = WGS84F * (2.0 - WGS84F)
	WGS84EP2  = WGS84E2 / (1.0 - WGS84E2)
	minRadius = 1.0e3 // ECEF positions closer than 1 km to Earth's center are rejected.
)

// Vec3 is an ECEF Cartesian position in metres.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// LLA is latitude/longitude in degrees and height above ellipsoid in metres.
type LLA struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude"`
}

// Finite reports whether all components are finite real numbers.
func (v Vec3) Finite() bool {
	return math.IsInf(v.X, 0) == false && !math.IsNaN(v.X) &&
		math.IsInf(v.Y, 0) == false && !math.IsNaN(v.Y) &&
		math.IsInf(v.Z, 0) == false && !math.IsNaN(v.Z)
}

// Norm returns the Cartesian distance from Earth's center.
func (v Vec3) Norm() float64 { return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z) }

// NearEarthCenter reports whether the supplied approximate receiver position
// cannot be used as a geodetic starting position.
func NearEarthCenter(v Vec3) bool { return !v.Finite() || v.Norm() < minRadius }

// ECEFToLLA uses the standard closed-form Bowring/WGS-84 conversion.
func ECEFToLLA(v Vec3) LLA {
	x, y, z := v.X, v.Y, v.Z
	lon := math.Atan2(y, x)
	p := math.Hypot(x, y)
	theta := math.Atan2(z*WGS84A, p*WGS84B)
	sinTheta, cosTheta := math.Sincos(theta)
	lat := math.Atan2(
		z+WGS84EP2*WGS84B*sinTheta*sinTheta*sinTheta,
		p-WGS84E2*WGS84A*cosTheta*cosTheta*cosTheta,
	)
	sinLat, cosLat := math.Sincos(lat)
	n := WGS84A / math.Sqrt(1.0-WGS84E2*sinLat*sinLat)
	h := 0.0
	if cosLat > 1e-12 {
		h = p/cosLat - n
	} else {
		h = math.Abs(z)/math.Sqrt(1.0-WGS84E2) - WGS84B
	}
	return LLA{
		Latitude:  lat * 180.0 / math.Pi,
		Longitude: lon * 180.0 / math.Pi,
		Altitude:  h,
	}
}

// LocalBasis returns the east, north and up unit vectors at a latitude and
// longitude in radians.
func LocalBasis(lat, lon float64) (east, north, up Vec3) {
	sLat, cLat := math.Sincos(lat)
	sLon, cLon := math.Sincos(lon)
	east = Vec3{-sLon, cLon, 0}
	north = Vec3{-sLat * cLon, -sLat * sLon, cLat}
	up = Vec3{cLat * cLon, cLat * sLon, sLat}
	return
}
