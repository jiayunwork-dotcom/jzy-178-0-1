package coords

import (
	"math"
	"testing"
)

func TestECEFLLARoundTripOrigin(t *testing.T) {
	v := Vec3{X: WGS84A, Y: 0, Z: 0}
	lla := ECEFToLLA(v)
	if math.Abs(lla.Latitude) > 1e-9 || math.Abs(lla.Longitude) > 1e-9 || math.Abs(lla.Altitude) > 1e-6 {
		t.Fatalf("unexpected LLA: %+v", lla)
	}
}

func TestFiniteAndNearEarthCenter(t *testing.T) {
	if !NearEarthCenter(Vec3{}) {
		t.Fatal("zero vector should be flagged")
	}
	if !NearEarthCenter(Vec3{X: math.NaN()}) {
		t.Fatal("NaN vector should be flagged")
	}
	if NearEarthCenter(Vec3{X: WGS84A}) {
		t.Fatal("surface receiver must be valid")
	}
}
