// Package sim contains deterministic simulation helpers shared by tests and the
// replay-documentation command. It is not used by the server.
package sim

import (
	"math"

	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/position"
)

// Receiver is a point on the equatorial prime meridian in metres.
var Receiver = coords.Vec3{X: coords.WGS84A, Y: 0, Z: 0}

// RNG is a deterministic xorshift64* generator.
type RNG struct {
	state    uint64
	spare    float64
	hasSpare bool
}

// NewRNG seeds a generator. A zero seed is advanced once.
func NewRNG(seed uint64) *RNG {
	if seed == 0 {
		seed = 0x9e3779b97f4a7c15
	}
	return &RNG{state: seed}
}

// Uint64 returns a pseudo-random uint64.
func (r *RNG) Uint64() uint64 {
	r.state ^= r.state >> 12
	r.state ^= r.state << 25
	r.state ^= r.state >> 27
	return r.state * 0x2545f4914f6cdd1d
}

// Float64 returns a uniform value in [0,1).
func (r *RNG) Float64() float64 {
	return float64(r.Uint64()>>11) * 0x1.0p-53
}

// Normal returns a standard normal sample using Box-Muller.
func (r *RNG) Normal() float64 {
	if r.hasSpare {
		r.hasSpare = false
		return r.spare
	}
	u1, u2 := 0.0, 0.0
	for u1 == 0 {
		u1 = r.Float64()
	}
	u2 = r.Float64()
	mag := math.Sqrt(-2 * math.Log(u1))
	r.spare = mag * math.Sin(2*math.Pi*u2)
	r.hasSpare = true
	return mag * math.Cos(2*math.Pi*u2)
}

// Satellite describes a generated line of sight by azimuth/elevation angles in
// degrees and geometric range in metres.
type Satellite struct {
	PRN       int
	Azimuth   float64
	Elevation float64
	Range     float64
}

// DefaultSky is a well-distributed eight-satellite geometry.
func DefaultSky() []Satellite {
	els := []float64{18, 35, 55, 72, 22, 42, 62, 28}
	out := make([]Satellite, len(els))
	for i, el := range els {
		out[i] = Satellite{PRN: i + 1, Azimuth: float64(i) * 45, Elevation: el, Range: 21_500_000}
	}
	return out
}

// PoorSky clusters the satellites in one narrow sky sector, giving weak
// horizontal geometry.
func PoorSky() []Satellite {
	az := []float64{80, 85, 90, 95, 100, 88}
	els := []float64{10, 12, 14, 10, 12, 16}
	out := make([]Satellite, len(az))
	for i := range az {
		out[i] = Satellite{PRN: i + 1, Azimuth: az[i], Elevation: els[i], Range: 21_500_000}
	}
	return out
}

// Measurements builds noise-free/noise-added pseudoranges around Receiver.
// biasPRN of zero means no injected bias. clockBias and bias are in metres.
func Measurements(sky []Satellite, sigma float64, clockBias float64, biasPRN int, bias float64, rng *RNG) []position.Measurement {
	out := make([]position.Measurement, len(sky))
	for i, s := range sky {
		az := s.Azimuth * math.Pi / 180
		el := s.Elevation * math.Pi / 180
		// Local east/north/up at receiver (lat=0, lon=0): east=-Y, north=+Z, up=+X.
		east := -math.Sin(el) * math.Sin(az)
		north := math.Sin(el) * math.Cos(az)
		up := math.Cos(el)
		los := coords.Vec3{X: up, Y: east, Z: north}
		sat := coords.Vec3{
			X: Receiver.X + s.Range*los.X,
			Y: Receiver.Y + s.Range*los.Y,
			Z: Receiver.Z + s.Range*los.Z,
		}
		pr := s.Range + clockBias
		if s.PRN == biasPRN {
			pr += bias
		}
		if rng != nil {
			pr += sigma * rng.Normal()
		}
		out[i] = position.Measurement{PRN: s.PRN, Satellite: sat, Pseudorange: pr, Sigma: sigma}
	}
	return out
}

// ApproximatePosition is a deliberately imperfect receiver initial guess.
func ApproximatePosition() coords.Vec3 {
	return coords.Vec3{X: Receiver.X + 850, Y: Receiver.Y - 420, Z: Receiver.Z + 260}
}
