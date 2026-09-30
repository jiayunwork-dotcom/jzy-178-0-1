package position_test

import (
	"math"
	"testing"

	"gnss-integrity/internal/position"
	"gnss-integrity/internal/sim"
)

func TestWeightedLeastSquaresRecoversTruth(t *testing.T) {
	ms := sim.Measurements(sim.DefaultSky(), 0.1, 0, 0, 0, nil)
	sol, err := position.Solve(ms, sim.ApproximatePosition())
	if err != nil {
		t.Fatal(err)
	}
	truth := sim.Receiver
	if math.Abs(sol.Position.X-truth.X)+math.Abs(sol.Position.Y-truth.Y)+math.Abs(sol.Position.Z-truth.Z) > 1e-5 {
		t.Fatalf("position=%v truth=%v", sol.Position, truth)
	}
	if math.Abs(sol.ClockBias) > 1e-5 {
		t.Fatalf("clock=%v", sol.ClockBias)
	}
	if sol.Iterations > position.MaxIterations || !sol.Converged {
		t.Fatalf("iterations=%d converged=%v", sol.Iterations, sol.Converged)
	}
}

func TestCommonPseudorangeConstantMovesOnlyClock(t *testing.T) {
	base := sim.Measurements(sim.DefaultSky(), 0.1, 0, 0, 0, nil)
	shifted := sim.Measurements(sim.DefaultSky(), 0.1, 73_421.9, 0, 0, nil)
	a, err := position.Solve(base, sim.ApproximatePosition())
	if err != nil {
		t.Fatal(err)
	}
	b, err := position.Solve(shifted, sim.ApproximatePosition())
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(b.Position.X-a.Position.X)+math.Abs(b.Position.Y-a.Position.Y)+math.Abs(b.Position.Z-a.Position.Z) > 1e-5 {
		t.Fatalf("position changed by common pseudorange offset: %v", b.Position)
	}
	if math.Abs(b.ClockBias-a.ClockBias-73_421.9) > 1e-5 {
		t.Fatalf("clock delta=%v", b.ClockBias-a.ClockBias)
	}
}

func TestWeightedNoiseSolution(t *testing.T) {
	ms := sim.Measurements(sim.DefaultSky(), 1.5, 0, 0, 0, sim.NewRNG(1234))
	sol, err := position.Solve(ms, sim.ApproximatePosition())
	if err != nil {
		t.Fatal(err)
	}
	d := math.Hypot(math.Hypot(sol.Position.X-sim.Receiver.X, sol.Position.Y-sim.Receiver.Y), sol.Position.Z-sim.Receiver.Z)
	if d > 5 {
		t.Fatalf("noisy solution error too large: %.3f m", d)
	}
}
