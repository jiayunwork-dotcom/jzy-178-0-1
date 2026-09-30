package raim_test

import (
	"math"
	"testing"

	"gnss-integrity/internal/chisq"
	"gnss-integrity/internal/position"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/raim"
	"gnss-integrity/internal/sim"
)

func testProfile() profile.Profile {
	p := profile.Builtins()["enroute"]
	p.HorizontalAlertLimit = 1e6 // geometry-only tests should not alarm on HAL
	return p
}

func detectionThreshold(p profile.Profile, df int) float64 {
	return chisq.Quantile(float64(df), 1-p.ProbabilityFalseAlarm)
}

func TestNoFaultStatisticDistributionAndStatus(t *testing.T) {
	p := testProfile()
	df := 4
	const samples = 6000
	rng := sim.NewRNG(20260930)
	var sum float64
	p.ProbabilityFalseAlarm = 1e-9
	threshold := detectionThreshold(p, df)
	var over int
	for i := 0; i < samples; i++ {
		ms := sim.Measurements(sim.DefaultSky(), 2.0, 0, 0, 0, rng)
		res := raim.Evaluate(ms, nil, sim.ApproximatePosition(), p)
		if res.Status != raim.StatusOK || res.DegreesFreedom != df {
			t.Fatalf("status=%s df=%d", res.Status, res.DegreesFreedom)
		}
		if math.Abs(res.Threshold-threshold) > 1e-9 {
			t.Fatalf("threshold not attached: %v %v", res.Threshold, threshold)
		}
		sum += res.Statistic
		if res.Statistic > threshold {
			over++
		}
	}
	mean := sum / samples
	if rel := math.Abs(mean-float64(df)) / float64(df); rel > 0.05 {
		t.Fatalf("mean=%v df=%d relative=%.3f", mean, df, rel)
	}
	rate := float64(over) / samples
	// Six thousand samples should not contain an event at Pfa=1e-9.
	if rate > 0.001 {
		t.Fatalf("empirical false alarm rate %v", rate)
	}
}

func TestLargeBiasDetectedAndCorrectSatelliteExcluded(t *testing.T) {
	p := testProfile()
	ms := sim.Measurements(sim.DefaultSky(), 0.1, 0, 4, 850, sim.NewRNG(7))
	bad := raim.Evaluate(ms, nil, sim.ApproximatePosition(), p)
	if bad.Status != raim.StatusExcluded || bad.ExcludedPRN != 4 {
		t.Fatalf("status=%s excluded=%d", bad.Status, bad.ExcludedPRN)
	}
	d := math.Hypot(math.Hypot(bad.Position.X-sim.Receiver.X, bad.Position.Y-sim.Receiver.Y), bad.Position.Z-sim.Receiver.Z)
	if d > 2 {
		t.Fatalf("post-exclusion position error %.3f", d)
	}
}

func TestStatisticMonotonicWithBias(t *testing.T) {
	var previous float64
	for _, bias := range []float64{0, 10, 25, 50, 100, 250, 600} {
		ms := sim.Measurements(sim.DefaultSky(), 0.1, 0, 2, bias, nil)
		sol, err := solveAll(ms)
		if err != nil {
			t.Fatal(err)
		}
		if sol.SSE < previous {
			t.Fatalf("bias %v statistic %v decreased from %v", bias, sol.SSE, previous)
		}
		previous = sol.SSE
	}
}

func TestThresholdGrowsAsFalseAlarmShrinks(t *testing.T) {
	p1 := testProfile()
	p2 := p1
	p2.ProbabilityFalseAlarm = 1e-7
	if detectionThreshold(p2, 5) <= detectionThreshold(p1, 5) {
		t.Fatal("smaller Pfa must increase threshold")
	}
}

func TestPoorGeometryIncreasesHDOPAndHPL(t *testing.T) {
	p := testProfile()
	good := raim.Evaluate(sim.Measurements(sim.DefaultSky(), 0.1, 0, 0, 0, nil), nil, sim.ApproximatePosition(), p)
	poor := raim.Evaluate(sim.Measurements(sim.PoorSky(), 0.1, 0, 0, 0, nil), nil, sim.ApproximatePosition(), p)
	if !(poor.HDOP > good.HDOP) || !(poor.HPL > good.HPL) || poor.HPL <= 0 {
		t.Fatalf("good HDOP/HPL=%v/%v poor=%v/%v", good.HDOP, good.HPL, poor.HDOP, poor.HPL)
	}
}

func solveAll(ms []position.Measurement) (*position.Solution, error) {
	return position.Solve(ms, sim.ApproximatePosition())
}

func TestFourAndFiveSatelliteRules(t *testing.T) {
	p := testProfile()
	sky := sim.DefaultSky()[:5]
	res := raim.Evaluate(sim.Measurements(sky, 0.1, 0, 0, 0, nil), nil, sim.ApproximatePosition(), p)
	if res.Status != raim.StatusOK || res.DegreesFreedom != 1 || res.HPL <= 0 {
		t.Fatalf("five satellite result: %+v", res.Status)
	}
	four := sim.DefaultSky()[:4]
	res = raim.Evaluate(sim.Measurements(four, 0.1, 0, 0, 0, nil), nil, sim.ApproximatePosition(), p)
	if res.Status != raim.StatusUnavailable || res.IntegrityAvailable {
		t.Fatalf("four satellite result: %+v", res)
	}
}
