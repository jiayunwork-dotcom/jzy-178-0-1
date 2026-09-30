package validation

import (
	"math"
	"strings"
	"testing"

	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/position"
	"gnss-integrity/internal/sim"
)

func validEpoch() monitor.Epoch {
	return monitor.Epoch{
		Timestamp:    1,
		Measurements: sim.Measurements(sim.DefaultSky(), 1, 0, 0, 0, nil),
		Initial:      sim.ApproximatePosition(),
	}
}

func TestValidEpoch(t *testing.T) {
	if err := ValidateEpoch(validEpoch()); err != nil {
		t.Fatal(err)
	}
}

func TestInvalidEpochFields(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*monitor.Epoch)
		field  string
	}{
		{"timestamp", func(e *monitor.Epoch) { e.Timestamp = 0 }, "timestamp"},
		{"too few", func(e *monitor.Epoch) { e.Measurements = e.Measurements[:3] }, "satellites"},
		{"duplicate prn", func(e *monitor.Epoch) { e.Measurements[1].PRN = e.Measurements[0].PRN }, "satellites[1].prn"},
		{"nan coordinate", func(e *monitor.Epoch) { e.Measurements[2].Satellite.X = math.NaN() }, "satellites[2].satellite"},
		{"inf pseudorange", func(e *monitor.Epoch) { e.Measurements[2].Pseudorange = math.Inf(1) }, "satellites[2].pseudorange"},
		{"zero sigma", func(e *monitor.Epoch) { e.Measurements[3].Sigma = 0 }, "satellites[3].sigma"},
		{"negative sigma", func(e *monitor.Epoch) { e.Measurements[3].Sigma = -1 }, "satellites[3].sigma"},
		{"earth center", func(e *monitor.Epoch) { e.Initial = coords.Vec3{} }, "initial_position"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			e := validEpoch()
			c.mutate(&e)
			err := ValidateEpoch(e)
			if err == nil {
				t.Fatal("expected error")
			}
			var fe FieldError
			if !asField(err, &fe) {
				t.Fatalf("not a field error: %v", err)
			}
			if fe.Field != c.field {
				t.Fatalf("field=%q want %q (%v)", fe.Field, c.field, err)
			}
		})
	}
}

func TestCoplanarGeometryRejected(t *testing.T) {
	e := validEpoch()
	// All satellites at identical azimuth have coplanar line-of-sight vectors.
	e.Measurements = []position.Measurement{}
	for prn := 1; prn <= 8; prn++ {
		ms := sim.Measurements([]sim.Satellite{{PRN: prn, Azimuth: 90, Elevation: float64(20 + prn*5), Range: 21_500_000}}, 1, 0, 0, 0, nil)
		e.Measurements = append(e.Measurements, ms...)
	}
	err := ValidateEpoch(e)
	if err == nil || !strings.Contains(err.Error(), "singular") {
		t.Fatalf("expected singular geometry, got %v", err)
	}
}

func TestBatchLimitAndIndexedField(t *testing.T) {
	epochs := make([]monitor.Epoch, 3601)
	for i := range epochs {
		epochs[i] = validEpoch()
	}
	if err := ValidateBatch(epochs, 3600); err == nil {
		t.Fatal("expected batch limit error")
	}
	epochs = epochs[:2]
	epochs[1].Measurements[0].Sigma = 0
	err := ValidateBatch(epochs, 3600)
	if err == nil || !strings.Contains(err.Error(), "epochs[1].satellites[0].sigma") {
		t.Fatalf("indexed error: %v", err)
	}
}

func asField(err error, target *FieldError) bool {
	for err != nil {
		if f, ok := err.(FieldError); ok {
			*target = f
			return true
		}
		type unwrapper interface{ Unwrap() error }
		u, ok := err.(unwrapper)
		if !ok {
			return false
		}
		err = u.Unwrap()
	}
	return false
}
