package monitor

import (
	"errors"
	"testing"

	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/sim"
)

func recoveryProfile() profile.Profile {
	p := profile.Builtins()["enroute"]
	p.HorizontalAlertLimit = 1e9
	p.RecoveryEpochs = 3
	p.AlarmEpochs = 3
	p.ClearAlarmEpochs = 3
	return p
}

func epoch(ts int64, biasPRN int, bias float64, faulty *int) Epoch {
	return Epoch{
		Timestamp:    ts,
		Measurements: sim.Measurements(sim.DefaultSky(), 0.05, 0, biasPRN, bias, sim.NewRNG(uint64(ts*100+17))),
		Initial:      sim.ApproximatePosition(),
		FaultyPRN:    faulty,
	}
}

func TestIsolationPersistsUntilRecoveryEpochs(t *testing.T) {
	p := recoveryProfile()
	st := NewState(p)
	fault := 4
	r, err := st.Process(epoch(1, 4, 900, &fault))
	if err != nil || r.NewlyIsolatedPRNs[0] != 4 {
		t.Fatalf("isolation failed: %v %+v", err, r.NewlyIsolatedPRNs)
	}
	// Fault remains for two epochs: satellite is not pulled back.
	for ts := int64(2); ts <= 3; ts++ {
		r, err = st.Process(epoch(ts, 4, 900, &fault))
		if err != nil {
			t.Fatal(err)
		}
		if len(r.NewlyIsolatedPRNs) != 0 || !st.Satellites[4].Isolated {
			t.Fatalf("faulty satellite re-entered at %d", ts)
		}
	}
	// Fault removed; first two clean epochs must keep isolation.
	for ts := int64(4); ts <= 5; ts++ {
		r, err = st.Process(epoch(ts, 0, 0, nil))
		if err != nil {
			t.Fatal(err)
		}
		if len(r.RecoveredPRNs) != 0 {
			t.Fatalf("recovered early at %d", ts)
		}
		if st.Satellites[4].RecoveryConsecutive >= p.RecoveryEpochs {
			t.Fatal("recovery counter impossible before configured epochs")
		}
	}
	r, err = st.Process(epoch(6, 0, 0, nil))
	if err != nil || len(r.RecoveredPRNs) != 1 || r.RecoveredPRNs[0] != 4 {
		t.Fatalf("recovery failed: %v %+v", err, r.RecoveredPRNs)
	}
	if st.Satellites[4].Isolated {
		t.Fatal("satellite remains isolated")
	}
}

func TestDuplicateAndBackwardTimestampsRejected(t *testing.T) {
	st := NewState(recoveryProfile())
	if _, err := st.Process(epoch(10, 0, 0, nil)); err != nil {
		t.Fatal(err)
	}
	if _, err := st.Process(epoch(10, 0, 0, nil)); !errors.Is(err, ErrDuplicateTimestamp) {
		t.Fatalf("duplicate: %v", err)
	}
	before := *st
	if _, err := st.Process(epoch(9, 0, 0, nil)); !errors.Is(err, ErrTimestampBeforeLast) {
		t.Fatalf("backward: %v", err)
	}
	if st.Stats.AcceptedEpochs != before.Stats.AcceptedEpochs {
		t.Fatal("rejected timestamp advanced state")
	}
}

func TestAlarmPersistenceAndFalseAlarmCount(t *testing.T) {
	p := recoveryProfile()
	p.HorizontalAlertLimit = 0.2 // below the clean geometry HPL, forcing unsafe snapshots
	p.AlarmEpochs = 3
	p.ClearAlarmEpochs = 2
	st := NewState(p)
	var raisedAt int64
	for ts := int64(1); ts <= 4; ts++ {
		r, err := st.Process(epoch(ts, 0, 0, nil))
		if err != nil {
			t.Fatal(err)
		}
		if r.AlarmRaised {
			raisedAt = ts
		}
	}
	if raisedAt != 3 {
		t.Fatalf("alarm raised at %d, expected third persistent unsafe epoch", raisedAt)
	}
	if st.Stats.AlarmCount != 1 || st.Stats.FalseAlarmCount != 1 {
		t.Fatalf("counts=%+v", st.Stats)
	}
}
