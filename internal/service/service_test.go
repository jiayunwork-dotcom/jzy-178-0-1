package service

import (
	"path/filepath"
	"reflect"
	"testing"

	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/sim"
)

func testProfile() profile.Profile {
	p := profile.Builtins()["enroute"]
	p.Name = "test-replay"
	p.HorizontalAlertLimit = 900000
	p.RecoveryEpochs = 2
	p.AlarmEpochs = 2
	p.ClearAlarmEpochs = 2
	return p
}

func makeEpochs(n int, faultStart, faultEnd int64, faultyPRN int, bias float64) []monitor.Epoch {
	out := make([]monitor.Epoch, n)
	for i := range out {
		ts := int64(i + 1)
		biasPRN, thisBias := 0, 0.0
		var faulty *int
		if ts >= faultStart && ts <= faultEnd {
			biasPRN, thisBias, faulty = faultyPRN, bias, &faultyPRN
		}
		out[i] = monitor.Epoch{
			Timestamp:    ts,
			Measurements: sim.Measurements(sim.DefaultSky(), 0.03, 0, biasPRN, thisBias, sim.NewRNG(uint64(1000+i))),
			Initial:      sim.ApproximatePosition(),
			FaultyPRN:    faulty,
		}
	}
	return out
}

func newSession(t *testing.T, svc *Service, p profile.Profile) string {
	t.Helper()
	if err := svc.CreateProfile(p); err != nil {
		t.Fatal(err)
	}
	id, _, err := svc.CreateSession(p.Name)
	if err != nil {
		t.Fatal(err)
	}
	return id
}

func TestBatchAndIndividualSubmissionAreIdentical(t *testing.T) {
	dir := t.TempDir()
	p := testProfile()
	epochs := makeEpochs(20, 5, 10, 3, 800)

	svcA, err := New(filepath.Join(dir, "a"))
	if err != nil {
		t.Fatal(err)
	}
	idA := newSession(t, svcA, p)
	batch, err := svcA.ProcessBatch(idA, epochs)
	if err != nil {
		t.Fatal(err)
	}
	stateA, _ := svcA.Session(idA)

	svcB, _ := New(filepath.Join(dir, "b"))
	idB := newSession(t, svcB, p)
	individual := make([]*monitor.EpochResult, 0, len(epochs))
	for _, e := range epochs {
		r, err := svcB.ProcessEpoch(idB, e)
		if err != nil {
			t.Fatal(err)
		}
		individual = append(individual, r)
	}
	stateB, _ := svcB.Session(idB)

	if !reflect.DeepEqual(batch, individual) {
		for i := range batch {
			if !reflect.DeepEqual(batch[i], individual[i]) {
				t.Fatalf("epoch %d differs:\nbatch=%#v\nstep=%#v", i+1, batch[i], individual[i])
			}
		}
	}
	if !reflect.DeepEqual(stateA, stateB) {
		t.Fatalf("final states differ:\n%#v\n%#v", stateA, stateB)
	}
}

func TestRestartResumeMatchesContinuousRun(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "data")
	p := testProfile()
	epochs := makeEpochs(18, 4, 12, 5, 850)

	svc, err := New(dir)
	if err != nil {
		t.Fatal(err)
	}
	if err := svc.CreateProfile(p); err != nil {
		t.Fatal(err)
	}
	id, _, err := svc.CreateSession(p.Name)
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range epochs[:7] {
		if _, err := svc.ProcessEpoch(id, e); err != nil {
			t.Fatal(err)
		}
	}

	// A brand-new service instance is equivalent to a process restart.
	restarted, err := New(dir)
	if err != nil {
		t.Fatal(err)
	}
	for _, e := range epochs[7:] {
		if _, err := restarted.ProcessEpoch(id, e); err != nil {
			t.Fatal(err)
		}
	}
	resumedState, _ := restarted.Session(id)

	continuousDir := filepath.Join(t.TempDir(), "continuous")
	continuous, _ := New(continuousDir)
	if err := continuous.CreateProfile(p); err != nil {
		t.Fatal(err)
	}
	idC, _, _ := continuous.CreateSession(p.Name)
	if _, err := continuous.ProcessBatch(idC, epochs); err != nil {
		t.Fatal(err)
	}
	continuousState, _ := continuous.Session(idC)

	if !reflect.DeepEqual(resumedState, continuousState) {
		t.Fatalf("restart/resume state differs")
	}
}

func TestBatchValidationDoesNotAdvanceState(t *testing.T) {
	svc, err := New(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	id := newSession(t, svc, testProfile())
	good := makeEpochs(3, 0, 0, 0, 0)
	if _, err := svc.ProcessBatch(id, good); err != nil {
		t.Fatal(err)
	}
	bad := makeEpochs(4, 0, 0, 0, 0)
	bad = append(bad, good...) // duplicate timestamps
	if _, err := svc.ProcessBatch(id, bad); err == nil {
		t.Fatal("expected duplicate batch rejection")
	}
	st, _ := svc.Session(id)
	if st.Stats.AcceptedEpochs != 3 || st.LastTimestamp != 3 {
		t.Fatalf("state advanced after failed batch: %+v", st.Stats)
	}
}
