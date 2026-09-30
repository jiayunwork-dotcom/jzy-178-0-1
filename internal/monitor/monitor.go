// Package monitor owns cross-epoch state: satellite isolation/recovery and
// alarm persistence.
package monitor

import (
	"errors"
	"fmt"
	"math"
	"sort"

	"gnss-integrity/internal/chisq"
	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/position"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/raim"
)

var (
	// ErrDuplicateTimestamp means an epoch with this timestamp has already been
	// incorporated into the session.
	ErrDuplicateTimestamp = errors.New("duplicate epoch timestamp")
	// ErrTimestampBeforeLast rejects out-of-order replay data.
	ErrTimestampBeforeLast = errors.New("epoch timestamp is before the last accepted timestamp")
)

// Epoch is one submitted set of visible pseudoranges. FaultyPRN is optional
// ground-truth metadata used solely to classify false alarms in injected-data
// tests; zero/nil means no declared fault.
type Epoch struct {
	Timestamp    int64
	Measurements []position.Measurement
	Initial      coords.Vec3
	FaultyPRN    *int
}

// SatelliteState records cross-epoch isolation bookkeeping.
type SatelliteState struct {
	PRN                 int  `json:"prn"`
	Isolated            bool `json:"isolated"`
	RecoveryConsecutive int  `json:"recovery_consecutive"`
	VisibleLastEpoch    bool `json:"visible_last_epoch"`
}

// Statistics are session-level availability and alarm counters.
type Statistics struct {
	AcceptedEpochs  int     `json:"accepted_epochs"`
	AvailableEpochs int     `json:"available_epochs"`
	Availability    float64 `json:"availability"`
	DetectionCount  int     `json:"detection_count"`
	AlarmCount      int     `json:"alarm_count"`
	FalseAlarmCount int     `json:"false_alarm_count"`
}

// State is the complete durable session state.
type State struct {
	Profile          profile.Profile        `json:"profile"`
	Satellites       map[int]SatelliteState `json:"satellites"`
	LastTimestamp    int64                  `json:"last_timestamp"`
	HasLastTimestamp bool                   `json:"has_last_timestamp"`
	AlarmActive      bool                   `json:"alarm_active"`
	UnsafeStreak     int                    `json:"unsafe_streak"`
	SafeStreak       int                    `json:"safe_streak"`
	StreakFaultPRNs  map[int]bool           `json:"streak_fault_prns,omitempty"`
	Stats            Statistics             `json:"statistics"`
	LastSnapshot     *raim.Result           `json:"last_snapshot,omitempty"`
}

// EpochResult contains the single-epoch snapshot and resulting state changes.
type EpochResult struct {
	Snapshot          *raim.Result `json:"snapshot"`
	AlarmRaised       bool         `json:"alarm_raised"`
	AlarmCleared      bool         `json:"alarm_cleared"`
	AlarmActive       bool         `json:"alarm_active"`
	UnsafeStreak      int          `json:"unsafe_streak"`
	SafeStreak        int          `json:"safe_streak"`
	RecoveredPRNs     []int        `json:"recovered_prns"`
	NewlyIsolatedPRNs []int        `json:"newly_isolated_prns"`
	Statistics        Statistics   `json:"statistics"`
}

// NewState creates a session bound to an immutable profile snapshot.
func NewState(p profile.Profile) *State {
	return &State{
		Profile:    p,
		Satellites: map[int]SatelliteState{},
	}
}

// Clone deep-copies state, allowing batch processing to roll back after any
// validation/processing failure.
func (s *State) Clone() *State {
	cp := *s
	cp.Satellites = make(map[int]SatelliteState, len(s.Satellites))
	for k, v := range s.Satellites {
		cp.Satellites[k] = v
	}
	if s.StreakFaultPRNs != nil {
		cp.StreakFaultPRNs = make(map[int]bool, len(s.StreakFaultPRNs))
		for prn := range s.StreakFaultPRNs {
			cp.StreakFaultPRNs[prn] = true
		}
	}
	if s.LastSnapshot != nil {
		snap := *s.LastSnapshot
		cp.LastSnapshot = &snap
	}
	return &cp
}

// IsolatedPRNs returns currently isolated satellite numbers.
func (s *State) IsolatedPRNs() map[int]bool {
	out := map[int]bool{}
	for prn, st := range s.Satellites {
		if st.Isolated {
			out[prn] = true
		}
	}
	return out
}

// Process incorporates one already field-validated epoch.
func (s *State) Process(e Epoch) (*EpochResult, error) {
	if s.HasLastTimestamp {
		if e.Timestamp == s.LastTimestamp {
			return nil, ErrDuplicateTimestamp
		}
		if e.Timestamp < s.LastTimestamp {
			return nil, fmt.Errorf("%w: %d < %d", ErrTimestampBeforeLast, e.Timestamp, s.LastTimestamp)
		}
	}

	isolatedBefore := s.IsolatedPRNs()
	snap := raim.Evaluate(e.Measurements, isolatedBefore, e.Initial, s.Profile)
	snap.Timestamp = e.Timestamp

	visible := map[int]bool{}
	for _, m := range e.Measurements {
		visible[m.PRN] = true
	}
	for prn := range s.Satellites {
		st := s.Satellites[prn]
		st.VisibleLastEpoch = visible[prn]
		if st.Isolated && !visible[prn] {
			// Recovery evidence must be consecutive and from the satellite itself.
			st.RecoveryConsecutive = 0
		}
		s.Satellites[prn] = st
	}
	for _, m := range e.Measurements {
		st := s.Satellites[m.PRN]
		st.PRN = m.PRN
		st.VisibleLastEpoch = true
		s.Satellites[m.PRN] = st
	}

	// Calculate add-back evidence using the final active single-epoch solution.
	activeMeasurements := make([]position.Measurement, 0, len(e.Measurements))
	allByPRN := map[int]position.Measurement{}
	for _, m := range e.Measurements {
		allByPRN[m.PRN] = m
		if !isolatedBefore[m.PRN] {
			activeMeasurements = append(activeMeasurements, m)
		}
	}
	canUseSnapshotForRecovery := snap.Status == raim.StatusOK || snap.Status == raim.StatusExcluded
	var recovered []int
	for prn := range isolatedBefore {
		if !visible[prn] {
			continue
		}
		check := recoveryCheck(activeMeasurements, allByPRN[prn], e.Initial, s.Profile, canUseSnapshotForRecovery)
		snap.RecoveryChecks[prn] = check
		st := s.Satellites[prn]
		if check.Accepted {
			st.RecoveryConsecutive++
		} else {
			st.RecoveryConsecutive = 0
		}
		s.Satellites[prn] = st
	}

	newlyIsolated := []int{}
	if snap.Status == raim.StatusExcluded && snap.ExcludedPRN != 0 {
		prn := snap.ExcludedPRN
		st := s.Satellites[prn]
		st.PRN = prn
		st.Isolated = true
		st.RecoveryConsecutive = 0
		st.VisibleLastEpoch = true
		s.Satellites[prn] = st
		newlyIsolated = append(newlyIsolated, prn)
		snap.IsolatedPRNs = append(snap.IsolatedPRNs, prn)
		sort.Ints(snap.IsolatedPRNs)
		for i := range snap.Residuals {
			if snap.Residuals[i].PRN == prn {
				snap.Residuals[i].Used = false
				snap.Residuals[i].Excluded = true
			}
		}
	}

	// Isolation is released only before the next epoch's active set is built.
	recoveryPRNs := make([]int, 0, len(isolatedBefore))
	for prn := range isolatedBefore {
		st := s.Satellites[prn]
		if st.Isolated && st.RecoveryConsecutive >= s.Profile.RecoveryEpochs {
			st.Isolated = false
			st.RecoveryConsecutive = 0
			s.Satellites[prn] = st
			recoveryPRNs = append(recoveryPRNs, prn)
		}
	}
	sort.Ints(recoveryPRNs)
	recovered = recoveryPRNs

	er := s.applyAlarm(snap, e.FaultyPRN)
	s.LastTimestamp = e.Timestamp
	s.HasLastTimestamp = true
	s.LastSnapshot = snap

	s.Stats.AcceptedEpochs++
	serviceAvailable := snap.IntegrityAvailable && snap.HPL > 0 && snap.HPL <= s.Profile.HorizontalAlertLimit
	if serviceAvailable {
		s.Stats.AvailableEpochs++
	}
	if snap.Status == raim.StatusExcluded || snap.Status == raim.StatusDetectionFailed || snap.Status == raim.StatusExclusionFailed {
		s.Stats.DetectionCount++
	}
	s.Stats.Availability = float64(s.Stats.AvailableEpochs) / float64(s.Stats.AcceptedEpochs)
	er.Snapshot = snap
	er.RecoveredPRNs = recovered
	er.NewlyIsolatedPRNs = newlyIsolated
	er.Statistics = s.Stats
	return er, nil
}

func (s *State) applyAlarm(snap *raim.Result, faultyPRN *int) *EpochResult {
	er := &EpochResult{AlarmActive: s.AlarmActive}
	safe := snap.IntegrityAvailable && snap.HPL > 0 && snap.HPL <= s.Profile.HorizontalAlertLimit && !snap.SnapshotUnsafe

	switch {
	case snap.SnapshotUnsafe:
		s.UnsafeStreak++
		s.SafeStreak = 0
		if s.StreakFaultPRNs == nil {
			s.StreakFaultPRNs = map[int]bool{}
		}
		if faultyPRN != nil && *faultyPRN != 0 {
			s.StreakFaultPRNs[*faultyPRN] = true
		}
		if !s.AlarmActive && s.UnsafeStreak >= s.Profile.AlarmEpochs {
			s.AlarmActive = true
			er.AlarmRaised = true
			s.Stats.AlarmCount++
			if len(s.StreakFaultPRNs) == 0 {
				s.Stats.FalseAlarmCount++
			}
		}
	case safe:
		s.SafeStreak++
		s.UnsafeStreak = 0
		s.StreakFaultPRNs = nil
		if s.AlarmActive && s.SafeStreak >= s.Profile.ClearAlarmEpochs {
			s.AlarmActive = false
			er.AlarmCleared = true
		}
	default:
		// Integrity unavailable but no unsafe condition (e.g. four satellites):
		// neither confirms nor clears an alarm.
		s.UnsafeStreak = 0
		s.SafeStreak = 0
		s.StreakFaultPRNs = nil
	}

	er.AlarmActive = s.AlarmActive
	er.UnsafeStreak = s.UnsafeStreak
	er.SafeStreak = s.SafeStreak
	return er
}

func recoveryCheck(active []position.Measurement, isolated position.Measurement, initial coords.Vec3, p profile.Profile, snapshotAcceptable bool) raim.RecoveryCheck {
	check := raim.RecoveryCheck{PRN: isolated.PRN}
	combined := make([]position.Measurement, 0, len(active)+1)
	combined = append(combined, active...)
	combined = append(combined, isolated)
	if len(combined) < 5 {
		return check
	}
	sol, err := position.Solve(combined, initial)
	if err != nil {
		check.AddbackSingular = true
		return check
	}
	for _, r := range sol.Residuals {
		if r.PRN == isolated.PRN {
			check.Residual = r.Value
			check.NormalizedZ = r.Weighted
			check.ZPassed = math.Abs(r.Weighted) <= p.RecoveryZLimit
		}
	}
	df := len(combined) - 4
	threshold := chisq.Quantile(float64(df), 1.0-p.ProbabilityFalseAlarm)
	check.AddbackStat = sol.SSE
	check.AddbackPassed = snapshotAcceptable && sol.SSE <= threshold
	check.Accepted = check.ZPassed && check.AddbackPassed
	return check
}
