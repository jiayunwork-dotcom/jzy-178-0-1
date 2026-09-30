// Package raim implements single-epoch weighted chi-square Receiver Autonomous
// Integrity Monitoring, fault detection/exclusion and horizontal protection
// level calculation.
package raim

import (
	"math"
	"sort"

	"gnss-integrity/internal/chisq"
	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/position"
	"gnss-integrity/internal/profile"
)

// Status is the single-epoch integrity conclusion.
type Status string

const (
	StatusOK                 Status = "no_fault_detected"
	StatusExcluded           Status = "fault_detected_excluded"
	StatusDetectionFailed    Status = "fault_detection_failed"
	StatusExclusionFailed    Status = "fault_exclusion_failed"
	StatusUnavailable        Status = "integrity_unavailable"
	StatusInsufficientActive Status = "insufficient_active_measurements"
)

// ExclusionAttempt records one trial satellite removal.
type ExclusionAttempt struct {
	PRN       int     `json:"prn"`
	Passed    bool    `json:"passed"`
	Statistic float64 `json:"statistic"`
	Threshold float64 `json:"threshold"`
	Singular  bool    `json:"singular,omitempty"`
}

// RecoveryCheck is the isolated-satellite evidence gathered in an epoch.
type RecoveryCheck struct {
	PRN             int     `json:"prn"`
	Residual        float64 `json:"residual"`
	NormalizedZ     float64 `json:"normalized_z"`
	ZPassed         bool    `json:"z_passed"`
	AddbackStat     float64 `json:"addback_statistic"`
	AddbackPassed   bool    `json:"addback_passed"`
	AddbackSingular bool    `json:"addback_singular,omitempty"`
	Accepted        bool    `json:"accepted"`
}

// Result is the complete snapshot before cross-epoch persistence is applied.
type Result struct {
	Timestamp          int64                 `json:"timestamp"`
	ProfileName        string                `json:"profile_name"`
	Status             Status                `json:"status"`
	Position           *coords.Vec3          `json:"position,omitempty"`
	LLA                *coords.LLA           `json:"lla,omitempty"`
	ClockBias          float64               `json:"clock_bias"`
	Iterations         int                   `json:"iterations"`
	Converged          bool                  `json:"converged"`
	ActivePRNs         []int                 `json:"active_prns"`
	IsolatedPRNs       []int                 `json:"isolated_prns"`
	ExcludedPRN        int                   `json:"excluded_prn,omitempty"`
	ActiveCount        int                   `json:"active_count"`
	DegreesFreedom     int                   `json:"degrees_of_freedom"`
	Statistic          float64               `json:"statistic"`
	Threshold          float64               `json:"threshold"`
	ExclusionTries     []ExclusionAttempt    `json:"exclusion_attempts,omitempty"`
	Residuals          []position.Residual   `json:"residuals"`
	HDOP               float64               `json:"hdop"`
	HPL                float64               `json:"horizontal_protection_level"`
	HPLThreshold       float64               `json:"hpl_threshold"`
	HPLNoncentrality   float64               `json:"hpl_noncentrality"`
	HPLWithinHAL       bool                  `json:"hpl_within_hal"`
	IntegrityAvailable bool                  `json:"integrity_available"`
	SnapshotUnsafe     bool                  `json:"snapshot_unsafe"`
	UnsafeReasons      []string              `json:"unsafe_reasons,omitempty"`
	RecoveryChecks     map[int]RecoveryCheck `json:"recovery_checks,omitempty"`
	Solution           *position.Solution    `json:"-"`
}

// Evaluate runs the single-epoch solution. isolatedPRNs are visible satellites
// whose measurements must not enter the active solution; their residuals are
// still calculated for recovery evidence by the monitor.
func Evaluate(measurements []position.Measurement, isolated map[int]bool, initial coords.Vec3, p profile.Profile) *Result {
	all := measurements
	active := make([]position.Measurement, 0, len(all))
	isolatedList := make([]int, 0)
	for _, m := range all {
		if isolated[m.PRN] {
			isolatedList = append(isolatedList, m.PRN)
		} else {
			active = append(active, m)
		}
	}
	sort.Ints(isolatedList)
	res := &Result{
		Timestamp:      0,
		ProfileName:    p.Name,
		ActivePRNs:     prns(active),
		IsolatedPRNs:   isolatedList,
		ActiveCount:    len(active),
		RecoveryChecks: map[int]RecoveryCheck{},
	}
	if len(active) < 4 {
		res.Status = StatusInsufficientActive
		res.IntegrityAvailable = false
		res.UnsafeReasons = []string{"fewer than four active measurements"}
		fillAllResiduals(res, all, nil, isolated)
		return res
	}

	sol, err := position.Solve(active, initial)
	if err != nil {
		res.Status = StatusDetectionFailed
		res.IntegrityAvailable = false
		res.SnapshotUnsafe = true
		res.UnsafeReasons = []string{"active geometry cannot be solved: " + err.Error()}
		fillAllResiduals(res, all, nil, isolated)
		return res
	}
	n := len(active)
	setSolution(res, sol)
	res.DegreesFreedom = n - 4
	res.Threshold = detectionThreshold(p, n-4)
	res.Statistic = sol.SSE

	if n < 5 {
		res.Status = StatusUnavailable
		res.IntegrityAvailable = false
		res.UnsafeReasons = []string{"fewer than five satellites: position-only, integrity unavailable"}
		fillAllResiduals(res, all, sol, isolated)
		return res
	}

	if sol.SSE <= res.Threshold {
		res.Status = StatusOK
	} else if n < 6 {
		res.Status = StatusDetectionFailed
		res.SnapshotUnsafe = true
		res.UnsafeReasons = []string{"chi-square detection failed and fewer than six satellites prevent exclusion"}
	} else {
		exclude(active, initial, p, sol, res)
	}

	calculateHPL(res, p)
	fillAllResiduals(res, all, res.Solution, isolated)

	// Integrity is available when a valid all-clear test exists and its HPL is
	// within the alert limit. Detection/exclusion failure is immediately unsafe.
	switch res.Status {
	case StatusOK, StatusExcluded:
		res.IntegrityAvailable = res.ActiveCount >= 5 && res.HPL > 0
	default:
		res.IntegrityAvailable = false
	}
	if !res.IntegrityAvailable && !res.SnapshotUnsafe {
		// Four-satellite position-only mode is unavailable rather than an alarm.
		if res.Status != StatusUnavailable {
			res.SnapshotUnsafe = true
		}
	}
	if res.HPL > 0 && res.HPL > p.HorizontalAlertLimit {
		res.HPLWithinHAL = false
		res.SnapshotUnsafe = true
		res.UnsafeReasons = append(res.UnsafeReasons, "horizontal protection level exceeds horizontal alert limit")
	} else if res.HPL > 0 {
		res.HPLWithinHAL = true
	}
	return res
}

func exclude(active []position.Measurement, initial coords.Vec3, p profile.Profile, original *position.Solution, res *Result) {
	var passing []position.Solution
	for i := range active {
		subset := make([]position.Measurement, 0, len(active)-1)
		subset = append(subset, active[:i]...)
		subset = append(subset, active[i+1:]...)
		attempt := ExclusionAttempt{PRN: active[i].PRN, Threshold: detectionThreshold(p, len(subset)-4)}
		sol, err := position.Solve(subset, initial)
		if err != nil {
			attempt.Singular = true
		} else {
			attempt.Statistic = sol.SSE
			attempt.Passed = len(subset) >= 5 && sol.SSE <= attempt.Threshold
			if attempt.Passed {
				passing = append(passing, *sol)
			}
		}
		res.ExclusionTries = append(res.ExclusionTries, attempt)
	}
	if len(passing) != 1 {
		res.Status = StatusExclusionFailed
		res.SnapshotUnsafe = true
		res.UnsafeReasons = []string{"chi-square detection failed but no unique satellite explains the fault"}
		return
	}
	sol := passing[0]
	res.Status = StatusExcluded
	res.ExcludedPRN = findExcludedPRN(active, sol)
	res.ActivePRNs = prnsFromSolution(sol)
	res.ActiveCount = len(sol.Residuals)
	res.DegreesFreedom = res.ActiveCount - 4
	res.Statistic = sol.SSE
	res.Threshold = detectionThreshold(p, res.DegreesFreedom)
	setSolution(res, &sol)
}

func calculateHPL(res *Result, p profile.Profile) {
	if res.Solution == nil || res.ActiveCount < 5 {
		return
	}
	df := res.ActiveCount - 4
	t := detectionThreshold(p, df)
	lambda := chisq.NoncentralityForCDF(float64(df), t, p.ProbabilityMissedDetect)
	if lambda > 0 {
		res.HPL = res.Solution.MaxSlope * math.Sqrt(lambda)
		res.HPLThreshold = t
		res.HPLNoncentrality = lambda
	}
}

func detectionThreshold(p profile.Profile, df int) float64 {
	if df <= 0 {
		return 0
	}
	return chisq.Quantile(float64(df), 1.0-p.ProbabilityFalseAlarm)
}

func setSolution(res *Result, sol *position.Solution) {
	res.Solution = sol
	pos := sol.Position
	lla := sol.LLA
	res.Position = &pos
	res.LLA = &lla
	res.ClockBias = sol.ClockBias
	res.Iterations = sol.Iterations
	res.Converged = sol.Converged
	res.HDOP = sol.HDOP
}

func fillAllResiduals(res *Result, all []position.Measurement, sol *position.Solution, isolated map[int]bool) {
	byPRN := map[int]position.Residual{}
	if sol != nil {
		for _, r := range sol.Residuals {
			byPRN[r.PRN] = r
		}
	}
	res.Residuals = make([]position.Residual, 0, len(all))
	for _, m := range all {
		if r, ok := byPRN[m.PRN]; ok {
			r.Isolated = isolated[m.PRN]
			res.Residuals = append(res.Residuals, r)
			continue
		}
		r := position.Residual{PRN: m.PRN, Sigma: m.Sigma, Used: false, Isolated: isolated[m.PRN]}
		if sol != nil {
			dx := m.Satellite.X - sol.Position.X
			dy := m.Satellite.Y - sol.Position.Y
			dz := m.Satellite.Z - sol.Position.Z
			r.Value = math.Sqrt(dx*dx+dy*dy+dz*dz) + sol.ClockBias - m.Pseudorange
			r.Weighted = r.Value / m.Sigma
		}
		res.Residuals = append(res.Residuals, r)
	}
}

func prns(ms []position.Measurement) []int {
	out := make([]int, len(ms))
	for i, m := range ms {
		out[i] = m.PRN
	}
	return out
}

func prnsFromSolution(sol position.Solution) []int {
	out := make([]int, len(sol.Residuals))
	for i, r := range sol.Residuals {
		out[i] = r.PRN
	}
	return out
}

func findExcludedPRN(all []position.Measurement, sol position.Solution) int {
	used := map[int]bool{}
	for _, r := range sol.Residuals {
		used[r.PRN] = true
	}
	for _, m := range all {
		if !used[m.PRN] {
			return m.PRN
		}
	}
	return 0
}
