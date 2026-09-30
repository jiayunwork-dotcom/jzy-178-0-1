// Package profile defines named integrity operating profiles.
package profile

import (
	"errors"
	"fmt"
	"regexp"
)

// Profile contains all statistical thresholds and persistence parameters for
// one phase of flight / validation run.
type Profile struct {
	Name                    string  `json:"name"`
	Description             string  `json:"description"`
	ProbabilityFalseAlarm   float64 `json:"probability_false_alarm"`
	ProbabilityMissedDetect float64 `json:"probability_missed_detection"`
	HorizontalAlertLimit    float64 `json:"horizontal_alert_limit"`
	// RecoveryEpochs is the number of consecutive acceptable epochs required
	// before an isolated satellite is returned to the active solution.
	RecoveryEpochs int `json:"recovery_epochs"`
	// RecoveryZLimit is the maximum absolute normalized isolated-satellite
	// post-fit residual accepted during recovery.
	RecoveryZLimit float64 `json:"recovery_z_limit"`
	// AlarmEpochs is the number of consecutive unsafe snapshots required to
	// raise an alarm; ClearAlarmEpochs is the number required to remove one.
	AlarmEpochs      int    `json:"alarm_epochs"`
	ClearAlarmEpochs int    `json:"clear_alarm_epochs"`
	Source           string `json:"source,omitempty"`
	Builtin          bool   `json:"builtin"`
}

var nameRE = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]{0,31}$`)

// Validate checks every field independently enough for useful API errors.
func (p Profile) Validate() error {
	var problems []error
	if !nameRE.MatchString(p.Name) {
		problems = append(problems, errors.New("name must contain 1-32 letters, digits, '_', '.' or '-' and not start with punctuation"))
	}
	if !(p.ProbabilityFalseAlarm > 0 && p.ProbabilityFalseAlarm < 1) {
		problems = append(problems, fmt.Errorf("probability_false_alarm must be in (0,1), got %v", p.ProbabilityFalseAlarm))
	}
	if !(p.ProbabilityMissedDetect > 0 && p.ProbabilityMissedDetect < 1) {
		problems = append(problems, fmt.Errorf("probability_missed_detection must be in (0,1), got %v", p.ProbabilityMissedDetect))
	}
	if !(p.HorizontalAlertLimit > 0 && p.HorizontalAlertLimit < 1e6) {
		problems = append(problems, fmt.Errorf("horizontal_alert_limit must be positive and less than 1,000,000 m, got %v", p.HorizontalAlertLimit))
	}
	if p.RecoveryEpochs < 1 || p.RecoveryEpochs > 3600 {
		problems = append(problems, fmt.Errorf("recovery_epochs must be in [1,3600], got %d", p.RecoveryEpochs))
	}
	if !(p.RecoveryZLimit > 0 && p.RecoveryZLimit < 100) {
		problems = append(problems, fmt.Errorf("recovery_z_limit must be in (0,100), got %v", p.RecoveryZLimit))
	}
	if p.AlarmEpochs < 1 || p.AlarmEpochs > 3600 {
		problems = append(problems, fmt.Errorf("alarm_epochs must be in [1,3600], got %d", p.AlarmEpochs))
	}
	if p.ClearAlarmEpochs < 1 || p.ClearAlarmEpochs > 3600 {
		problems = append(problems, fmt.Errorf("clear_alarm_epochs must be in [1,3600], got %d", p.ClearAlarmEpochs))
	}
	return errors.Join(problems...)
}

// Builtins returns the three built-in aviation phases.
//
// The statistical requirements and HALs follow the widely quoted table in
// RTCA DO-229D Appendix Section 2.1 ("GPS/WAAS Equipment Functional
// Requirements", oceanic/en route, terminal, and NPA RAIM screening values),
// summarized by the FAA TSO-C129/C129A guidance and Walter & Enge,
// "Understanding GPS Principles and Applications", Chapter 5: "GPS RAIM:
// Detection of Failures" (1996). The common RAIM screening table uses
// Pfa=1/100000, Pmd=1/1000 and HAL 2.0 nmi (en route), 1.0 nmi (terminal) and
// 0.3 nmi (NPA). Persistence values are deliberately implementation tunables,
// not standards values; 3 epochs at a typical 1 Hz output gives a conservative
// short delay while filtering one-epoch multipath spikes.
func Builtins() map[string]Profile {
	return map[string]Profile{
		"enroute": {
			Name:                    "enroute",
			Description:             "En route / oceanic and remote RAIM screening",
			ProbabilityFalseAlarm:   1e-5,
			ProbabilityMissedDetect: 1e-3,
			HorizontalAlertLimit:    3704,
			RecoveryEpochs:          3,
			RecoveryZLimit:          4,
			AlarmEpochs:             3,
			ClearAlarmEpochs:        3,
			Source:                  "RTCA DO-229D RAIM screening table; Walter & Enge (1996)",
			Builtin:                 true,
		},
		"terminal": {
			Name:                    "terminal",
			Description:             "Terminal area RAIM screening",
			ProbabilityFalseAlarm:   1e-5,
			ProbabilityMissedDetect: 1e-3,
			HorizontalAlertLimit:    1852,
			RecoveryEpochs:          3,
			RecoveryZLimit:          4,
			AlarmEpochs:             3,
			ClearAlarmEpochs:        3,
			Source:                  "RTCA DO-229D RAIM screening table; Walter & Enge (1996)",
			Builtin:                 true,
		},
		"npa": {
			Name:                    "npa",
			Description:             "Non-precision approach RAIM screening",
			ProbabilityFalseAlarm:   1e-5,
			ProbabilityMissedDetect: 1e-3,
			HorizontalAlertLimit:    556,
			RecoveryEpochs:          3,
			RecoveryZLimit:          4,
			AlarmEpochs:             3,
			ClearAlarmEpochs:        3,
			Source:                  "RTCA DO-229D RAIM screening table; Walter & Enge (1996)",
			Builtin:                 true,
		},
	}
}
