package profile

import (
	"strings"
	"testing"
)

func TestBuiltins(t *testing.T) {
	builtins := Builtins()
	for _, name := range []string{"enroute", "terminal", "npa"} {
		p, ok := builtins[name]
		if !ok {
			t.Fatalf("missing built-in profile %q", name)
		}
		if err := p.Validate(); err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if p.RecoveryEpochs <= 0 || p.AlarmEpochs <= 0 || p.ClearAlarmEpochs <= 0 {
			t.Fatalf("persistence must be positive in %s", name)
		}
	}
	if builtins["enroute"].HorizontalAlertLimit <= builtins["npa"].HorizontalAlertLimit {
		t.Fatal("NPA HAL should be tighter than en route HAL")
	}
}

func TestValidateReportsFieldNames(t *testing.T) {
	p := Builtins()["terminal"]
	p.Name = "bad name!"
	p.ProbabilityFalseAlarm = 0
	p.RecoveryEpochs = 0
	err := p.Validate()
	if err == nil {
		t.Fatal("expected validation failure")
	}
	msg := err.Error()
	for _, want := range []string{"name", "probability_false_alarm", "recovery_epochs"} {
		if !strings.Contains(msg, want) {
			t.Fatalf("error %q does not mention %s", msg, want)
		}
	}
}
