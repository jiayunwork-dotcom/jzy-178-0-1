package persistence

import (
	"testing"

	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/profile"
)

func TestProfileAndSessionRoundTrip(t *testing.T) {
	store, err := NewStore(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	p := profile.Builtins()["npa"]
	got, err := store.Profile("npa")
	if err != nil || got.HorizontalAlertLimit != p.HorizontalAlertLimit {
		t.Fatalf("builtin: %v %+v", err, got)
	}
	custom := p
	custom.Name = "ground-test"
	custom.HorizontalAlertLimit = 100
	if err := store.CreateProfile(custom, false); err != nil {
		t.Fatal(err)
	}
	if err := store.CreateProfile(custom, false); err == nil {
		t.Fatal("expected duplicate profile error")
	}
	loaded, err := store.Profile("ground-test")
	if err != nil || loaded.HorizontalAlertLimit != 100 {
		t.Fatalf("custom profile: %v %+v", err, loaded)
	}

	st := monitor.NewState(custom)
	if err := store.CreateSession("abc123", st); err != nil {
		t.Fatal(err)
	}
	st2, err := store.Session("abc123")
	if err != nil || st2.Profile.Name != "ground-test" {
		t.Fatalf("session: %v %+v", err, st2)
	}
}

func TestMissingProfileAndSession(t *testing.T) {
	store, _ := NewStore(t.TempDir())
	if _, err := store.Profile("missing"); err == nil {
		t.Fatal("expected missing profile")
	}
	if _, err := store.Session("missing"); err == nil {
		t.Fatal("expected missing session")
	}
}
