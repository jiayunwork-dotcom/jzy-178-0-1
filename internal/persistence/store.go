// Package persistence stores named profiles and replay sessions as local JSON
// files. Writes are atomic so a restart never observes a half-written file.
package persistence

import (
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"

	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/profile"
)

var (
	// ErrNotFound is returned for unknown profiles or sessions.
	ErrNotFound = errors.New("not found")
	// ErrExists prevents overwriting an existing named profile.
	ErrExists = errors.New("already exists")
)

// Store is a file-backed profile/session repository.
type Store struct {
	root string
	mu   sync.Mutex
}

// NewStore creates and uses root/profiles and root/sessions.
func NewStore(root string) (*Store, error) {
	if strings.TrimSpace(root) == "" {
		return nil, errors.New("storage root must not be empty")
	}
	for _, d := range []string{filepath.Join(root, "profiles"), filepath.Join(root, "sessions")} {
		if err := os.MkdirAll(d, 0o755); err != nil {
			return nil, err
		}
	}
	return &Store{root: root}, nil
}

// Root returns the mounted data directory.
func (s *Store) Root() string { return s.root }

// NewSessionID returns a random URL-safe session identifier.
func NewSessionID() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		panic(err)
	}
	return hex.EncodeToString(b[:])
}

// Profile reads a custom profile, falling back to immutable built-ins.
func (s *Store) Profile(name string) (profile.Profile, error) {
	if p, ok := profile.Builtins()[name]; ok {
		return p, nil
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	var p profile.Profile
	if err := readJSON(filepath.Join(s.root, "profiles", safeName(name)+".json"), &p); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return profile.Profile{}, fmt.Errorf("%w: profile %q", ErrNotFound, name)
		}
		return profile.Profile{}, err
	}
	return p, nil
}

// CreateProfile stores a caller-defined profile. Built-in names cannot be
// overwritten.
func (s *Store) CreateProfile(p profile.Profile, overwrite bool) error {
	if err := p.Validate(); err != nil {
		return err
	}
	if _, builtin := profile.Builtins()[p.Name]; builtin && !overwrite {
		return fmt.Errorf("%w: profile %q is built in", ErrExists, p.Name)
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	path := filepath.Join(s.root, "profiles", safeName(p.Name)+".json")
	if !overwrite {
		if _, err := os.Stat(path); err == nil {
			return fmt.Errorf("%w: profile %q", ErrExists, p.Name)
		} else if !errors.Is(err, os.ErrNotExist) {
			return err
		}
	}
	p.Builtin = false
	return writeJSON(path, p)
}

// ListProfiles returns built-in and custom profile names deterministically.
func (s *Store) ListProfiles() ([]string, error) {
	names := map[string]bool{}
	for name := range profile.Builtins() {
		names[name] = true
	}
	files, err := os.ReadDir(filepath.Join(s.root, "profiles"))
	if err != nil {
		return nil, err
	}
	for _, f := range files {
		if !f.IsDir() && filepath.Ext(f.Name()) == ".json" {
			names[strings.TrimSuffix(f.Name(), ".json")] = true
		}
	}
	out := make([]string, 0, len(names))
	for name := range names {
		out = append(out, name)
	}
	sort.Strings(out)
	return out, nil
}

// CreateSession writes a new session file.
func (s *Store) CreateSession(id string, st *monitor.State) error {
	if strings.TrimSpace(id) == "" {
		return errors.New("session id must not be empty")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	path := s.sessionPath(id)
	if _, err := os.Stat(path); err == nil {
		return fmt.Errorf("%w: session %q", ErrExists, id)
	} else if !errors.Is(err, os.ErrNotExist) {
		return err
	}
	return writeJSON(path, st)
}

// Session reads a session.
func (s *Store) Session(id string) (*monitor.State, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	var st monitor.State
	if err := readJSON(s.sessionPath(id), &st); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return nil, fmt.Errorf("%w: session %q", ErrNotFound, id)
		}
		return nil, err
	}
	if st.Satellites == nil {
		st.Satellites = map[int]monitor.SatelliteState{}
	}
	return &st, nil
}

// SaveSession atomically replaces a session file.
func (s *Store) SaveSession(id string, st *monitor.State) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return writeJSON(s.sessionPath(id), st)
}

func (s *Store) sessionPath(id string) string {
	return filepath.Join(s.root, "sessions", safeName(id)+".json")
}

func safeName(name string) string {
	name = filepath.Base(filepath.Clean("/" + name))
	name = strings.ReplaceAll(name, "..", "_")
	if name == "" || name == "." || name == string(filepath.Separator) {
		return "_"
	}
	return name
}

func readJSON(path string, dst any) error {
	b, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	return json.Unmarshal(b, dst)
}

func writeJSON(path string, value any) error {
	b, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		return err
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, b, 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
