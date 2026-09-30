// Package service wires persistence, validation and the cross-epoch monitor to
// HTTP DTOs.
package service

import (
	"sync"

	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/persistence"
	"gnss-integrity/internal/position"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/validation"
)

const maxBatchEpochs = 3600

// Service is the application facade.
type Service struct {
	store *persistence.Store
	mu    sync.Mutex
	locks map[string]*sync.Mutex
}

// New creates a service backed by dataDir.
func New(dataDir string) (*Service, error) {
	store, err := persistence.NewStore(dataDir)
	if err != nil {
		return nil, err
	}
	return &Service{store: store, locks: map[string]*sync.Mutex{}}, nil
}

func (s *Service) sessionLock(id string) *sync.Mutex {
	s.mu.Lock()
	defer s.mu.Unlock()
	if l, ok := s.locks[id]; ok {
		return l
	}
	l := &sync.Mutex{}
	s.locks[id] = l
	return l
}

// ListProfiles delegates to the file-backed store.
func (s *Service) ListProfiles() ([]string, error) { return s.store.ListProfiles() }

// Profile returns one built-in or custom profile.
func (s *Service) Profile(name string) (profile.Profile, error) { return s.store.Profile(name) }

// CreateProfile adds a caller-defined named profile.
func (s *Service) CreateProfile(p profile.Profile) error {
	if err := p.Validate(); err != nil {
		return err
	}
	return s.store.CreateProfile(p, false)
}

// CreateSession binds a new replay session to a profile.
func (s *Service) CreateSession(profileName string) (string, *monitor.State, error) {
	p, err := s.store.Profile(profileName)
	if err != nil {
		return "", nil, err
	}
	id := persistence.NewSessionID()
	st := monitor.NewState(p)
	if err := s.store.CreateSession(id, st); err != nil {
		return "", nil, err
	}
	return id, st, nil
}

// Session returns persisted session state.
func (s *Service) Session(id string) (*monitor.State, error) { return s.store.Session(id) }

// ProcessEpoch validates, applies and persists one epoch.
func (s *Service) ProcessEpoch(id string, e monitor.Epoch) (*monitor.EpochResult, error) {
	lock := s.sessionLock(id)
	lock.Lock()
	defer lock.Unlock()
	if err := validation.ValidateEpoch(e); err != nil {
		return nil, err
	}
	st, err := s.store.Session(id)
	if err != nil {
		return nil, err
	}
	result, err := st.Process(e)
	if err != nil {
		return nil, err
	}
	if err := s.store.SaveSession(id, st); err != nil {
		return nil, err
	}
	return result, nil
}

// ProcessBatch applies at most 3600 epochs atomically.
func (s *Service) ProcessBatch(id string, epochs []monitor.Epoch) ([]*monitor.EpochResult, error) {
	lock := s.sessionLock(id)
	lock.Lock()
	defer lock.Unlock()
	if err := validation.ValidateBatch(epochs, maxBatchEpochs); err != nil {
		return nil, err
	}
	if err := validateBatchOrder(epochs); err != nil {
		return nil, err
	}
	st, err := s.store.Session(id)
	if err != nil {
		return nil, err
	}
	candidate := st.Clone()
	results := make([]*monitor.EpochResult, 0, len(epochs))
	for _, e := range epochs {
		r, err := candidate.Process(e)
		if err != nil {
			return nil, err
		}
		results = append(results, r)
	}
	if err := s.store.SaveSession(id, candidate); err != nil {
		return nil, err
	}
	return results, nil
}

func validateBatchOrder(epochs []monitor.Epoch) error {
	for i := 1; i < len(epochs); i++ {
		if epochs[i].Timestamp == epochs[i-1].Timestamp {
			return validation.FieldError{Field: "epochs", Message: "duplicate epoch timestamp within batch"}
		}
		if epochs[i].Timestamp < epochs[i-1].Timestamp {
			return validation.FieldError{Field: "epochs", Message: "epoch timestamps must be strictly increasing"}
		}
	}
	return nil
}

// Vec3DTO is the ECEF JSON representation.
type Vec3DTO struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// SatelliteDTO is one observation in an HTTP request.
type SatelliteDTO struct {
	PRN         int     `json:"prn"`
	Satellite   Vec3DTO `json:"satellite"`
	Pseudorange float64 `json:"pseudorange"`
	Sigma       float64 `json:"sigma"`
}

// EpochDTO is one submitted epoch.
type EpochDTO struct {
	Timestamp  int64          `json:"timestamp"`
	Satellites []SatelliteDTO `json:"satellites"`
	InitialPos Vec3DTO        `json:"initial_position"`
	FaultyPRN  *int           `json:"faulty_prn,omitempty"`
}

// BatchDTO submits an entire replay segment.
type BatchDTO struct {
	Epochs []EpochDTO `json:"epochs"`
}

// SessionCreateDTO is the POST body for session creation.
type SessionCreateDTO struct {
	ProfileName string `json:"profile_name"`
}

// ToMonitorEpoch converts an HTTP epoch to the monitor input.
func ToMonitorEpoch(d EpochDTO) monitor.Epoch {
	return monitor.Epoch{
		Timestamp:    d.Timestamp,
		Initial:      coords.Vec3{X: d.InitialPos.X, Y: d.InitialPos.Y, Z: d.InitialPos.Z},
		FaultyPRN:    d.FaultyPRN,
		Measurements: ToMeasurements(d.Satellites),
	}
}

// ToMeasurements converts HTTP satellite rows.
func ToMeasurements(rows []SatelliteDTO) []position.Measurement {
	ms := make([]position.Measurement, 0, len(rows))
	for _, row := range rows {
		ms = append(ms, position.Measurement{
			PRN: row.PRN,
			Satellite: coords.Vec3{
				X: row.Satellite.X,
				Y: row.Satellite.Y,
				Z: row.Satellite.Z,
			},
			Pseudorange: row.Pseudorange,
			Sigma:       row.Sigma,
		})
	}
	return ms
}
