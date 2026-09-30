// Package validation performs request-level checks and keeps field names in
// every rejected-input error.
package validation

import (
	"fmt"
	"math"

	"gnss-integrity/internal/coords"
	"gnss-integrity/internal/monitor"
)

// FieldError points at a specific request field.
type FieldError struct {
	Field   string `json:"field"`
	Message string `json:"message"`
}

func (e FieldError) Error() string { return e.Field + ": " + e.Message }

// ValidateEpoch validates scalar fields and static geometry. Cross-epoch
// timestamp checks belong to monitor.
func ValidateEpoch(e monitor.Epoch) error {
	if e.Timestamp <= 0 {
		return FieldError{Field: "timestamp", Message: "must be a positive epoch timestamp"}
	}
	if len(e.Measurements) < 4 {
		return FieldError{Field: "satellites", Message: fmt.Sprintf("at least four visible satellites are required, got %d", len(e.Measurements))}
	}
	if coords.NearEarthCenter(e.Initial) {
		return FieldError{Field: "initial_position", Message: "must be an ECEF position at least 1 km from Earth's center and contain no NaN/Infinity"}
	}
	seen := map[int]bool{}
	dir := make([][3]float64, 0, len(e.Measurements))
	for i, m := range e.Measurements {
		field := fmt.Sprintf("satellites[%d]", i)
		if m.PRN <= 0 {
			return FieldError{Field: field + ".prn", Message: "must be a positive satellite number"}
		}
		if seen[m.PRN] {
			return FieldError{Field: field + ".prn", Message: fmt.Sprintf("satellite number %d appears more than once", m.PRN)}
		}
		seen[m.PRN] = true
		if !m.Satellite.Finite() {
			return FieldError{Field: field + ".satellite", Message: "x, y and z must all be finite numbers"}
		}
		if math.IsNaN(m.Pseudorange) || math.IsInf(m.Pseudorange, 0) {
			return FieldError{Field: field + ".pseudorange", Message: "must be a finite number"}
		}
		if m.Pseudorange <= 0 {
			return FieldError{Field: field + ".pseudorange", Message: fmt.Sprintf("must be positive, got %v", m.Pseudorange)}
		}
		if math.IsNaN(m.Sigma) || math.IsInf(m.Sigma, 0) || m.Sigma <= 0 {
			return FieldError{Field: field + ".sigma", Message: fmt.Sprintf("pseudorange error standard deviation must be positive and finite, got %v", m.Sigma)}
		}
		dx := m.Satellite.X - e.Initial.X
		dy := m.Satellite.Y - e.Initial.Y
		dz := m.Satellite.Z - e.Initial.Z
		r := math.Sqrt(dx*dx + dy*dy + dz*dz)
		if r < 1 || math.IsNaN(r) {
			return FieldError{Field: field + ".satellite", Message: "satellite coincides with receiver or geometry range is invalid"}
		}
		dir = append(dir, [3]float64{dx / r, dy / r, dz / r})
	}
	if rankLessThan4(dir) {
		return FieldError{Field: "satellites", Message: "line-of-sight geometry is singular (satellites are coplanar or otherwise do not span 3D plus clock)"}
	}
	return nil
}

func rankLessThan4(rows [][3]float64) bool {
	// Include the all-ones clock column, then Gaussian-eliminate G'G.
	var a [4][4]float64
	for _, d := range rows {
		g := [4]float64{-d[0], -d[1], -d[2], 1}
		for i := 0; i < 4; i++ {
			for j := 0; j < 4; j++ {
				a[i][j] += g[i] * g[j]
			}
		}
	}
	scale := 0.0
	for i := range a {
		for j := range a {
			scale = math.Max(scale, math.Abs(a[i][j]))
		}
	}
	if scale == 0 {
		return true
	}
	tol := scale * 1e-11
	for col := 0; col < 4; col++ {
		pivot := col
		best := math.Abs(a[col][col])
		for row := col + 1; row < 4; row++ {
			if math.Abs(a[row][col]) > best {
				best, pivot = math.Abs(a[row][col]), row
			}
		}
		if best < tol {
			return true
		}
		a[col], a[pivot] = a[pivot], a[col]
		p := a[col][col]
		for j := col; j < 4; j++ {
			a[col][j] /= p
		}
		for row := col + 1; row < 4; row++ {
			f := a[row][col]
			for j := col; j < 4; j++ {
				a[row][j] -= f * a[col][j]
			}
		}
	}
	return false
}

// ValidateBatch validates an entire batch before any state is advanced.
func ValidateBatch(epochs []monitor.Epoch, maxBatch int) error {
	if len(epochs) == 0 {
		return FieldError{Field: "epochs", Message: "must contain at least one epoch"}
	}
	if len(epochs) > maxBatch {
		return FieldError{Field: "epochs", Message: fmt.Sprintf("one batch may contain at most %d epochs, got %d", maxBatch, len(epochs))}
	}
	for i := range epochs {
		if err := ValidateEpoch(epochs[i]); err != nil {
			return fmt.Errorf("epochs[%d].%w", i, err)
		}
	}
	return nil
}
