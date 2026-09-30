// Package position implements weighted single-point least-squares GNSS
// positioning and the linear-algebra quantities used by RAIM.
package position

import (
	"fmt"
	"math"

	"gnss-integrity/internal/coords"
)

const (
	// MaxIterations is the single-epoch linearization limit.
	MaxIterations = 10
	convergenceM  = 0.001
)

// Measurement is one pseudorange observation at an epoch.
type Measurement struct {
	PRN         int         `json:"prn"`
	Satellite   coords.Vec3 `json:"satellite"`
	Pseudorange float64     `json:"pseudorange"`
	Sigma       float64     `json:"sigma"`
}

// Residual holds post-fit information for one satellite.
type Residual struct {
	PRN      int     `json:"prn"`
	Value    float64 `json:"value"`
	Weighted float64 `json:"weighted"`
	Sigma    float64 `json:"sigma"`
	Used     bool    `json:"used"`
	Excluded bool    `json:"excluded"`
	Isolated bool    `json:"isolated"`
}

// Solution is a weighted least-squares position solution.
type Solution struct {
	Position    coords.Vec3     `json:"position"`
	LLA         coords.LLA      `json:"lla"`
	ClockBias   float64         `json:"clock_bias"`
	Iterations  int             `json:"iterations"`
	Converged   bool            `json:"converged"`
	Residuals   []Residual      `json:"residuals"`
	SSE         float64         `json:"sse"`
	HDOP        float64         `json:"hdop"`
	HSlopes     map[int]float64 `json:"h_slopes"`
	MaxSlopePRN int             `json:"max_slope_prn"`
	MaxSlope    float64         `json:"max_slope"`
}

// GeometryError identifies an unusable design matrix rather than a bad scalar.
type GeometryError struct{ Message string }

func (e *GeometryError) Error() string { return e.Message }

// Solve performs iterative weighted least squares. Measurements are assumed to
// already be validated and may be an active subset of the visible constellation.
func Solve(measurements []Measurement, initial coords.Vec3) (*Solution, error) {
	n := len(measurements)
	if n < 4 {
		return nil, &GeometryError{Message: "at least four measurements are required"}
	}

	pos := initial
	clock := 0.0
	iterations := 0
	converged := false

	for iterations = 1; iterations <= MaxIterations; iterations++ {
		g, y, weights, err := design(measurements, pos, clock)
		if err != nil {
			return nil, err
		}
		dx, err := weightedNormalSolve(g, y, weights)
		if err != nil {
			return nil, err
		}
		pos.X += dx[0]
		pos.Y += dx[1]
		pos.Z += dx[2]
		clock += dx[3]
		delta := math.Sqrt(dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2])
		if delta < convergenceM {
			converged = true
			break
		}
	}

	g, _, weights, err := design(measurements, pos, clock)
	if err != nil {
		return nil, err
	}
	cov, err := inverse4(normalMatrix(g, weights))
	if err != nil {
		return nil, err
	}
	unweightedCov, err := inverse4(normalMatrix(g, nil))
	if err != nil {
		return nil, err
	}

	residuals := make([]Residual, n)
	sse := 0.0
	for i, m := range measurements {
		r := predictedMinusObs(m, pos, clock)
		wr := r / m.Sigma
		residuals[i] = Residual{PRN: m.PRN, Value: r, Weighted: wr, Sigma: m.Sigma, Used: true}
		sse += wr * wr
	}

	lat := coords.ECEFToLLA(pos).Latitude * math.Pi / 180
	lon := coords.ECEFToLLA(pos).Longitude * math.Pi / 180
	east, north, _ := coords.LocalBasis(lat, lon)
	slopes := make(map[int]float64, n)
	maxSlope := 0.0
	maxPRN := measurements[0].PRN
	// H=G P G'W. Column i of (I-H) maps a raw bias in measurement i to its
	// post-fit residual vector.
	for i := range measurements {
		// Column i of the projection H = G P G'W. A unit raw-range bias in
		// satellite i becomes (I-H)_*i in the post-fit residual vector, while
		// the solution shift is P G_i' w_i.
		var pg [4]float64
		for k := 0; k < 4; k++ {
			for q := 0; q < 4; q++ {
				pg[k] += cov[k][q] * g[i][q] * weights[i]
			}
		}
		var residualNorm float64
		for j := range measurements {
			hji := 0.0
			for k := 0; k < 4; k++ {
				hji += g[j][k] * pg[k]
			}
			z := 0.0
			if i == j {
				z = 1
			}
			z -= hji
			residualNorm += z * z * weights[j]
		}
		var solutionShift [4]float64
		for k := 0; k < 4; k++ {
			solutionShift[k] = pg[k] // P G_i' w_i already includes w_i
		}
		dxEast := solutionShift[0]*east.X + solutionShift[1]*east.Y + solutionShift[2]*east.Z
		dxNorth := solutionShift[0]*north.X + solutionShift[1]*north.Y + solutionShift[2]*north.Z
		hdelta := math.Hypot(dxEast, dxNorth)
		slope := 0.0
		if residualNorm > 0 {
			slope = hdelta / math.Sqrt(residualNorm)
		}
		slopes[measurements[i].PRN] = slope
		if slope > maxSlope {
			maxSlope, maxPRN = slope, measurements[i].PRN
		}
	}

	hdop := math.Sqrt(unweightedCov[0][0] + unweightedCov[1][1] + unweightedCov[2][2])
	// The expression above is ECEF-position DOP; rotate to local horizontal.
	var he, hn float64
	for i := 0; i < 3; i++ {
		var ei, ni float64
		if i == 0 {
			ei, ni = east.X, north.X
		} else if i == 1 {
			ei, ni = east.Y, north.Y
		} else {
			ei, ni = east.Z, north.Z
		}
		for j := 0; j < 3; j++ {
			var ej, nj float64
			if j == 0 {
				ej, nj = east.X, north.X
			} else if j == 1 {
				ej, nj = east.Y, north.Y
			} else if j == 2 {
				ej, nj = east.Z, north.Z
			}
			if i == j {
				he += ei * ej * unweightedCov[i][j]
				hn += ni * nj * unweightedCov[i][j]
			} else if i < j {
				v := unweightedCov[i][j] + unweightedCov[j][i]
				he += ei * ej * v
				hn += ni * nj * v
			}
		}
	}
	hdop = math.Sqrt(math.Max(0, he+hn))

	return &Solution{
		Position:    pos,
		LLA:         coords.ECEFToLLA(pos),
		ClockBias:   clock,
		Iterations:  iterations,
		Converged:   converged,
		Residuals:   residuals,
		SSE:         sse,
		HDOP:        hdop,
		HSlopes:     slopes,
		MaxSlopePRN: maxPRN,
		MaxSlope:    maxSlope,
	}, nil
}

func predictedMinusObs(m Measurement, pos coords.Vec3, clock float64) float64 {
	dx := m.Satellite.X - pos.X
	dy := m.Satellite.Y - pos.Y
	dz := m.Satellite.Z - pos.Z
	return math.Sqrt(dx*dx+dy*dy+dz*dz) + clock - m.Pseudorange
}

func design(ms []Measurement, pos coords.Vec3, clock float64) ([][4]float64, []float64, []float64, error) {
	n := len(ms)
	g := make([][4]float64, n)
	y := make([]float64, n)
	w := make([]float64, n)
	for i, m := range ms {
		dx := m.Satellite.X - pos.X
		dy := m.Satellite.Y - pos.Y
		dz := m.Satellite.Z - pos.Z
		r := math.Sqrt(dx*dx + dy*dy + dz*dz)
		if !math.IsInf(r, 0) && r < 1 {
			return nil, nil, nil, &GeometryError{Message: fmt.Sprintf("satellite %d geometry range is zero", m.PRN)}
		}
		// y is observed minus predicted; G x has opposite line-of-sight sign.
		g[i] = [4]float64{-dx / r, -dy / r, -dz / r, 1}
		y[i] = m.Pseudorange - (r + clock)
		w[i] = 1 / (m.Sigma * m.Sigma)
	}
	return g, y, w, nil
}

func weightedNormalSolve(g [][4]float64, y, w []float64) ([4]float64, error) {
	a := normalMatrix(g, w)
	var rhs [4]float64
	for i := range g {
		for r := 0; r < 4; r++ {
			rhs[r] += g[i][r] * w[i] * y[i]
		}
	}
	return solve4(a, rhs)
}

func normalMatrix(g [][4]float64, w []float64) [4][4]float64 {
	var a [4][4]float64
	for i := range g {
		wi := 1.0
		if w != nil {
			wi = w[i]
		}
		for r := 0; r < 4; r++ {
			for c := 0; c < 4; c++ {
				a[r][c] += wi * g[i][r] * g[i][c]
			}
		}
	}
	return a
}

func solve4(a [4][4]float64, b [4]float64) ([4]float64, error) {
	inv, err := inverse4(a)
	if err != nil {
		return [4]float64{}, err
	}
	var x [4]float64
	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			x[r] += inv[r][c] * b[c]
		}
	}
	return x, nil
}

func inverse4(src [4][4]float64) ([4][4]float64, error) {
	var a [4][8]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			a[i][j] = src[i][j]
		}
		a[i][i+4] = 1
	}
	scale := 0.0
	for _, row := range src {
		for _, v := range row {
			scale = math.Max(scale, math.Abs(v))
		}
	}
	if scale == 0 || math.IsNaN(scale) {
		return [4][4]float64{}, &GeometryError{Message: "satellite geometry matrix is singular"}
	}
	tol := scale * 1e-12
	for col := 0; col < 4; col++ {
		pivot := col
		best := math.Abs(a[col][col])
		for row := col + 1; row < 4; row++ {
			if v := math.Abs(a[row][col]); v > best {
				best, pivot = v, row
			}
		}
		if best < tol {
			return [4][4]float64{}, &GeometryError{Message: "satellite geometry matrix is singular or nearly singular"}
		}
		if pivot != col {
			a[col], a[pivot] = a[pivot], a[col]
		}
		p := a[col][col]
		for j := 0; j < 8; j++ {
			a[col][j] /= p
		}
		for row := 0; row < 4; row++ {
			if row == col {
				continue
			}
			f := a[row][col]
			for j := 0; j < 8; j++ {
				a[row][j] -= f * a[col][j]
			}
		}
	}
	var inv [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			inv[i][j] = a[i][j+4]
		}
	}
	return inv, nil
}
