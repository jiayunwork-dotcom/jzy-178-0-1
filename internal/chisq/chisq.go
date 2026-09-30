// Package chisq provides small statistical functions required by RAIM.
// It deliberately has no third-party statistics dependency.
package chisq

import (
	"math"
	"sync"
)

var (
	quantileCache sync.Map
	lambdaCache   sync.Map
)

type quantileKey struct {
	df float64
	p  float64
}

type lambdaKey struct {
	df, x, target float64
}

// Quantile returns the p-quantile of a central chi-square distribution.
// df > 0 and 0 < p < 1 are required.
func Quantile(df, p float64) float64 {
	if df <= 0 || p <= 0 || p >= 1 || math.IsNaN(df) || math.IsNaN(p) {
		return math.NaN()
	}
	key := quantileKey{df, p}
	if v, ok := quantileCache.Load(key); ok {
		return v.(float64)
	}
	q := quantileUncached(df, p)
	quantileCache.Store(key, q)
	return q
}

func quantileUncached(df, p float64) float64 {
	// Near-normal approximation is excellent for large df; use it as a bracket.
	z := normalQuantile(p)
	guess := df + math.Sqrt(2*df)*z + (2.0/3.0)*(z*z-1.0)
	if guess <= 0 {
		guess = df * 0.01
	}
	lo, hi := math.Max(1e-12, guess/2), math.Max(guess*2, df*2+10)
	for CDF(df, lo) > p {
		lo /= 2
	}
	for CDF(df, hi) < p {
		hi *= 2
	}
	for i := 0; i < 90; i++ {
		mid := (lo + hi) / 2
		if CDF(df, mid) < p {
			lo = mid
		} else {
			hi = mid
		}
		if hi-lo < 1e-12*math.Max(1, hi) {
			break
		}
	}
	return (lo + hi) / 2
}

// NoncentralCDF returns P(X <= x) where X is chi-square with df degrees of
// freedom and noncentrality lambda. It is the Poisson mixture of central
// chi-square CDFs.
func NoncentralCDF(df, lambda, x float64) float64 {
	if x <= 0 {
		return 0
	}
	if lambda <= 0 {
		return CDF(df, x)
	}
	// Sum from the Poisson mode to keep tiny probabilities representable.
	mode := math.Max(0, math.Floor(lambda/2))
	k := int(mode)
	w := math.Exp(-lambda/2 + float64(k)*math.Log(lambda/2) - logGamma(float64(k)+1))
	sum := w * regularizedGamma(float64(k)+df/2, x/2)
	total := w

	add := func(j int, weight float64) float64 {
		if weight == 0 || weight < 1e-300 {
			return 0
		}
		total += weight
		sum += weight * regularizedGamma(float64(j)+df/2, x/2)
		return weight
	}

	wDown := w
	for j := k - 1; j >= 0; j-- {
		wDown *= 2 * float64(j+1) / lambda
		if wDown < 1e-18 && j < k-5 {
			break
		}
		add(j, wDown)
	}
	wUp := w
	for j := k + 1; ; j++ {
		wUp *= lambda / (2 * float64(j))
		if total > 1 && wUp/total < 1e-15 {
			break
		}
		add(j, wUp)
		if j-k > 10000 {
			break
		}
	}
	if total <= 0 {
		return 0
	}
	cdf := sum / total
	if cdf < 0 {
		return 0
	}
	if cdf > 1 {
		return 1
	}
	return cdf
}

// NoncentralityForCDF solves NoncentralCDF(df, lambda, x)=target. RAIM uses it
// to find the noncentrality corresponding to the required missed-detection
// probability at the detection threshold x.
func NoncentralityForCDF(df, x, target float64) float64 {
	if x <= 0 || target <= 0 || target >= 1 {
		return math.NaN()
	}
	key := lambdaKey{df, x, target}
	if v, ok := lambdaCache.Load(key); ok {
		return v.(float64)
	}
	lambda := noncentralityUncached(df, x, target)
	lambdaCache.Store(key, lambda)
	return lambda
}

func noncentralityUncached(df, x, target float64) float64 {
	lo, hi := 0.0, 1.0
	for NoncentralCDF(df, hi, x) > target {
		hi *= 2
		if hi > 1e6 {
			break
		}
	}
	for i := 0; i < 80; i++ {
		mid := (lo + hi) / 2
		if NoncentralCDF(df, mid, x) > target {
			lo = mid
		} else {
			hi = mid
		}
	}
	return (lo + hi) / 2
}

// CDF returns the central chi-square CDF.
func CDF(df, x float64) float64 {
	if x <= 0 {
		return 0
	}
	return regularizedGamma(df/2, x/2)
}

func regularizedGamma(a, x float64) float64 {
	if x <= 0 {
		return 0
	}
	// Series representation (Numerical Recipes, regularized lower incomplete P).
	if x < a+1 {
		term := 1.0 / a
		sum := term
		for n := 1; n < 300; n++ {
			term *= x / (a + float64(n))
			sum += term
			if math.Abs(term) < math.Abs(sum)*1e-15 {
				break
			}
		}
		p := sum * math.Exp(-x+a*math.Log(x)-logGamma(a))
		if p < 0 {
			return 0
		}
		return math.Min(1, p)
	}
	// Continued fraction for Q, then P=1-Q.
	b := x + 1.0 - a
	c := 1e300
	d := 1.0 / b
	h := d
	for i := 1; i <= 300; i++ {
		an := -float64(i) * (float64(i) - a)
		b += 2
		d = an*d + b
		if math.Abs(d) < 1e-300 {
			d = 1e-300
		}
		c = b + an/c
		if math.Abs(c) < 1e-300 {
			c = 1e-300
		}
		d = 1 / d
		delta := d * c
		h *= delta
		if math.Abs(delta-1) < 1e-15 {
			break
		}
	}
	q := math.Exp(-x+a*math.Log(x)-logGamma(a)) * h
	if q < 0 {
		q = 0
	}
	if q > 1 {
		return 0
	}
	return 1 - q
}

func normalQuantile(p float64) float64 {
	// Peter Acklam's rational approximation; maximum error about 1.15e-9.
	a := []float64{-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00}
	b := []float64{-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01}
	c := []float64{-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00}
	d := []float64{7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00}
	plow, phigh := 0.02425, 0.97575
	var q, r float64
	switch {
	case p < plow:
		q = math.Sqrt(-2 * math.Log(p))
		return (((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q + c[5]) / ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q + 1)
	case p <= phigh:
		q = p - 0.5
		r = q * q
		return (((((a[0]*r+a[1])*r+a[2])*r+a[3])*r+a[4])*r + a[5]) * q / (((((b[0]*r+b[1])*r+b[2])*r+b[3])*r+b[4])*r + 1)
	default:
		q = math.Sqrt(-2 * math.Log(1-p))
		return -(((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q + c[5]) / ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q + 1)
	}
}

// logGamma is the Lanczos approximation used by Numerical Recipes.
func logGamma(x float64) float64 {
	c := []float64{
		76.18009172947146, -86.50532032941677, 24.01409824083091,
		-1.231739572450155, 0.1208650973866179e-2, -0.5395239384953e-5,
	}
	y := x
	tmp := x + 5.5
	tmp -= (x + 0.5) * math.Log(tmp)
	ser := 1.000000000190015
	for _, v := range c {
		y++
		ser += v / y
	}
	return -tmp + math.Log(2.5066282746310005*ser/x)
}
