package chisq

import (
	"math"
	"testing"
)

func TestCDFKnownPoints(t *testing.T) {
	cases := []struct{ df, x, want float64 }{
		{1, 0.4549, 0.5},
		{2, 1.3863, 0.5},
		{4, 3.3567, 0.5},
	}
	for _, c := range cases {
		got := CDF(c.df, c.x)
		if math.Abs(got-c.want) > 1e-3 {
			t.Fatalf("CDF(%v,%v)=%v, want %v", c.df, c.x, got, c.want)
		}
	}
}

func TestQuantileInvertsCDF(t *testing.T) {
	for _, p := range []float64{0.5, 0.95, 0.99, 0.99999} {
		q := Quantile(5, p)
		got := CDF(5, q)
		if math.Abs(got-p) > 1e-9 {
			t.Fatalf("df=5 p=%v q=%v CDF(q)=%v", p, q, got)
		}
	}
	// Standard table values: chi-square df=4, upper 5% is 9.488; df=5 upper
	// 0.001% quantile from this implementation/inverse CDF is 30.858.
	if got := Quantile(4, 0.95); math.Abs(got-9.488) > 0.01 {
		t.Fatalf("Quantile(4,.95)=%v", got)
	}
	if got := Quantile(5, 0.99999); math.Abs(got-30.858) > 0.02 {
		t.Fatalf("Quantile(5,.99999)=%v", got)
	}
}

func TestSmallerFalseAlarmYieldsLargerQuantile(t *testing.T) {
	if Quantile(6, 0.99999) <= Quantile(6, 0.999) {
		t.Fatal("higher confidence quantile must be larger")
	}
}

func TestNoncentralCDFMovesRightWithLambda(t *testing.T) {
	x := 20.0
	a := NoncentralCDF(4, 0, x)
	b := NoncentralCDF(4, 20, x)
	if !(a > b) || a <= 0 || a > 1 {
		t.Fatalf("noncentral CDF must decrease with lambda: %v %v", a, b)
	}
	lambda := NoncentralityForCDF(4, Quantile(4, 0.99999), 0.001)
	if lambda <= 0 {
		t.Fatalf("lambda=%v", lambda)
	}
	got := NoncentralCDF(4, lambda, Quantile(4, 0.99999))
	if math.Abs(got-0.001) > 1e-6 {
		t.Fatalf("inverted noncentral CDF=%v", got)
	}
}
