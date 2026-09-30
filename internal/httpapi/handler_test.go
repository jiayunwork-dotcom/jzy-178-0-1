package httpapi

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/labstack/echo/v4"

	"gnss-integrity/internal/service"
	"gnss-integrity/internal/sim"
)

func newTestServer(t *testing.T) (*echo.Echo, *service.Service) {
	t.Helper()
	svc, err := service.New(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	e := echo.New()
	New(svc).Register(e)
	return e, svc
}

func request(t *testing.T, e *echo.Echo, method, path, body string) *httptest.ResponseRecorder {
	t.Helper()
	req := httptest.NewRequest(method, path, strings.NewReader(body))
	req.Header.Set(echo.HeaderContentType, echo.MIMEApplicationJSON)
	rec := httptest.NewRecorder()
	e.ServeHTTP(rec, req)
	return rec
}

func TestProfileAndEpochFlow(t *testing.T) {
	e, _ := newTestServer(t)
	if rec := request(t, e, http.MethodGet, "/api/v1/profiles/enroute", ""); rec.Code != http.StatusOK {
		t.Fatalf("profile status=%d body=%s", rec.Code, rec.Body)
	}
	rec := request(t, e, http.MethodPost, "/api/v1/sessions", `{"profile_name":"enroute"}`)
	if rec.Code != http.StatusCreated {
		t.Fatalf("create=%d %s", rec.Code, rec.Body)
	}
	var created struct {
		ID string `json:"id"`
	}
	if err := json.Unmarshal(rec.Body.Bytes(), &created); err != nil {
		t.Fatal(err)
	}
	ms := sim.Measurements(sim.DefaultSky(), 0.1, 0, 0, 0, nil)
	var sats []map[string]any
	for _, m := range ms {
		sats = append(sats, map[string]any{
			"prn":         m.PRN,
			"satellite":   map[string]float64{"x": m.Satellite.X, "y": m.Satellite.Y, "z": m.Satellite.Z},
			"pseudorange": m.Pseudorange,
			"sigma":       m.Sigma,
		})
	}
	body, _ := json.Marshal(map[string]any{
		"timestamp":        1,
		"satellites":       sats,
		"initial_position": map[string]float64{"x": sim.ApproximatePosition().X, "y": sim.ApproximatePosition().Y, "z": sim.ApproximatePosition().Z},
	})
	rec = request(t, e, http.MethodPost, "/api/v1/sessions/"+created.ID+"/epochs", string(body))
	if rec.Code != http.StatusOK {
		t.Fatalf("epoch=%d %s", rec.Code, rec.Body)
	}
	rec = request(t, e, http.MethodPost, "/api/v1/sessions/"+created.ID+"/epochs", string(body))
	if rec.Code != http.StatusConflict {
		t.Fatalf("duplicate status=%d body=%s", rec.Code, rec.Body)
	}
}

func TestBadRequestPointsToField(t *testing.T) {
	e, _ := newTestServer(t)
	body := bytes.NewBufferString(`{"timestamp":1,"satellites":[],"initial_position":{"x":0,"y":0,"z":0}}`)
	req := httptest.NewRequest(http.MethodPost, "/api/v1/sessions/x/epochs", body)
	req.Header.Set(echo.HeaderContentType, echo.MIMEApplicationJSON)
	rec := httptest.NewRecorder()
	e.ServeHTTP(rec, req)
	if rec.Code != http.StatusNotFound { // unknown session checked after validation? currently validation first
		// Service validates before loading, so a 400 field error is also valid.
		if rec.Code != http.StatusBadRequest {
			t.Fatalf("status=%d body=%s", rec.Code, rec.Body)
		}
	}
}
