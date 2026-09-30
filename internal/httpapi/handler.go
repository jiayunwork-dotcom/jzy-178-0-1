// Package httpapi exposes the integrity replay service with Echo.
package httpapi

import (
	"errors"
	"net/http"

	"github.com/labstack/echo/v4"

	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/persistence"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/service"
	"gnss-integrity/internal/validation"
)

// Handler contains HTTP routes.
type Handler struct {
	svc *service.Service
}

// New constructs the handler.
func New(svc *service.Service) *Handler { return &Handler{svc: svc} }

// Register attaches all routes to e.
func (h *Handler) Register(e *echo.Echo) {
	e.GET("/healthz", h.health)

	api := e.Group("/api/v1")
	api.GET("/profiles", h.listProfiles)
	api.POST("/profiles", h.createProfile)
	api.GET("/profiles/:name", h.getProfile)
	api.POST("/sessions", h.createSession)
	api.GET("/sessions/:id", h.getSession)
	api.POST("/sessions/:id/epochs", h.submitEpoch)
	api.POST("/sessions/:id/batches", h.submitBatch)
}

func (h *Handler) health(c echo.Context) error {
	return c.JSON(http.StatusOK, map[string]string{"status": "ok"})
}

func (h *Handler) listProfiles(c echo.Context) error {
	names, err := h.svc.ListProfiles()
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusOK, map[string][]string{"profiles": names})
}

func (h *Handler) createProfile(c echo.Context) error {
	var p profile.Profile
	if err := c.Bind(&p); err != nil {
		return c.JSON(http.StatusBadRequest, errorBody("body", "invalid JSON: "+err.Error()))
	}
	if err := h.svc.CreateProfile(p); err != nil {
		return mapError(c, err)
	}
	saved, _ := h.svc.Profile(p.Name)
	return c.JSON(http.StatusCreated, saved)
}

func (h *Handler) getProfile(c echo.Context) error {
	p, err := h.svc.Profile(c.Param("name"))
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusOK, p)
}

func (h *Handler) createSession(c echo.Context) error {
	var req service.SessionCreateDTO
	if err := c.Bind(&req); err != nil {
		return c.JSON(http.StatusBadRequest, errorBody("body", "invalid JSON: "+err.Error()))
	}
	if req.ProfileName == "" {
		return c.JSON(http.StatusBadRequest, errorBody("profile_name", "profile_name is required"))
	}
	id, st, err := h.svc.CreateSession(req.ProfileName)
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusCreated, map[string]any{"id": id, "state": st})
}

func (h *Handler) getSession(c echo.Context) error {
	st, err := h.svc.Session(c.Param("id"))
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusOK, st)
}

func (h *Handler) submitEpoch(c echo.Context) error {
	var dto service.EpochDTO
	if err := c.Bind(&dto); err != nil {
		return c.JSON(http.StatusBadRequest, errorBody("body", "invalid JSON: "+err.Error()))
	}
	result, err := h.svc.ProcessEpoch(c.Param("id"), service.ToMonitorEpoch(dto))
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusOK, result)
}

func (h *Handler) submitBatch(c echo.Context) error {
	var dto service.BatchDTO
	if err := c.Bind(&dto); err != nil {
		return c.JSON(http.StatusBadRequest, errorBody("body", "invalid JSON: "+err.Error()))
	}
	if len(dto.Epochs) == 0 {
		return c.JSON(http.StatusBadRequest, errorBody("epochs", "must contain at least one epoch"))
	}
	epochs := make([]monitor.Epoch, len(dto.Epochs))
	for i, e := range dto.Epochs {
		epochs[i] = service.ToMonitorEpoch(e)
	}
	results, err := h.svc.ProcessBatch(c.Param("id"), epochs)
	if err != nil {
		return mapError(c, err)
	}
	return c.JSON(http.StatusOK, map[string]any{"results": results})
}

type apiError struct {
	Error struct {
		Field   string `json:"field,omitempty"`
		Message string `json:"message"`
	} `json:"error"`
}

func errorBody(field, message string) apiError {
	var e apiError
	e.Error.Field = field
	e.Error.Message = message
	return e
}

func mapError(c echo.Context, err error) error {
	var fieldErr validation.FieldError
	if errors.As(err, &fieldErr) {
		return c.JSON(http.StatusBadRequest, errorBody(fieldErr.Field, fieldErr.Message))
	}
	switch {
	case errors.Is(err, persistence.ErrNotFound):
		return c.JSON(http.StatusNotFound, errorBody("", err.Error()))
	case errors.Is(err, persistence.ErrExists), errors.Is(err, monitor.ErrDuplicateTimestamp):
		return c.JSON(http.StatusConflict, errorBody("", err.Error()))
	case errors.Is(err, monitor.ErrTimestampBeforeLast):
		return c.JSON(http.StatusUnprocessableEntity, errorBody("timestamp", err.Error()))
	default:
		return c.JSON(http.StatusInternalServerError, errorBody("", err.Error()))
	}
}
