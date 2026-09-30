// Command integrity-server runs the airborne GNSS integrity replay service.
package main

import (
	"log"
	"os"

	"github.com/labstack/echo/v4"

	"gnss-integrity/internal/httpapi"
	"gnss-integrity/internal/service"
)

func main() {
	dataDir := os.Getenv("INTEGRITY_DATA_DIR")
	if dataDir == "" {
		dataDir = "/data"
	}
	addr := os.Getenv("INTEGRITY_ADDR")
	if addr == "" {
		addr = ":8080"
	}
	svc, err := service.New(dataDir)
	if err != nil {
		log.Fatalf("initialize service: %v", err)
	}
	e := echo.New()
	e.HideBanner = true
	httpapi.New(svc).Register(e)
	log.Printf("GNSS integrity service listening on %s, data=%s", addr, dataDir)
	if err := e.Start(addr); err != nil {
		log.Fatal(err)
	}
}
