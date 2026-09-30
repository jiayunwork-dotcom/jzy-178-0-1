# syntax=docker/dockerfile:1

# Build stage: pinned to the required official Go 1.22 Alpine image.
FROM golang:1.22-alpine AS build
WORKDIR /src

# Cache module downloads independently of source changes.
COPY go.mod go.sum ./
RUN go mod download

COPY . .
RUN CGO_ENABLED=0 GOOS=linux go build -trimpath -ldflags='-s -w' \
    -o /out/integrity-server ./cmd/integrity-server

# Runtime stage: minimal Alpine with a writable mounted data directory.
FROM alpine:3.20
RUN addgroup -S integrity && adduser -S -G integrity -h /data integrity
WORKDIR /app
COPY --from=build /out/integrity-server /app/integrity-server
RUN mkdir -p /data/profiles /data/sessions && chown -R integrity:integrity /data /app
USER integrity

ENV INTEGRITY_ADDR=:8080 \
    INTEGRITY_DATA_DIR=/data
EXPOSE 8080
VOLUME ["/data"]
ENTRYPOINT ["/app/integrity-server"]
