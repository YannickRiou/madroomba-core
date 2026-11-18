package main

import (
	"errors"
	"log"
	"math"
	"path/filepath"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/sim"
)

// scanSource abstracts the real lidar and the simulator.
type scanSource interface {
	ReadScan() (*lidar.Scan, error)
}

// simScanner paces simulated scans at the real sensor's ~5 Hz.
type simScanner struct{ r *sim.Robot }

// ReadScan returns a simulated scan every 200 ms.
func (s simScanner) ReadScan() (*lidar.Scan, error) {
	time.Sleep(200 * time.Millisecond)
	return s.r.Scan(), nil
}

// openLidar keeps trying until the lidar opens and starts. At boot the daemon
// can start before udev has given /dev/ttyUSB0 to the dialout group, so a
// single attempt fails with "permission denied" and the lidar stays off.
func openLidar(port string) *lidar.Device {
	for warned := false; ; time.Sleep(5 * time.Second) {
		name, err := lidarDevice(port)
		if err == nil {
			var dev *lidar.Device
			if dev, err = lidar.Open(name); err == nil {
				if err = dev.Start(); err == nil {
					log.Printf("madroombad: lidar started on %s", name)
					return dev
				}
				dev.Close()
			}
		}
		if !warned {
			log.Printf("madroombad: lidar: %v (retrying every 5s)", err)
			warned = true
		}
	}
}

// lidarDevice resolves the -lidar-port flag. "auto" looks for a USB serial
// adapter at every attempt: after a USB disconnect the adapter can come back
// as ttyUSB1 while the old ttyUSB0 is still held open.
func lidarDevice(port string) (string, error) {
	if port != "auto" {
		return port, nil
	}
	for _, pattern := range []string{"/dev/ttyUSB*", "/dev/ttyACM*"} {
		if m, _ := filepath.Glob(pattern); len(m) > 0 {
			return m[0], nil
		}
	}
	return "", errors.New("no USB serial adapter found (/dev/ttyUSB*, /dev/ttyACM*)")
}

// normalizeScan converts a raw sensor-frame scan to the robot frame
// (0° = front, counter-clockwise positive).
func normalizeScan(raw *lidar.Scan, yawDeg float64, invert bool) *lidar.Scan {
	out := &lidar.Scan{RPM: raw.RPM}
	off := int(math.Round(yawDeg))
	for deg := 0; deg < 360; deg++ {
		src := deg
		if invert {
			src = (360 - deg) % 360
		}
		src = ((src+off)%360 + 360) % 360
		out.Ranges[deg] = raw.Ranges[src]
		out.Intensities[deg] = raw.Intensities[src]
	}
	return out
}
