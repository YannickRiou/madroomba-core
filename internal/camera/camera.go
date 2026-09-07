// Package camera turns a Raspberry Pi camera into an MJPEG HTTP stream by
// piping the output of rpicam-vid / libcamera-vid / raspivid, and offers a
// simple frame-difference motion detector for the docking station.
package camera

import (
	"bufio"
	"bytes"
	"fmt"
	"image"
	"image/jpeg"
	"io"
	"log"
	"net/http"
	"os/exec"
	"sync"
	"time"
)

// candidateCommands lists known Pi camera CLIs, newest first, with the
// arguments that make them emit raw MJPEG on stdout.
func candidateCommands(width, height, fps int) [][]string {
	std := func(name string) []string {
		return []string{name, "-t", "0", "--codec", "mjpeg",
			"--width", fmt.Sprint(width), "--height", fmt.Sprint(height),
			"--framerate", fmt.Sprint(fps), "-n", "-o", "-"}
	}
	return [][]string{
		std("rpicam-vid"),
		std("libcamera-vid"),
		{"raspivid", "-t", "0", "-cd", "MJPEG",
			"-w", fmt.Sprint(width), "-h", fmt.Sprint(height),
			"-fps", fmt.Sprint(fps), "-n", "-o", "-"},
	}
}

// Streamer broadcasts JPEG frames read from a camera process.
type Streamer struct {
	mu     sync.Mutex
	subs   map[chan []byte]struct{}
	latest []byte
	cmd    *exec.Cmd
}

// Start launches the first available camera CLI. If command is non-empty it
// is used as-is (split on spaces) instead of autodetecting.
func Start(command []string, width, height, fps int) (*Streamer, error) {
	var candidates [][]string
	if len(command) > 0 {
		candidates = [][]string{command}
	} else {
		candidates = candidateCommands(width, height, fps)
	}
	for _, c := range candidates {
		if _, err := exec.LookPath(c[0]); err != nil {
			continue
		}
		cmd := exec.Command(c[0], c[1:]...)
		stdout, err := cmd.StdoutPipe()
		if err != nil {
			return nil, err
		}
		cmd.Stderr = io.Discard
		if err := cmd.Start(); err != nil {
			continue
		}
		s := &Streamer{subs: make(map[chan []byte]struct{}), cmd: cmd}
		go s.readLoop(stdout)
		log.Printf("camera: streaming via %s", c[0])
		return s, nil
	}
	return nil, fmt.Errorf("camera: no usable camera command found")
}

// readLoop splits the raw MJPEG byte stream into JPEG frames
// (0xFFD8 ... 0xFFD9) and fans them out.
func (s *Streamer) readLoop(r io.Reader) {
	br := bufio.NewReaderSize(r, 1<<16)
	var frame []byte
	inFrame := false
	prev := byte(0)
	for {
		b, err := br.ReadByte()
		if err != nil {
			log.Printf("camera: stream ended: %v", err)
			return
		}
		if !inFrame {
			if prev == 0xFF && b == 0xD8 {
				inFrame = true
				frame = append(frame[:0], 0xFF, 0xD8)
			}
		} else {
			frame = append(frame, b)
			if prev == 0xFF && b == 0xD9 {
				s.publish(append([]byte(nil), frame...))
				inFrame = false
			}
		}
		prev = b
	}
}

func (s *Streamer) publish(frame []byte) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.latest = frame
	for ch := range s.subs {
		select {
		case ch <- frame:
		default: // slow client: drop frame
		}
	}
}

// Latest returns the most recent frame (nil if none yet).
func (s *Streamer) Latest() []byte {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.latest
}

// Stop kills the camera process.
func (s *Streamer) Stop() {
	if s.cmd != nil && s.cmd.Process != nil {
		s.cmd.Process.Kill()
	}
}

// ServeHTTP streams frames as multipart/x-mixed-replace (MJPEG over HTTP),
// which <img> tags display natively.
func (s *Streamer) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	ch := make(chan []byte, 2)
	s.mu.Lock()
	s.subs[ch] = struct{}{}
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		delete(s.subs, ch)
		s.mu.Unlock()
	}()

	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
	for {
		select {
		case <-r.Context().Done():
			return
		case frame := <-ch:
			fmt.Fprintf(w, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(frame))
			if _, err := w.Write(frame); err != nil {
				return
			}
			fmt.Fprint(w, "\r\n")
			if f, ok := w.(http.Flusher); ok {
				f.Flush()
			}
		}
	}
}

// MotionDetector compares successive downscaled grayscale frames.
type MotionDetector struct {
	mu         sync.Mutex
	prev       []float64
	Score      float64   // last mean absolute difference, 0..255
	Threshold  float64   // score above which motion is reported
	LastMotion time.Time // zero until first detection
}

// NewMotionDetector builds a detector; threshold ~4-8 works indoors.
func NewMotionDetector(threshold float64) *MotionDetector {
	return &MotionDetector{Threshold: threshold}
}

const motionGridW, motionGridH = 32, 24

// Feed analyses a JPEG frame; returns true if motion was detected.
func (m *MotionDetector) Feed(frame []byte) (bool, error) {
	img, err := jpeg.Decode(bytes.NewReader(frame))
	if err != nil {
		return false, err
	}
	cur := downscaleGray(img)

	m.mu.Lock()
	defer m.mu.Unlock()
	if m.prev == nil {
		m.prev = cur
		return false, nil
	}
	var sum float64
	for i := range cur {
		d := cur[i] - m.prev[i]
		if d < 0 {
			d = -d
		}
		sum += d
	}
	m.prev = cur
	m.Score = sum / float64(len(cur))
	if m.Score >= m.Threshold {
		m.LastMotion = time.Now()
		return true, nil
	}
	return false, nil
}

// Status returns the last score and motion time under lock.
func (m *MotionDetector) Status() (score float64, last time.Time) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.Score, m.LastMotion
}

func downscaleGray(img image.Image) []float64 {
	b := img.Bounds()
	out := make([]float64, motionGridW*motionGridH)
	for gy := 0; gy < motionGridH; gy++ {
		for gx := 0; gx < motionGridW; gx++ {
			// Sample the center pixel of each grid cell (fast, good enough).
			px := b.Min.X + (gx*2+1)*b.Dx()/(motionGridW*2)
			py := b.Min.Y + (gy*2+1)*b.Dy()/(motionGridH*2)
			r, g, bl, _ := img.At(px, py).RGBA()
			out[gy*motionGridW+gx] = float64(299*r+587*g+114*bl) / 1000 / 257
		}
	}
	return out
}
