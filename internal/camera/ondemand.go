package camera

import (
	"log"
	"net/http"
	"sync"
	"time"
)

// OnDemand runs the camera only while someone watches it: the process
// starts with the first viewer and stops IdleStop after the last one
// leaves. A camera process draws a few hundred milliamps and keeps the CPU
// busy, which matters on a robot running from its own battery.
type OnDemand struct {
	Command       []string // custom command, empty to autodetect (see Start)
	Width, Height int
	FPS           int
	IdleStop      time.Duration

	mu      sync.Mutex
	s       *Streamer
	viewers int
	timer   *time.Timer
}

// acquire starts the camera if needed and counts one more viewer.
func (o *OnDemand) acquire() (*Streamer, error) {
	o.mu.Lock()
	defer o.mu.Unlock()
	if o.timer != nil {
		o.timer.Stop()
		o.timer = nil
	}
	if o.s == nil {
		s, err := Start(o.Command, o.Width, o.Height, o.FPS)
		if err != nil {
			return nil, err
		}
		o.s = s
	}
	o.viewers++
	return o.s, nil
}

// release counts one viewer less and schedules the stop after the last one.
func (o *OnDemand) release() {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.viewers--
	if o.viewers > 0 || o.s == nil {
		return
	}
	o.timer = time.AfterFunc(o.IdleStop, func() {
		o.mu.Lock()
		defer o.mu.Unlock()
		if o.viewers == 0 && o.s != nil {
			o.s.Stop()
			o.s = nil
			log.Println("camera: stopped, nobody watching")
		}
	})
}

// ServeHTTP streams the camera as MJPEG, starting it if needed (the first
// viewer waits a few seconds). It answers 503 when no camera works.
func (o *OnDemand) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	s, err := o.acquire()
	if err != nil {
		http.Error(w, err.Error(), http.StatusServiceUnavailable)
		return
	}
	defer o.release()
	s.ServeHTTP(w, r)
}
