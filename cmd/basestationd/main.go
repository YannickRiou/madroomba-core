// basestationd runs on the Raspberry Pi Zero attached to the Roomba's
// charging dock. It streams the dock camera over MJPEG and watches the
// dock area with a simple frame-difference motion detector, so you can:
//   - visually check that the robot docked correctly,
//   - get a "last activity in front of the dock" timestamp,
//   - later: give the robot an absolute pose fix when it is in view
//     (ArUco marker on the robot's top cover — see README roadmap).
package main

import (
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"strings"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/camera"
)

var (
	listen    = flag.String("listen", ":8081", "HTTP listen address")
	cameraCmd = flag.String("camera", "auto", "camera command: 'auto' or a custom command line emitting MJPEG on stdout")
	threshold = flag.Float64("motion-threshold", 5.0, "mean gray-level difference (0-255) that counts as motion")
)

const page = `<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8"><title>MadRoomba — Base</title>
<style>
:root{--color-navy:#113B54;--color-chartreuse:#DBE64C;--color-white:#FFFFFF;
--color-periwinkle:#BBD1FF;--color-water-green:#E2F4DF;--color-pine:#024442}
body{font-family:Mulish,system-ui,sans-serif;background:var(--color-white);
color:var(--color-navy);margin:0;display:flex;flex-direction:column;min-height:100vh}
header{background:var(--color-navy);color:var(--color-white);padding:10px 18px}
h1{font-family:'Funnel Display',sans-serif;font-size:20px;margin:0}
main{flex:1;padding:16px;display:flex;gap:16px;flex-wrap:wrap}
img{max-width:640px;width:100%;height:auto;align-self:flex-start;border-radius:12px;background:#000}
.card{background:var(--color-water-green);border-radius:12px;padding:14px;min-width:220px}
.card b{color:var(--color-pine)}
#motion{display:inline-block;padding:2px 10px;border-radius:10px;font-weight:700;
background:var(--color-periwinkle)}
#motion.active{background:var(--color-chartreuse)}
</style></head><body>
<header><h1>MadRoomba — Base de charge</h1></header>
<main>
  <img src="/camera" alt="dock camera">
  <div class="card">
    <p>Activité : <span id="motion">—</span></p>
    <p>Score : <b id="score">—</b></p>
    <p>Dernière activité : <b id="last">jamais</b></p>
    <p>Uptime : <b id="uptime">—</b></p>
  </div>
</main>
<script>
setInterval(async () => {
  try {
    const s = await (await fetch('/api/status')).json();
    const m = document.getElementById('motion');
    m.textContent = s.motion ? 'mouvement' : 'calme';
    m.classList.toggle('active', s.motion);
    document.getElementById('score').textContent = s.score.toFixed(2);
    document.getElementById('last').textContent = s.last_motion || 'jamais';
    document.getElementById('uptime').textContent = s.uptime;
  } catch (e) {}
}, 1000);
</script></body></html>`

func main() {
	flag.Parse()
	start := time.Now()

	var custom []string
	if *cameraCmd != "auto" && *cameraCmd != "" {
		custom = strings.Fields(*cameraCmd)
	}
	cam, err := camera.Start(custom, 640, 480, 10)
	if err != nil {
		log.Fatalf("basestationd: %v", err)
	}
	defer cam.Stop()

	det := camera.NewMotionDetector(*threshold)
	go func() {
		for range time.Tick(400 * time.Millisecond) {
			if f := cam.Latest(); f != nil {
				det.Feed(f)
			}
		}
	}()

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write([]byte(page))
	})
	http.Handle("/camera", cam)
	http.HandleFunc("/api/status", func(w http.ResponseWriter, r *http.Request) {
		score, last := det.Status()
		resp := map[string]interface{}{
			"motion": !last.IsZero() && time.Since(last) < 3*time.Second,
			"score":  score,
			"uptime": time.Since(start).Round(time.Second).String(),
		}
		if !last.IsZero() {
			resp["last_motion"] = last.Format("15:04:05")
		}
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(resp)
	})

	log.Printf("basestationd: listening on http://%s", *listen)
	log.Fatal(http.ListenAndServe(*listen, nil))
}
