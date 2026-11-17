// Package ui holds the single web page shared by madroombad (robot) and
// basestationd (dock). Both daemons serve the same page and relay the other
// daemon under /proxy/..., so the browser only ever talks to one address and
// still sees both halves.
package ui

import (
	_ "embed"
	"encoding/json"
	"net/http"
	"net/http/httputil"
	"net/url"
)

//go:embed index.html
var page []byte

// Mount registers the page ("/") and GET /api/config on mux. robot and base
// are the URL prefixes the page must use for each daemon: "" means same
// origin, nil means "not configured" (the page hides that half).
func Mount(mux *http.ServeMux, robot, base *string) {
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Header().Set("Cache-Control", "no-cache")
		w.Write(page)
	})
	mux.HandleFunc("/api/config", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]*string{"robot": robot, "base": base})
	})
}

// Proxy relays prefix/* to the daemon at target (e.g. "http://madserver:8081"),
// streaming responses (SSE, MJPEG) without buffering.
func Proxy(mux *http.ServeMux, prefix, target string) error {
	u, err := url.Parse(target)
	if err != nil {
		return err
	}
	rp := httputil.NewSingleHostReverseProxy(u)
	rp.FlushInterval = -1
	rp.ErrorHandler = func(w http.ResponseWriter, r *http.Request, err error) {
		w.WriteHeader(http.StatusBadGateway) // peer offline: the page shows it
	}
	mux.Handle(prefix+"/", http.StripPrefix(prefix, rp))
	return nil
}
