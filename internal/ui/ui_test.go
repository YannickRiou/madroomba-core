package ui

import (
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

func TestConfigAndPage(t *testing.T) {
	mux := http.NewServeMux()
	self, base := "", "/proxy/base"
	Mount(mux, &self, &base)

	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, httptest.NewRequest("GET", "/api/config", nil))
	if got := strings.TrimSpace(rec.Body.String()); got != `{"base":"/proxy/base","robot":""}` {
		t.Fatalf("config = %s", got)
	}

	rec = httptest.NewRecorder()
	mux.ServeHTTP(rec, httptest.NewRequest("GET", "/", nil))
	if !strings.Contains(rec.Body.String(), "<title>MadRoomba</title>") {
		t.Fatal("page not served on /")
	}
	rec = httptest.NewRecorder()
	mux.ServeHTTP(rec, httptest.NewRequest("GET", "/nope", nil))
	if rec.Code != http.StatusNotFound {
		t.Fatalf("unknown path = %d, want 404", rec.Code)
	}
}

func TestProxyRelaysAndSurvivesAnOfflinePeer(t *testing.T) {
	peer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		io.WriteString(w, "peer:"+r.URL.Path)
	}))
	mux := http.NewServeMux()
	if err := Proxy(mux, "/proxy/x", peer.URL); err != nil {
		t.Fatal(err)
	}
	front := httptest.NewServer(mux)
	defer front.Close()

	get := func() (int, string) {
		resp, err := http.Get(front.URL + "/proxy/x/api/status")
		if err != nil {
			t.Fatal(err)
		}
		defer resp.Body.Close()
		b, _ := io.ReadAll(resp.Body)
		return resp.StatusCode, string(b)
	}
	if code, body := get(); code != 200 || body != "peer:/api/status" {
		t.Fatalf("relay = %d %q", code, body)
	}
	peer.Close()
	if code, _ := get(); code != http.StatusBadGateway {
		t.Fatalf("offline peer = %d, want 502", code)
	}
}
