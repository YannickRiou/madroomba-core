package camera

import (
	"io"
	"strings"
	"testing"
	"testing/iotest"
)

func TestGridDistanceIgnoresGlobalBrightness(t *testing.T) {
	a := make([]float64, 100)
	b := make([]float64, 100)
	for i := range a {
		a[i] = float64(i)
		b[i] = float64(i) + 40 // same picture, brighter
	}
	if d := GridDistance(a, b); d > 1e-9 {
		t.Fatalf("distance = %v, want 0 for a pure brightness shift", d)
	}
	b[0] += 100 // a real difference
	if d := GridDistance(a, b); d < 0.5 {
		t.Fatalf("distance = %v, want a visible difference", d)
	}
	if d := GridDistance(a, a[:10]); d != 0 {
		t.Fatalf("mismatched lengths must give 0, got %v", d)
	}
}

func TestReadLoopSplitsFrames(t *testing.T) {
	f1 := "\xff\xd8AAA\xff\xd9"
	f2 := "\xff\xd8BBBB\xff\xd9"
	in := "junkÿ" + f1 + "xx" + f2
	// one byte per read puts every marker across two reads
	for _, r := range []io.Reader{strings.NewReader(in), iotest.OneByteReader(strings.NewReader(in))} {
		s := &Streamer{subs: make(map[chan []byte]struct{}), done: make(chan struct{})}
		s.readLoop(r)
		if got := string(s.Latest()); got != f2 {
			t.Fatalf("latest frame = %q, want the second frame %q", got, f2)
		}
		select {
		case <-s.done:
		default:
			t.Fatal("done must be closed when the stream ends")
		}
	}
}

func TestMotionDetectorReset(t *testing.T) {
	m := NewMotionDetector(1)
	m.Score = 9
	m.prev = []float64{1}
	m.Reset()
	if score, last := m.Status(); score != 0 || !last.IsZero() || m.prev != nil {
		t.Fatalf("Reset left state behind: score=%v last=%v prev=%v", score, last, m.prev)
	}
}
