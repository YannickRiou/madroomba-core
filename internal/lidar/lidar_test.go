package lidar

import (
	"bytes"
	"io"
	"testing"
)

type fakePort struct{ io.Reader }

func (f *fakePort) Write(p []byte) (int, error) { return len(p), nil }
func (f *fakePort) Close() error                { return nil }

// buildFrame builds a valid 42-byte frame for the given index, with each of
// the 6 measures set to distMM[j] / intens[j].
func buildFrame(idx int, distMM [6]uint16, intens [6]uint16) []byte {
	f := make([]byte, FrameSize)
	f[0] = SyncByte
	f[1] = byte(IndexBase + idx)
	f[2] = 0xB8 // 3000 -> 300.0 RPM
	f[3] = 0x0B
	for j := 0; j < 6; j++ {
		off := 4 + j*6
		f[off] = byte(intens[j])
		f[off+1] = byte(intens[j] >> 8)
		f[off+2] = byte(distMM[j])
		f[off+3] = byte(distMM[j] >> 8)
	}
	return f
}

func TestReadFrame(t *testing.T) {
	dist := [6]uint16{500, 600, 700, 800, 900, 1000}
	intens := [6]uint16{10, 20, 30, 40, 50, 60}
	raw := append([]byte{0x00, 0x42}, buildFrame(3, dist, intens)...) // leading noise

	d := New(&fakePort{bytes.NewReader(raw)})
	f, err := d.ReadFrame()
	if err != nil {
		t.Fatal(err)
	}
	if f.Index != 3 {
		t.Errorf("index = %d, want 3", f.Index)
	}
	if f.RPM != 300.0 {
		t.Errorf("rpm = %f, want 300", f.RPM)
	}
	for j := 0; j < 6; j++ {
		want := float64(dist[j]) / 1000
		if f.Ranges[j] != want {
			t.Errorf("range[%d] = %f, want %f", j, f.Ranges[j], want)
		}
		if f.Intensities[j] != intens[j] {
			t.Errorf("intensity[%d] = %d, want %d", j, f.Intensities[j], intens[j])
		}
	}
}

func TestReadScanAssemblesFullRevolution(t *testing.T) {
	var raw []byte
	mk := func(idx int) []byte {
		var dist, intens [6]uint16
		for j := range dist {
			deg := idx*6 + j
			dist[j] = uint16(1000 + deg) // encodes its own angle
			intens[j] = uint16(deg)
		}
		return buildFrame(idx, dist, intens)
	}
	// One full revolution, then the first frame of the next one to trigger
	// the wrap detection.
	for idx := 0; idx < FramesPerScan; idx++ {
		raw = append(raw, mk(idx)...)
	}
	raw = append(raw, mk(0)...)

	d := New(&fakePort{bytes.NewReader(raw)})
	s, err := d.ReadScan()
	if err != nil {
		t.Fatal(err)
	}
	for deg := 0; deg < 360; deg++ {
		want := float64(1000+deg) / 1000
		if s.Ranges[deg] != want {
			t.Fatalf("ranges[%d] = %f, want %f", deg, s.Ranges[deg], want)
		}
	}
	if s.RPM != 300.0 {
		t.Errorf("rpm = %f, want 300", s.RPM)
	}
}

func TestReadScanFiltersOutOfRange(t *testing.T) {
	var raw []byte
	dist := [6]uint16{0, 50, 120, 3500, 3600, 65535} // mm
	raw = append(raw, buildFrame(0, dist, [6]uint16{})...)
	raw = append(raw, buildFrame(1, [6]uint16{500, 500, 500, 500, 500, 500}, [6]uint16{})...)
	raw = append(raw, buildFrame(0, [6]uint16{}, [6]uint16{})...) // wrap

	d := New(&fakePort{bytes.NewReader(raw)})
	s, err := d.ReadScan()
	if err != nil {
		t.Fatal(err)
	}
	wantValid := map[int]float64{2: 0.12, 3: 3.5}
	for deg := 0; deg < 6; deg++ {
		want := wantValid[deg] // 0 for filtered
		if s.Ranges[deg] != want {
			t.Errorf("ranges[%d] = %f, want %f", deg, s.Ranges[deg], want)
		}
	}
}
