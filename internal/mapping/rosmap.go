package mapping

import (
	"bufio"
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"strconv"
	"strings"
)

// freeThreshold is the log-odds value below which a cell counts as free.
const freeThreshold = -20

// Cell classes used by the filtered view and the ROS export.
const (
	cellFree     int8 = -1
	cellUnknown  int8 = 0
	cellOccupied int8 = 1
)

// classes sorts every cell into free, unknown or occupied. With filter, an
// obstacle cell with no obstacle among its 8 neighbours (a leg walking by,
// a reflection) becomes free when mostly surrounded by free space, unknown
// otherwise. The grid itself is left untouched. Caller holds g.mu.
func (g *Grid) classes(filter bool) []int8 {
	c := make([]int8, len(g.cells))
	for i, v := range g.cells {
		switch {
		case v >= occupiedThreshold:
			c[i] = cellOccupied
		case v <= freeThreshold:
			c[i] = cellFree
		}
	}
	if !filter {
		return c
	}
	out := append([]int8(nil), c...)
	for y := 1; y < g.H-1; y++ {
		for x := 1; x < g.W-1; x++ {
			i := y*g.W + x
			if c[i] != cellOccupied {
				continue
			}
			occ, free := 0, 0
			for dy := -1; dy <= 1; dy++ {
				for dx := -1; dx <= 1; dx++ {
					switch c[i+dy*g.W+dx] {
					case cellOccupied:
						occ++
					case cellFree:
						free++
					}
				}
			}
			if occ == 1 { // only itself
				if free >= 5 {
					out[i] = cellFree
				} else {
					out[i] = cellUnknown
				}
			}
		}
	}
	return out
}

// ROS map_server conventions for a "trinary" map image.
const (
	rosOccupied = 0
	rosFree     = 254
	rosUnknown  = 205
)

// ROSMeta is the content of a ROS map_server YAML file.
type ROSMeta struct {
	Image          string
	Resolution     float64
	Origin         [3]float64
	Negate         bool
	OccupiedThresh float64
	FreeThresh     float64
}

// EncodeROS exports the filtered map in the ROS map_server format: a
// grayscale PNG (black obstacle, white free, gray unknown) and its YAML
// description, which refers to the image as name + ".png".
func (g *Grid) EncodeROS(name string) (pngData, yaml []byte, err error) {
	g.mu.Lock()
	cls := g.classes(true)
	w, h, res := g.W, g.H, g.CellM
	g.mu.Unlock()

	img := image.NewGray(image.Rect(0, 0, w, h))
	for i, c := range cls {
		switch c {
		case cellOccupied:
			img.Pix[i] = rosOccupied
		case cellFree:
			img.Pix[i] = rosFree
		default:
			img.Pix[i] = rosUnknown
		}
	}
	var buf bytes.Buffer
	if err := png.Encode(&buf, img); err != nil {
		return nil, nil, err
	}
	// Cell (W/2, H/2) is centered on world (0,0); the origin is the world
	// position of the image's lower-left corner.
	ox := (-float64(w)/2 - 0.5) * res
	oy := (0.5 - float64(h)/2) * res
	yaml = []byte(fmt.Sprintf("image: %s.png\nmode: trinary\nresolution: %.6g\norigin: [%.6g, %.6g, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
		name, res, ox, oy))
	return buf.Bytes(), yaml, nil
}

// ParseROSYAML reads the keys of a ROS map YAML file that matter here.
func ParseROSYAML(data []byte) (ROSMeta, error) {
	m := ROSMeta{OccupiedThresh: 0.65, FreeThresh: 0.196}
	sc := bufio.NewScanner(bytes.NewReader(data))
	for sc.Scan() {
		k, v, ok := strings.Cut(sc.Text(), ":")
		if !ok {
			continue
		}
		k, v = strings.TrimSpace(k), strings.TrimSpace(v)
		if i := strings.Index(v, "#"); i >= 0 {
			v = strings.TrimSpace(v[:i])
		}
		var err error
		switch k {
		case "image":
			m.Image = strings.Trim(v, `"'`)
		case "resolution":
			m.Resolution, err = strconv.ParseFloat(v, 64)
		case "negate":
			m.Negate = v == "1" || v == "true"
		case "occupied_thresh":
			m.OccupiedThresh, err = strconv.ParseFloat(v, 64)
		case "free_thresh":
			m.FreeThresh, err = strconv.ParseFloat(v, 64)
		case "origin":
			parts := strings.Split(strings.Trim(v, "[] "), ",")
			if len(parts) != 3 {
				return m, fmt.Errorf("origin: want 3 values, got %q", v)
			}
			for i, p := range parts {
				if m.Origin[i], err = strconv.ParseFloat(strings.TrimSpace(p), 64); err != nil {
					break
				}
			}
		}
		if err != nil {
			return m, fmt.Errorf("%s: %w", k, err)
		}
	}
	if m.Resolution <= 0 {
		return m, fmt.Errorf("resolution missing or not positive")
	}
	return m, sc.Err()
}

// DecodeROS replaces the grid content with a ROS map. Each image pixel is
// placed at its world position, so the image may have another size or
// resolution than the grid; what falls outside the grid is dropped.
// Obstacles and free cells come back with a firm confidence, and scan
// matching is enabled at once.
func (g *Grid) DecodeROS(m ROSMeta, img image.Image) {
	g.mu.Lock()
	defer g.mu.Unlock()
	for i := range g.cells {
		g.cells[i] = 0
	}
	b := img.Bounds()
	ih := b.Dy()
	for py := b.Min.Y; py < b.Max.Y; py++ {
		for px := b.Min.X; px < b.Max.X; px++ {
			p := float64(color.GrayModel.Convert(img.At(px, py)).(color.Gray).Y)
			if m.Negate {
				p = 255 - p
			}
			occ := (255 - p) / 255
			var v int16
			switch {
			case occ > m.OccupiedThresh:
				v = 2 * occupiedThreshold
			case occ < m.FreeThresh:
				v = 2 * freeThreshold
			default:
				continue
			}
			x := m.Origin[0] + (float64(px-b.Min.X)+0.5)*m.Resolution
			y := m.Origin[1] + (float64(ih-(py-b.Min.Y))-0.5)*m.Resolution
			if cx, cy, ok := g.worldToCell(x, y); ok {
				g.cells[cy*g.W+cx] = v
			}
		}
	}
	if g.updates < 5 {
		g.updates = 5
	}
}
