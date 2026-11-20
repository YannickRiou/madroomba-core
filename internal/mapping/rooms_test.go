package mapping

import (
	"bytes"
	"image/png"
	"math"
	"testing"

	"github.com/YannickRiou/madroomba-core/internal/geom"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/sim"
	"github.com/YannickRiou/madroomba-core/internal/zones"
)

// roomBox is an axis-aligned rectangle: a room or a piece of furniture.
type roomBox struct{ x0, y0, x1, y1 float64 }

func (b roomBox) near(x, y, m float64) bool {
	return x > b.x0-m && x < b.x1+m && y > b.y0-m && y < b.y1+m
}

// roomFlat is a flat made of rooms side by side, with doorways cut in the
// walls between them, and furniture standing in the rooms.
type roomFlat struct {
	rooms     []roomBox
	furniture []roomBox
	doors     []geom.Point // centre of each doorway
	doorW     float64
	world     *sim.World
}

// newRoomFlat builds the walls: the outline of every room, except where a
// doorway is cut, and the sides of the furniture.
func newRoomFlat(rooms, furniture []roomBox, doors []geom.Point, doorW float64) *roomFlat {
	f := &roomFlat{rooms: rooms, furniture: furniture, doors: doors, doorW: doorW, world: &sim.World{}}
	for _, b := range append(append([]roomBox(nil), rooms...), furniture...) {
		for _, s := range []sim.Segment{{X1: b.x0, Y1: b.y0, X2: b.x1, Y2: b.y0}, {X1: b.x1, Y1: b.y0, X2: b.x1, Y2: b.y1},
			{X1: b.x1, Y1: b.y1, X2: b.x0, Y2: b.y1}, {X1: b.x0, Y1: b.y1, X2: b.x0, Y2: b.y0}} {
			f.addWall(s)
		}
	}
	return f
}

// addWall adds a segment minus the doorways that lie on it. Walls shared by
// two rooms are added twice, which is harmless.
func (f *roomFlat) addWall(s sim.Segment) {
	pieces := []sim.Segment{s}
	for _, d := range f.doors {
		var next []sim.Segment
		for _, p := range pieces {
			if !roomOnSegment(p, d, f.doorW/2) {
				next = append(next, p)
				continue
			}
			if s.X1 == s.X2 { // vertical wall
				lo, hi := math.Min(p.Y1, p.Y2), math.Max(p.Y1, p.Y2)
				next = append(next, sim.Segment{X1: p.X1, Y1: lo, X2: p.X1, Y2: d.Y - f.doorW/2}, sim.Segment{X1: p.X1, Y1: d.Y + f.doorW/2, X2: p.X1, Y2: hi})
			} else {
				lo, hi := math.Min(p.X1, p.X2), math.Max(p.X1, p.X2)
				next = append(next, sim.Segment{X1: lo, Y1: p.Y1, X2: d.X - f.doorW/2, Y2: p.Y1}, sim.Segment{X1: d.X + f.doorW/2, Y1: p.Y1, X2: hi, Y2: p.Y1})
			}
		}
		pieces = next
	}
	f.world.Segments = append(f.world.Segments, pieces...)
}

func roomOnSegment(s sim.Segment, p geom.Point, tol float64) bool {
	return geom.SegmentDist(p, geom.Point{X: s.X1, Y: s.Y1}, geom.Point{X: s.X2, Y: s.Y2}) < 1e-9 &&
		math.Min(s.X1, s.X2)-1e-9 <= p.X+tol && p.X-tol <= math.Max(s.X1, s.X2)+1e-9 &&
		math.Min(s.Y1, s.Y2)-1e-9 <= p.Y+tol && p.Y-tol <= math.Max(s.Y1, s.Y2)+1e-9
}

// map integrates scans from poses spread over the free floor, as a robot
// exploring everywhere would, and from the middle of each doorway.
func (f *roomFlat) mapped() *Grid {
	var poses []odom.Pose
	for _, r := range f.rooms {
		for x := r.x0 + 0.4; x < r.x1-0.3; x += 0.8 {
			for y := r.y0 + 0.4; y < r.y1-0.3; y += 0.8 {
				blocked := false
				for _, fu := range f.furniture {
					blocked = blocked || fu.near(x, y, 0.3)
				}
				if !blocked {
					poses = append(poses, odom.Pose{X: x, Y: y})
				}
			}
		}
	}
	for _, d := range f.doors {
		poses = append(poses, odom.Pose{X: d.X, Y: d.Y})
	}
	return roomMapOf(f.world, poses)
}

// roomMapOf integrates a scan of the world from every pose, enough times for
// the free cells to be confident.
func roomMapOf(w *sim.World, poses []odom.Pose) *Grid {
	g := NewGrid(400, 400, 0.05)
	for _, p := range poses {
		s := scanAt(w, p)
		for i := 0; i < 4; i++ {
			g.IntegrateScan(p, s)
		}
	}
	return g
}

// The test flat: three rooms joined by 0.8 m doorways, offset from the map
// origin so that all four quadrants of the grid are exercised.
//
//	+------------+--------------+
//	|    C       |       D      |   y 3..6.5
//	+---- door --+--- door -----+
//	|     A      |       B      |   y -1..3
//	+------------+--------------+
func roomTestFlat() *roomFlat {
	rooms := []roomBox{
		{-4, -1, 0, 3},    // A: 4 x 4
		{0, -1, 3.5, 3},   // B: 3.5 x 4
		{-4, 3, 3.5, 6.5}, // C: 7.5 x 3.5
	}
	furniture := []roomBox{
		{-3.5, -0.8, -2.6, -0.2}, // in A
		{1.2, 0.8, 2.2, 1.6},     // in B
		{-2, 4.2, -0.8, 4.8},     // in C
		{1.5, 5.6, 3.2, 6.3},     // in C
	}
	doors := []geom.Point{{X: 0, Y: 1}, {X: -2, Y: 3}, {X: 1.7, Y: 3}}
	return newRoomFlat(rooms, furniture, doors, 0.8)
}

// roomCheck verifies that rooms are the rooms of the flat: one per room, with
// the right area, the centre inside, and an outline a zone accepts.
func roomCheck(t *testing.T, f *roomFlat, rooms []Room) {
	t.Helper()
	if len(rooms) != len(f.rooms) {
		t.Fatalf("got %d rooms, want %d", len(rooms), len(f.rooms))
	}
	for i, want := range f.rooms {
		cx, cy := (want.x0+want.x1)/2, (want.y0+want.y1)/2
		found := 0
		for _, r := range rooms {
			if !r.Outline.Contains(geom.Point{X: cx, Y: cy}) {
				continue
			}
			found++
			area := (want.x1 - want.x0) * (want.y1 - want.y0)
			if math.Abs(r.AreaM2-area) > 0.15*area {
				t.Errorf("room %d: area %.2f m², want %.2f within 15%%", i, r.AreaM2, area)
			}
			if !r.Outline.Contains(r.Center) {
				t.Errorf("room %d: centre %v outside its outline", i, r.Center)
			}
		}
		if found != 1 {
			t.Errorf("room %d: centre (%.2f, %.2f) lies in %d outlines, want 1", i, cx, cy, found)
		}
	}
	for i, r := range rooms {
		zs := []zones.Zone{{Name: "room", Kind: zones.Clean, Points: r.Outline}}
		if err := zones.Validate(zs); err != nil {
			t.Errorf("outline %d is not a valid zone: %v (%d points)", i, err, len(r.Outline))
		}
		if i > 0 && r.AreaM2 > rooms[i-1].AreaM2 {
			t.Errorf("rooms not sorted largest first at %d", i)
		}
	}
}

func TestRoomsFlat(t *testing.T) {
	f := roomTestFlat()
	roomCheck(t, f, f.mapped().Rooms(1.0))
}

func TestRoomsFlatAfterROSRoundTrip(t *testing.T) {
	f := roomTestFlat()
	g := f.mapped()
	data, yaml, err := g.EncodeROS("flat")
	if err != nil {
		t.Fatal(err)
	}
	meta, err := ParseROSYAML(yaml)
	if err != nil {
		t.Fatal(err)
	}
	img, err := png.Decode(bytes.NewReader(data))
	if err != nil {
		t.Fatal(err)
	}
	loaded := NewGrid(400, 400, 0.05)
	loaded.DecodeROS(meta, img)
	roomCheck(t, f, loaded.Rooms(1.0))
}

func TestRoomsSingleRoomWithFurniture(t *testing.T) {
	room := roomBox{-3, -2.5, 3, 2.5}
	furniture := []roomBox{{-2.8, 1.2, -1.6, 2.3}, {-0.6, -0.4, 0.8, 0.6}, {1.8, -2.3, 2.8, -1.7}}
	f := newRoomFlat([]roomBox{room}, furniture, nil, 0)
	rooms := f.mapped().Rooms(1.0)
	if len(rooms) != 1 {
		t.Fatalf("got %d rooms, want 1", len(rooms))
	}
	if want := 30.0; math.Abs(rooms[0].AreaM2-want) > 0.15*want {
		t.Errorf("area %.2f m², want %.1f within 15%%", rooms[0].AreaM2, want)
	}
}

// DefaultWorld has a 1.2 m opening in its partition: no doorway for a robot
// that calls 1.0 m a door, a doorway for 1.4 m.
func TestRoomsDefaultWorld(t *testing.T) {
	var poses []odom.Pose
	for x := -3.6; x < 3.7; x += 0.7 {
		for y := -2.6; y < 2.7; y += 0.7 {
			if !(roomBox{-2.6, 1.4, -1.4, 2.4}).near(x, y, 0.3) && !(roomBox{-0.5, -2.5, 0.7, -1.9}).near(x, y, 0.3) {
				poses = append(poses, odom.Pose{X: x, Y: y})
			}
		}
	}
	poses = append(poses, odom.Pose{X: 1.2, Y: -0.2})
	g := roomMapOf(sim.DefaultWorld(), poses)
	for _, c := range []struct {
		door float64
		want int
	}{{1.0, 1}, {1.4, 2}} {
		rooms := g.Rooms(c.door)
		t.Logf("doorM %.1f: %d rooms", c.door, len(rooms))
		for _, r := range rooms {
			t.Logf("  %.1f m², centre (%.2f, %.2f), %d points", r.AreaM2, r.Center.X, r.Center.Y, len(r.Outline))
		}
		if len(rooms) != c.want {
			t.Errorf("doorM %.1f: got %d rooms, want %d", c.door, len(rooms), c.want)
		}
	}
}

func TestRoomsEmptyGrid(t *testing.T) {
	if rooms := NewGrid(400, 400, 0.05).Rooms(1.0); len(rooms) != 0 {
		t.Errorf("got %d rooms on an empty grid, want none", len(rooms))
	}
}

func BenchmarkRooms(b *testing.B) {
	g := roomTestFlat().mapped()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.Rooms(1.0)
	}
}
