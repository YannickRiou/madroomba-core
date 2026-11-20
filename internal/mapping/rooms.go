package mapping

import (
	"math"
	"sort"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

const (
	// roomMinAreaM2 drops closets, bays and map noise: nobody asks the
	// robot to clean a room under one square meter.
	roomMinAreaM2 = 1.0
	// roomSimplifyCells is the Douglas-Peucker tolerance, in cells. Cell
	// staircases on a wall that is not axis-aligned stay under it.
	roomSimplifyCells = 1.5
	// roomMaxPoints is the most vertices a zone may have (zones.MaxPoints;
	// the zones package is not imported to keep the two independent).
	roomMaxPoints = 100
)

// Room is one room found in the map.
type Room struct {
	Outline geom.Polygon // outer boundary, world metres
	Center  Point        // a point inside the room, far from its walls
	AreaM2  float64      // area inside Outline, furniture included
}

// Rooms splits the known free space into rooms separated at openings
// narrower than doorM (a doorway). Rooms are returned largest first.
//
// Free cells further than doorM/2 from any wall, furniture or unknown space
// are the cores of the rooms: a doorway is narrower, so it cuts them apart.
// Each core then grows back over all free cells, so the border between two
// rooms falls in the doorway. Free cells that no core reaches (corridors
// narrower than a doorway, specks) belong to no room. Unexplored space is not
// free, so a room only shows up once enough of it has been seen.
func (g *Grid) Rooms(doorM float64) []Room {
	g.mu.Lock()
	cls := g.classes(true)
	w, h, res := g.W, g.H, g.CellM
	g.mu.Unlock()

	free := make([]bool, len(cls))
	for i, c := range cls {
		free[i] = c == cellFree
	}
	dist := roomDistSq(free, w, h)
	label, count := roomGrow(free, dist, w, h, doorM/2/res)

	// Per room: its first cell in reading order, where the outline starts,
	// and its cell furthest from any wall.
	first := make([]int, count+1)
	center := make([]int, count+1)
	for i := range first {
		first[i], center[i] = -1, -1
	}
	for i, l := range label {
		if l == 0 {
			continue
		}
		if first[l] < 0 {
			first[l] = i
		}
		if center[l] < 0 || dist[i] > dist[center[l]] {
			center[l] = i
		}
	}

	cell := func(x, y int) Point { // world position of a cell centre
		return Point{X: float64(x-w/2) * res, Y: float64(h/2-y) * res}
	}
	var rooms []Room
	for l := 1; l <= count; l++ {
		corners := roomTrace(label, int32(l), w, h, first[l]%w, first[l]/w)
		ring := make(geom.Polygon, len(corners))
		for i, c := range corners { // lattice corners, not cell centres
			ring[i] = Point{X: (float64(c[0]-w/2) - 0.5) * res, Y: (float64(h/2-c[1]) + 0.5) * res}
		}
		outline := roomFit(ring, roomSimplifyCells*res)
		if outline == nil || outline.Area() < roomMinAreaM2 {
			continue
		}
		rooms = append(rooms, Room{
			Outline: outline,
			Center:  cell(center[l]%w, center[l]/w),
			AreaM2:  outline.Area(),
		})
	}
	sort.Slice(rooms, func(i, j int) bool { return rooms[i].AreaM2 > rooms[j].AreaM2 })
	return rooms
}

// roomGrow labels the rooms: 1..count over the free cells, 0 elsewhere. The
// cores are the 4-connected groups of free cells further than minDist cells
// from a non-free one, and they grow breadth first so they meet halfway.
func roomGrow(free []bool, dist []float64, w, h int, minDist float64) (label []int32, count int) {
	label = make([]int32, len(free))
	min2 := minDist * minDist
	var queue []int
	for i := range free {
		if !free[i] || dist[i] <= min2 || label[i] != 0 {
			continue
		}
		count++
		label[i] = int32(count)
		core := len(queue)
		queue = append(queue, i)
		for ; core < len(queue); core++ { // flood this core
			roomNeighbours(queue[core], w, h, func(n int) {
				if free[n] && dist[n] > min2 && label[n] == 0 {
					label[n] = int32(count)
					queue = append(queue, n)
				}
			})
		}
	}
	// Every core cell is queued, in core order; the same queue now spreads
	// them over the rest of the free cells.
	for i := 0; i < len(queue); i++ {
		l := label[queue[i]]
		roomNeighbours(queue[i], w, h, func(n int) {
			if free[n] && label[n] == 0 {
				label[n] = l
				queue = append(queue, n)
			}
		})
	}
	return label, count
}

// roomNeighbours calls f with the 4 neighbours of cell i inside the grid.
func roomNeighbours(i, w, h int, f func(int)) {
	x, y := i%w, i/w
	if x > 0 {
		f(i - 1)
	}
	if x < w-1 {
		f(i + 1)
	}
	if y > 0 {
		f(i - w)
	}
	if y < h-1 {
		f(i + w)
	}
}

// roomDistSq returns, for each free cell, the squared distance in cells to
// the nearest non-free cell (0 for the others). Outside the grid counts as
// non-free. It is the exact transform of Felzenszwalb and Huttenlocher: a
// parabola envelope along columns, then along rows.
func roomDistSq(free []bool, w, h int) []float64 {
	const inf = 1e10
	pw, ph := w+2, h+2 // a non-free border all around
	f := make([]float64, pw*ph)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if free[y*w+x] {
				f[(y+1)*pw+x+1] = inf
			}
		}
	}
	n := pw
	if ph > n {
		n = ph
	}
	line, out := make([]float64, n), make([]float64, n)
	v, z := make([]int, n), make([]float64, n+1)
	for x := 0; x < pw; x++ {
		for y := 0; y < ph; y++ {
			line[y] = f[y*pw+x]
		}
		roomEnvelope(line[:ph], out[:ph], v, z)
		for y := 0; y < ph; y++ {
			f[y*pw+x] = out[y]
		}
	}
	for y := 0; y < ph; y++ {
		copy(line, f[y*pw:(y+1)*pw])
		roomEnvelope(line[:pw], f[y*pw:(y+1)*pw], v, z)
	}
	d := make([]float64, w*h)
	for y := 0; y < h; y++ {
		copy(d[y*w:(y+1)*w], f[(y+1)*pw+1:(y+1)*pw+1+w])
	}
	return d
}

// roomEnvelope writes into out the lower envelope of the parabolas rooted at
// each index of f: out[q] = min over p of f[p] + (q-p)². v and z are scratch.
func roomEnvelope(f, out []float64, v []int, z []float64) {
	k := 0
	v[0] = 0
	z[0], z[1] = math.Inf(-1), math.Inf(1)
	for q := 1; q < len(f); q++ {
		for {
			p := v[k]
			s := ((f[q] + float64(q*q)) - (f[p] + float64(p*p))) / float64(2*q-2*p)
			if s > z[k] {
				k++
				v[k], z[k], z[k+1] = q, s, math.Inf(1)
				break
			}
			k--
		}
	}
	k = 0
	for q := range f {
		for z[k+1] < float64(q) {
			k++
		}
		out[q] = float64((q-v[k])*(q-v[k])) + f[v[k]]
	}
}

// roomTrace follows the outer boundary of room id, which starts at its first
// cell (sx, sy) in reading order, and returns its corners as lattice points:
// (x, y) is the top-left corner of cell (x, y). The room stays on the right
// hand and cells touching only by a corner are not joined, which matches how
// the room was grown. Holes are not visited.
func roomTrace(label []int32, id int32, w, h, sx, sy int) [][2]int {
	in := func(x, y int) bool { return x >= 0 && y >= 0 && x < w && y < h && label[y*w+x] == id }
	dirs := [4][2]int{{1, 0}, {0, 1}, {-1, 0}, {0, -1}} // east, then clockwise on screen
	x, y, d := sx, sy, 0
	var out [][2]int
	for {
		x, y = x+dirs[d][0], y+dirs[d][1]
		r := dirs[(d+1)&3]
		ahead := func(side int) bool { // the cell ahead on the right (+1) or left (-1)
			return in(x+(dirs[d][0]+side*r[0]-1)/2, y+(dirs[d][1]+side*r[1]-1)/2)
		}
		nd := d
		switch {
		case !ahead(1):
			nd = (d + 1) & 3
		case ahead(-1):
			nd = (d + 3) & 3
		}
		if nd != d {
			out = append(out, [2]int{x, y})
		}
		d = nd
		if x == sx && y == sy && d == 0 {
			return out
		}
	}
}

// roomFit simplifies a closed outline with Douglas-Peucker. The tolerance
// grows until the result is a simple polygon a zone may take, so a very
// ragged room gets a coarser outline rather than an invalid one. It returns
// nil if nothing is left.
func roomFit(ring geom.Polygon, tol float64) geom.Polygon {
	for ; ; tol *= 1.5 {
		out := roomSimplify(ring, tol)
		if len(out) < 3 {
			return nil
		}
		if len(out) <= roomMaxPoints && out.Simple() {
			return out
		}
	}
}

// roomSimplify is Douglas-Peucker on a closed ring, split at the first
// point and the one furthest from it.
func roomSimplify(ring geom.Polygon, tol float64) geom.Polygon {
	n := len(ring)
	pts := append(append([]Point(nil), ring...), ring[0])
	far := 0
	for i := range ring {
		if geom.Dist(ring[0], ring[i]) > geom.Dist(ring[0], ring[far]) {
			far = i
		}
	}
	keep := make([]bool, n+1)
	keep[0], keep[far] = true, true
	var dp func(i, j int)
	dp = func(i, j int) {
		worst, at := tol, -1
		for k := i + 1; k < j; k++ {
			if d := geom.SegmentDist(pts[k], pts[i], pts[j]); d > worst {
				worst, at = d, k
			}
		}
		if at >= 0 {
			keep[at] = true
			dp(i, at)
			dp(at, j)
		}
	}
	dp(0, far)
	dp(far, n)
	var out geom.Polygon
	for i := 0; i < n; i++ {
		if keep[i] {
			out = append(out, pts[i])
		}
	}
	return out
}
