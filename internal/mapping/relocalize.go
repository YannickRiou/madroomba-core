package mapping

import (
	"math"
	"sort"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
)

const (
	// relocLevels is the depth of the pyramid: the coarsest level pools
	// blocks of 2^(relocLevels-1) cells, 80 cm on a 5 cm grid.
	relocLevels = 5
	// relocMinRays is how many valid rays a scan needs; fewer see too little
	// of the room to tell places apart.
	relocMinRays = 40
	// relocMinScore is the share of rays that must land on an obstacle (with
	// one cell of tolerance) for a pose to count as explaining the scan.
	relocMinScore = 0.6
	// relocAmbiguity: another pose scoring this share of the best one makes
	// the answer ambiguous.
	relocAmbiguity = 0.9
	// A pose is "the same place" as the best one within relocSameM metres and
	// relocSameDeg degrees: the peak around a true pose is not a rival.
	relocSameM   = 0.30
	relocSameDeg = 10
)

// Bits of a pyramid cell. Pooling ORs them, so a coarse cell says whether any
// of its fine cells has the property.
const (
	relocNear  byte = 1 << iota // obstacle within one cell
	relocFree                   // known free: the robot can be here
	relocExact                  // obstacle on this very cell
)

// Localization is the result of a global search.
type Localization struct {
	Pose  odom.Pose
	Score float64 // share of the scan's valid rays that land on an obstacle, 0..1
	// Second is the score of the best pose that is clearly somewhere else. It
	// is only searched down to relocAmbiguity of Score, and is 0 when no other
	// pose gets that close.
	Second float64
}

// Relocalize searches the whole map for the pose that best explains the scan.
// ok is false when no pose explains the scan well enough, or when two distant
// poses explain it about equally well (symmetric rooms, too few returns); loc
// then still holds the best pose found, for diagnostics.
// The grid is only locked while its cells are copied: the search takes
// seconds on a Pi Zero and must not hold up the lidar loop.
func (g *Grid) Relocalize(s *lidar.Scan) (loc Localization, ok bool) {
	rays := relocRays(s)
	if len(rays) < relocMinRays {
		return loc, false
	}
	g.mu.Lock()
	cls := g.classes(true)
	w, h, cell := g.W, g.H, g.CellM
	g.mu.Unlock()

	srch := newRelocSearch(relocBuild(cls, w, h, cell), rays)
	n := float64(len(rays))
	srch.best = int(math.Ceil(relocMinScore*n)) - 1
	if !srch.run(nil) {
		return loc, false
	}
	first := srch.found
	loc.Pose = srch.pose(first)
	loc.Pose, loc.Score = srch.p.refine(loc.Pose, rays)

	srch.best = int(math.Ceil(relocAmbiguity*float64(first.score))) - 1
	if srch.run(&first) {
		loc.Second = float64(srch.found.score) / n
		return loc, false
	}
	return loc, true
}

// relocRay is one valid lidar return.
type relocRay struct {
	deg  int     // index in Scan.Ranges
	r    float64 // meters
	c, s float64 // cos and sin of the ray angle in the robot frame
}

func relocRays(s *lidar.Scan) []relocRay {
	var rays []relocRay
	for deg, r := range s.Ranges {
		if r < lidar.MinRangeM || r > lidar.MaxRangeM {
			continue
		}
		a := float64(deg) * math.Pi / 180
		rays = append(rays, relocRay{deg, r, math.Cos(a), math.Sin(a)})
	}
	return rays
}

// relocPyramid holds the map as bit cells at several resolutions, padded by
// the lidar range so a ray never needs a bounds check.
type relocPyramid struct {
	w, h         int // grid size, cells
	pad          int
	stride, rows int
	cell         float64
	lv           [relocLevels][]byte
}

// index is the position of grid cell (x, y) in a level.
func (p *relocPyramid) index(x, y int) int {
	return (y+p.pad)*p.stride + x + p.pad
}

// relocBuild turns cell classes into the pyramid. Level k holds, at each
// cell, the OR of the 2^k x 2^k block that starts there.
func relocBuild(cls []int8, w, h int, cell float64) *relocPyramid {
	pad := int(math.Ceil(lidar.MaxRangeM/cell)) + 1 + (1 << (relocLevels - 1))
	p := &relocPyramid{w: w, h: h, pad: pad, stride: w + 2*pad, rows: h + 2*pad, cell: cell}
	base := make([]byte, p.stride*p.rows)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			i := p.index(x, y)
			switch cls[y*w+x] {
			case cellFree:
				base[i] |= relocFree
			case cellOccupied:
				base[i] |= relocExact
				for dy := -1; dy <= 1; dy++ {
					for dx := -1; dx <= 1; dx++ {
						base[i+dy*p.stride+dx] |= relocNear
					}
				}
			}
		}
	}
	p.lv[0] = base
	for k := 1; k < relocLevels; k++ {
		src, half := p.lv[k-1], 1<<(k-1)
		dst := make([]byte, len(src))
		for y := 0; y < p.rows; y++ {
			for x := 0; x < p.stride; x++ {
				i := y*p.stride + x
				v := src[i]
				right, down := x+half < p.stride, y+half < p.rows
				if right {
					v |= src[i+half]
				}
				if down {
					v |= src[i+half*p.stride]
					if right {
						v |= src[i+half*p.stride+half]
					}
				}
				dst[i] = v
			}
		}
		p.lv[k] = dst
	}
	return p
}

// fit counts the rays whose end lands near an obstacle and on one, for a pose
// that need not sit on a cell center.
func (p *relocPyramid) fit(pose odom.Pose, rays []relocRay) (near, exact int) {
	ct, st := math.Cos(pose.Theta), math.Sin(pose.Theta)
	l0 := p.lv[0]
	for _, r := range rays {
		x := pose.X + r.r*(ct*r.c-st*r.s)
		y := pose.Y + r.r*(st*r.c+ct*r.s)
		cx := p.w/2 + int(math.Round(x/p.cell))
		cy := p.h/2 - int(math.Round(y/p.cell))
		v := l0[p.index(cx, cy)]
		near += int(v & relocNear)
		exact += int(v&relocExact) >> 2
	}
	return near, exact
}

// refine hill climbs below the cell and degree resolution of the search and
// returns the pose with its share of rays near an obstacle.
func (p *relocPyramid) refine(pose odom.Pose, rays []relocRay) (odom.Pose, float64) {
	obj := func(q odom.Pose) int {
		n, e := p.fit(q, rays)
		return n + e
	}
	best, bestObj := pose, obj(pose)
	for _, st := range [...]struct{ d, a float64 }{
		{0.025, 0.5 * math.Pi / 180},
		{0.0125, 0.25 * math.Pi / 180},
		{0.005, 0.1 * math.Pi / 180},
	} {
		for iter := 0; iter < 10; iter++ {
			improved := false
			for _, c := range [...]odom.Pose{
				{X: best.X + st.d, Y: best.Y, Theta: best.Theta},
				{X: best.X - st.d, Y: best.Y, Theta: best.Theta},
				{X: best.X, Y: best.Y + st.d, Theta: best.Theta},
				{X: best.X, Y: best.Y - st.d, Theta: best.Theta},
				{X: best.X, Y: best.Y, Theta: best.Theta + st.a},
				{X: best.X, Y: best.Y, Theta: best.Theta - st.a},
			} {
				if o := obj(c); o > bestObj {
					best, bestObj = c, o
					improved = true
				}
			}
			if !improved {
				break
			}
		}
	}
	best.Theta = odom.NormalizeAngle(best.Theta)
	near, _ := p.fit(best, rays)
	return best, float64(near) / float64(len(rays))
}

// relocNode is a block of 2^k x 2^k candidate positions (x, y is its corner,
// in grid cells) at heading a degrees, with an upper bound of its score.
type relocNode struct {
	x, y, a, k int
	score      int
}

// relocSearch is a branch and bound over (x, y, heading): a block's bound is
// the rays that end near an obstacle anywhere in its pooled cell, so no
// position inside the block can do better, and blocks that cannot beat the
// best leaf so far are never opened.
type relocSearch struct {
	p    *relocPyramid
	offs [360][]int32 // per heading, index offset of each ray's end
	best int          // a result must score above this
	// found is the best leaf above best, when run reports one.
	found relocNode
	hit   bool
	apart *relocNode // leaves near this one are not candidates
}

func newRelocSearch(p *relocPyramid, rays []relocRay) *relocSearch {
	s := &relocSearch{p: p}
	var cs [360][2]float64
	for i := range cs {
		a := float64(i) * math.Pi / 180
		cs[i] = [2]float64{math.Cos(a), math.Sin(a)}
	}
	for a := range s.offs {
		o := make([]int32, len(rays))
		for i, r := range rays {
			k := (a + r.deg) % 360
			dx := int(math.Round(r.r * cs[k][0] / p.cell))
			dy := -int(math.Round(r.r * cs[k][1] / p.cell))
			o[i] = int32(dy*p.stride + dx)
		}
		s.offs[a] = o
	}
	return s
}

// pose is the world pose at the center of a leaf's cell.
func (s *relocSearch) pose(n relocNode) odom.Pose {
	return odom.Pose{
		X:     float64(n.x-s.p.w/2) * s.p.cell,
		Y:     float64(s.p.h/2-n.y) * s.p.cell,
		Theta: odom.NormalizeAngle(float64(n.a) * math.Pi / 180),
	}
}

func (s *relocSearch) score(k, x, y, a int) int {
	lv := s.p.lv[k]
	base := s.p.index(x, y)
	sum := 0
	for _, o := range s.offs[a] {
		sum += int(lv[base+int(o)] & relocNear)
	}
	return sum
}

// excluded reports whether the whole block is near the apart node, in
// position and heading.
func (s *relocSearch) excluded(x, y, a, k int) bool {
	q := s.apart
	if q == nil {
		return false
	}
	da := (a - q.a + 360) % 360
	if da > 180 {
		da = 360 - da
	}
	if da > relocSameDeg {
		return false
	}
	far := (1 << k) - 1
	dx := math.Max(math.Abs(float64(x-q.x)), math.Abs(float64(x+far-q.x)))
	dy := math.Max(math.Abs(float64(y-q.y)), math.Abs(float64(y+far-q.y)))
	return math.Hypot(dx, dy)*s.p.cell <= relocSameM
}

// before orders nodes by decreasing bound, with a fixed tie break so the
// result does not depend on sort internals.
func (n relocNode) before(m relocNode) bool {
	switch {
	case n.score != m.score:
		return n.score > m.score
	case n.a != m.a:
		return n.a < m.a
	case n.y != m.y:
		return n.y < m.y
	}
	return n.x < m.x
}

// run looks for a leaf scoring above s.best, skipping those near apart.
// Returns whether it found one; s.found is then the best.
func (s *relocSearch) run(apart *relocNode) bool {
	s.apart, s.hit = apart, false
	top := relocLevels - 1
	var nodes []relocNode
	for y := 0; y < s.p.h; y += 1 << top {
		for x := 0; x < s.p.w; x += 1 << top {
			if s.p.lv[top][s.p.index(x, y)]&relocFree == 0 {
				continue
			}
			for a := 0; a < 360; a++ {
				if s.excluded(x, y, a, top) {
					continue
				}
				if sc := s.score(top, x, y, a); sc > s.best {
					nodes = append(nodes, relocNode{x, y, a, top, sc})
				}
			}
		}
	}
	sort.Slice(nodes, func(i, j int) bool { return nodes[i].before(nodes[j]) })
	s.descend(nodes)
	return s.hit
}

// descend opens nodes, sorted by decreasing bound, best first.
func (s *relocSearch) descend(nodes []relocNode) {
	for _, n := range nodes {
		if n.score <= s.best {
			return
		}
		if n.k == 0 {
			s.best, s.found, s.hit = n.score, n, true
			return
		}
		var kids [4]relocNode
		m := 0
		half := 1 << (n.k - 1)
		for _, d := range [4][2]int{{0, 0}, {half, 0}, {0, half}, {half, half}} {
			x, y := n.x+d[0], n.y+d[1]
			if x >= s.p.w || y >= s.p.h || s.p.lv[n.k-1][s.p.index(x, y)]&relocFree == 0 ||
				s.excluded(x, y, n.a, n.k-1) {
				continue
			}
			if sc := s.score(n.k-1, x, y, n.a); sc > s.best {
				kids[m] = relocNode{x, y, n.a, n.k - 1, sc}
				m++
			}
		}
		sort.Slice(kids[:m], func(i, j int) bool { return kids[i].before(kids[j]) })
		s.descend(kids[:m])
	}
}
