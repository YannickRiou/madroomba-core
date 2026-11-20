package mapping

import (
	"container/heap"
	"math"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

// Point is a position in the map frame, in meters.
type Point = geom.Point

// planCellM is the resolution of the planning grid: coarser than the map,
// which keeps A* fast on a Pi Zero.
const planCellM = 0.10

// PlanPath returns waypoints from a to b that keep the robot (a disc of the
// given radius) away from mapped obstacles and from every point for which
// blocked returns true. Unknown space counts as free. The path ends exactly
// at b; ok is false when b cannot be reached. Cells around a are always
// passable, so a robot touching a wall can still leave it.
func (g *Grid) PlanPath(a, b Point, radius float64, blocked func(Point) bool) (path []Point, ok bool) {
	return g.PlanPathNear(a, b, radius, 0, blocked)
}

// PlanPathNear is PlanPath for a goal the robot only needs to come within
// near of: when b itself is too close to an obstacle (a frontier along a
// wall), the path ends at the free point nearest to b, if one lies within
// near.
func (g *Grid) PlanPathNear(a, b Point, radius, near float64, blocked func(Point) bool) (path []Point, ok bool) {
	g.mu.Lock()
	k := int(math.Max(1, math.Round(planCellM/g.CellM)))
	cw, ch := g.W/k, g.H/k
	res := float64(k) * g.CellM
	wall := make([]bool, cw*ch)
	// A cell is blocked when its center is closer than radius plus half its
	// diagonal to an obstacle, so that no point of a free cell (and so no
	// straight segment through free cells) comes closer than radius.
	reach := radius + res*math.Sqrt2/2
	r := int(math.Ceil(reach / res))
	for i, v := range g.cells {
		if v < occupiedThreshold {
			continue
		}
		fx, fy := i%g.W, i/g.W
		ox, oy := float64(fx-g.W/2)*g.CellM, float64(g.H/2-fy)*g.CellM
		cx, cy := fx/k, fy/k
		for dy := -r; dy <= r; dy++ {
			for dx := -r; dx <= r; dx++ {
				x, y := cx+dx, cy+dy
				if x < 0 || y < 0 || x >= cw || y >= ch {
					continue
				}
				px, py := (float64(x*k)+float64(k-1)/2-float64(g.W/2))*g.CellM, (float64(g.H/2)-float64(y*k)-float64(k-1)/2)*g.CellM
				if math.Hypot(px-ox, py-oy) <= reach {
					wall[y*cw+x] = true
				}
			}
		}
	}
	g.mu.Unlock()

	toCell := func(p Point) (int, int, bool) {
		fx := g.W/2 + int(math.Round(p.X/g.CellM))
		fy := g.H/2 - int(math.Round(p.Y/g.CellM))
		x, y := fx/k, fy/k
		return x, y, fx >= 0 && fy >= 0 && x < cw && y < ch
	}
	center := func(x, y int) Point {
		return Point{
			X: (float64(x*k) + float64(k-1)/2 - float64(g.W/2)) * g.CellM,
			Y: (float64(g.H/2) - float64(y*k) - float64(k-1)/2) * g.CellM,
		}
	}
	sx, sy, ok1 := toCell(a)
	gx, gy, ok2 := toCell(b)
	if !ok1 || !ok2 {
		return nil, false
	}
	blockedCell := make([]bool, cw*ch)
	for y := 0; y < ch; y++ {
		for x := 0; x < cw; x++ {
			blockedCell[y*cw+x] = wall[y*cw+x] || (blocked != nil && blocked(center(x, y)))
		}
	}
	for dy := -r; dy <= r; dy++ { // the robot is physically at a
		for dx := -r; dx <= r; dx++ {
			if x, y := sx+dx, sy+dy; x >= 0 && y >= 0 && x < cw && y < ch && dx*dx+dy*dy <= r*r {
				blockedCell[y*cw+x] = false
			}
		}
	}
	end := b
	if blockedCell[gy*cw+gx] {
		n, best, bestD := int(math.Ceil(near/res)), -1, math.Inf(1)
		for dy := -n; dy <= n; dy++ {
			for dx := -n; dx <= n; dx++ {
				x, y := gx+dx, gy+dy
				if x < 0 || y < 0 || x >= cw || y >= ch || blockedCell[y*cw+x] {
					continue
				}
				c := center(x, y)
				if d := math.Hypot(c.X-b.X, c.Y-b.Y); d <= near && d < bestD {
					best, bestD = y*cw+x, d
				}
			}
		}
		if best < 0 {
			return nil, false
		}
		gx, gy, end = best%cw, best/cw, center(best%cw, best/cw)
	}

	cells := astar(blockedCell, cw, ch, sy*cw+sx, gy*cw+gx)
	if cells == nil {
		return nil, false
	}
	// Keep only the cells where the straight line from the previous kept
	// cell would hit something.
	clear := func(i, j int) bool {
		x0, y0, x1, y1 := i%cw, i/cw, j%cw, j/cw
		dx, dy := abs(x1-x0), -abs(y1-y0)
		stx, sty := 1, 1
		if x0 > x1 {
			stx = -1
		}
		if y0 > y1 {
			sty = -1
		}
		e := dx + dy
		for {
			if blockedCell[y0*cw+x0] {
				return false
			}
			if x0 == x1 && y0 == y1 {
				return true
			}
			if e2 := 2 * e; e2 >= dy {
				e += dy
				x0 += stx
			} else {
				e += dx
				y0 += sty
			}
		}
	}
	anchor := cells[0]
	for i := 1; i < len(cells); i++ {
		if !clear(anchor, cells[i]) {
			anchor = cells[i-1]
			path = append(path, center(anchor%cw, anchor/cw))
		}
	}
	return append(path, end), true
}

// astar runs A* on an 8-connected grid and returns the cell indexes from
// start to goal, or nil.
func astar(blocked []bool, w, h, start, goal int) []int {
	gScore := make([]float64, len(blocked))
	from := make([]int, len(blocked))
	for i := range gScore {
		gScore[i] = math.Inf(1)
		from[i] = -1
	}
	hfn := func(i int) float64 {
		dx, dy := math.Abs(float64(i%w-goal%w)), math.Abs(float64(i/w-goal/w))
		return math.Max(dx, dy) + (math.Sqrt2-1)*math.Min(dx, dy)
	}
	gScore[start] = 0
	open := &nodeHeap{{start, hfn(start)}}
	for open.Len() > 0 {
		cur := heap.Pop(open).(node)
		if cur.i == goal {
			var out []int
			for i := goal; i != -1; i = from[i] {
				out = append([]int{i}, out...)
			}
			return out
		}
		if cur.f > gScore[cur.i]+hfn(cur.i)+1e-9 {
			continue // stale entry
		}
		x, y := cur.i%w, cur.i/w
		for dy := -1; dy <= 1; dy++ {
			for dx := -1; dx <= 1; dx++ {
				nx, ny := x+dx, y+dy
				if (dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= w || ny >= h {
					continue
				}
				n := ny*w + nx
				// no corner cutting between two blocked cells
				if blocked[n] || (dx != 0 && dy != 0 && (blocked[y*w+nx] || blocked[ny*w+x])) {
					continue
				}
				cost := 1.0
				if dx != 0 && dy != 0 {
					cost = math.Sqrt2
				}
				if t := gScore[cur.i] + cost; t < gScore[n] {
					gScore[n], from[n] = t, cur.i
					heap.Push(open, node{n, t + hfn(n)})
				}
			}
		}
	}
	return nil
}

type node struct {
	i int
	f float64
}

// nodeHeap is the A* open set, a min-heap on f (container/heap).
type nodeHeap []node

func (h nodeHeap) Len() int            { return len(h) }
func (h nodeHeap) Less(i, j int) bool  { return h[i].f < h[j].f }
func (h nodeHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *nodeHeap) Push(x interface{}) { *h = append(*h, x.(node)) }
func (h *nodeHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}
