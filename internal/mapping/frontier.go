package mapping

import "math"

// Frontiers returns the frontiers of the map: groups of free cells that
// touch unknown space, where driving would reveal more of the place. Each
// frontier is given by one of its own cells, the one closest to the
// group's centroid (the centroid itself may lie in unknown space). Groups
// of fewer than minCells cells are ignored: they are noise, or gaps the
// lidar cannot fill (under a sofa, beyond a glass door).
func (g *Grid) Frontiers(minCells int) []Point {
	g.mu.Lock()
	cls := g.classes(false)
	w, h, res := g.W, g.H, g.CellM
	g.mu.Unlock()

	isFrontier := func(x, y int) bool {
		if cls[y*w+x] != cellFree {
			return false
		}
		for _, d := range [4][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			nx, ny := x+d[0], y+d[1]
			if nx >= 0 && ny >= 0 && nx < w && ny < h && cls[ny*w+nx] == cellUnknown {
				return true
			}
		}
		return false
	}
	seen := make([]bool, w*h)
	var out []Point
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if seen[y*w+x] || !isFrontier(x, y) {
				continue
			}
			// flood the 8-connected group of frontier cells
			group := []int{y*w + x}
			seen[y*w+x] = true
			for i := 0; i < len(group); i++ {
				cx, cy := group[i]%w, group[i]/w
				for dy := -1; dy <= 1; dy++ {
					for dx := -1; dx <= 1; dx++ {
						nx, ny := cx+dx, cy+dy
						if nx < 0 || ny < 0 || nx >= w || ny >= h || seen[ny*w+nx] || !isFrontier(nx, ny) {
							continue
						}
						seen[ny*w+nx] = true
						group = append(group, ny*w+nx)
					}
				}
			}
			if len(group) < minCells {
				continue
			}
			mx, my := 0.0, 0.0
			for _, i := range group {
				mx += float64(i % w)
				my += float64(i / w)
			}
			mx, my = mx/float64(len(group)), my/float64(len(group))
			best, bestD := group[0], math.Inf(1)
			for _, i := range group {
				if d := math.Hypot(float64(i%w)-mx, float64(i/w)-my); d < bestD {
					best, bestD = i, d
				}
			}
			out = append(out, Point{X: float64(best%w-w/2) * res, Y: float64(h/2-best/w) * res})
		}
	}
	return out
}
