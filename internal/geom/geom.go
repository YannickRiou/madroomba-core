// Package geom holds the plane geometry shared by the map, the zones and
// the navigation: points and simple polygons, in meters, in the map frame
// (X right, Y up).
package geom

import "math"

// Point is a position in the map frame, in meters.
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Dist is the distance between two points.
func Dist(a, b Point) float64 { return math.Hypot(a.X-b.X, a.Y-b.Y) }

// SegmentDist is the distance from p to the segment ab.
func SegmentDist(p, a, b Point) float64 {
	dx, dy := b.X-a.X, b.Y-a.Y
	l2 := dx*dx + dy*dy
	if l2 == 0 {
		return Dist(p, a)
	}
	t := math.Max(0, math.Min(1, ((p.X-a.X)*dx+(p.Y-a.Y)*dy)/l2))
	return Dist(p, Point{a.X + t*dx, a.Y + t*dy})
}

// PolylineDist is the distance from p to an open polyline (a virtual
// wall), and Length its total length.
func PolylineDist(pts []Point, p Point) float64 {
	if len(pts) == 1 {
		return Dist(p, pts[0])
	}
	d := math.Inf(1)
	for i := 1; i < len(pts); i++ {
		d = math.Min(d, SegmentDist(p, pts[i-1], pts[i]))
	}
	return d
}

// Length is the total length of an open polyline.
func Length(pts []Point) float64 {
	l := 0.0
	for i := 1; i < len(pts); i++ {
		l += Dist(pts[i-1], pts[i])
	}
	return l
}

// Polygon is a closed polygon given by its vertices in order (either
// direction); the last vertex connects back to the first.
type Polygon []Point

// Contains tells whether p is inside the polygon (even-odd rule).
func (pg Polygon) Contains(p Point) bool {
	in := false
	for i, j := 0, len(pg)-1; i < len(pg); j, i = i, i+1 {
		a, b := pg[i], pg[j]
		if (a.Y > p.Y) != (b.Y > p.Y) && p.X < (b.X-a.X)*(p.Y-a.Y)/(b.Y-a.Y)+a.X {
			in = !in
		}
	}
	return in
}

// EdgeDist is the distance from p to the polygon outline.
func (pg Polygon) EdgeDist(p Point) float64 {
	d := math.Inf(1)
	for i, j := 0, len(pg)-1; i < len(pg); j, i = i, i+1 {
		d = math.Min(d, SegmentDist(p, pg[j], pg[i]))
	}
	return d
}

// Distance is the distance from p to the polygon, 0 inside.
func (pg Polygon) Distance(p Point) float64 {
	if pg.Contains(p) {
		return 0
	}
	return pg.EdgeDist(p)
}

// Area is the polygon area in square meters.
func (pg Polygon) Area() float64 {
	s := 0.0
	for i, j := 0, len(pg)-1; i < len(pg); j, i = i, i+1 {
		s += pg[j].X*pg[i].Y - pg[i].X*pg[j].Y
	}
	return math.Abs(s) / 2
}

// Bounds returns the bounding box corners.
func (pg Polygon) Bounds() (min, max Point) {
	min = Point{math.Inf(1), math.Inf(1)}
	max = Point{math.Inf(-1), math.Inf(-1)}
	for _, p := range pg {
		min.X, min.Y = math.Min(min.X, p.X), math.Min(min.Y, p.Y)
		max.X, max.Y = math.Max(max.X, p.X), math.Max(max.Y, p.Y)
	}
	return min, max
}

// Simple tells whether no two non-adjacent edges cross, so the polygon
// has a well-defined inside.
func (pg Polygon) Simple() bool {
	n := len(pg)
	for i := 0; i < n; i++ {
		a, b := pg[i], pg[(i+1)%n]
		for j := i + 1; j < n; j++ {
			if j == i+1 || (i == 0 && j == n-1) {
				continue // adjacent edges share a vertex
			}
			if segmentsCross(a, b, pg[j], pg[(j+1)%n]) {
				return false
			}
		}
	}
	return true
}

func segmentsCross(a, b, c, d Point) bool {
	cross := func(o, p, q Point) float64 { return (p.X-o.X)*(q.Y-o.Y) - (p.Y-o.Y)*(q.X-o.X) }
	d1, d2 := cross(c, d, a), cross(c, d, b)
	d3, d4 := cross(a, b, c), cross(a, b, d)
	return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0
}

// Rect returns the rectangle with corners (x0, y0) and (x1, y1).
func Rect(x0, y0, x1, y1 float64) Polygon {
	return Polygon{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
}
