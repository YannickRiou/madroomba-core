package robot

import (
	"fmt"
	"regexp"
	"strconv"
	"strings"
)

// noteToken matches one note of the text notation: a note name with an
// optional sharp and an octave ("C4" is middle C, MIDI 60), or "R" for a
// rest, optionally followed by "/duration" in 1/64 s (16 by default).
var noteToken = regexp.MustCompile(`^(?:([A-Ga-g])(#?)(\d)|([Rr]))(?:/(\d{1,3}))?$`)

var semitone = map[byte]int{'C': 0, 'D': 2, 'E': 4, 'F': 5, 'G': 7, 'A': 9, 'B': 11}

// ParseNotes reads a song written as space-separated notes, for example
// "C5/12 E5/12 G5 R/8 C6/24". The page uses the same notation.
func ParseNotes(text string) ([]Note, error) {
	var out []Note
	for _, tok := range strings.Fields(text) {
		m := noteToken.FindStringSubmatch(tok)
		if m == nil {
			return nil, fmt.Errorf("bad note %q", tok)
		}
		var pitch int
		if m[4] == "" {
			octave, _ := strconv.Atoi(m[3])
			pitch = 12*(octave+1) + semitone[strings.ToUpper(m[1])[0]]
			if m[2] == "#" {
				pitch++
			}
			if pitch < 31 || pitch > 127 {
				return nil, fmt.Errorf("note %q out of range (G1 to G9)", tok)
			}
		}
		dur := 16
		if m[5] != "" {
			dur, _ = strconv.Atoi(m[5])
		}
		if dur < 1 || dur > 255 {
			return nil, fmt.Errorf("note %q: duration 1 to 255", tok)
		}
		out = append(out, Note{Pitch: uint8(pitch), Dur: uint8(dur)})
	}
	if len(out) == 0 || len(out) > MaxNotes {
		return nil, fmt.Errorf("a song has 1 to %d notes, got %d", MaxNotes, len(out))
	}
	return out, nil
}
