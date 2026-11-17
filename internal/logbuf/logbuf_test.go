package logbuf

import (
	"fmt"
	"testing"
)

func TestKeepsOnlyTheLastLines(t *testing.T) {
	b := New(3)
	for i := 1; i <= 5; i++ {
		fmt.Fprintf(b, "line %d\n", i)
	}
	got := b.Lines()
	if len(got) != 3 || got[0] != "line 3" || got[2] != "line 5" {
		t.Fatalf("lines = %q, want the last three", got)
	}
}

func TestSplitsMultiLineWrites(t *testing.T) {
	b := New(10)
	b.Write([]byte("a\nb\n"))
	if got := b.Lines(); len(got) != 2 || got[1] != "b" {
		t.Fatalf("lines = %q", got)
	}
}
