package robot

import "testing"

func TestParseNotes(t *testing.T) {
	got, err := ParseNotes("C4 A4/32 C#5/8 R/4 g9")
	if err != nil {
		t.Fatal(err)
	}
	want := []Note{{60, 16}, {69, 32}, {73, 8}, {0, 4}, {127, 16}}
	if len(got) != len(want) {
		t.Fatalf("got %v", got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("note %d = %v, want %v", i, got[i], want[i])
		}
	}
	for _, bad := range []string{"", "H4", "C4/0", "C4/300", "F1", "C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4 C4"} {
		if _, err := ParseNotes(bad); err == nil {
			t.Errorf("%q accepted", bad)
		}
	}
}
