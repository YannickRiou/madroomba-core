// Package logbuf keeps the last log lines in memory so a web UI can show
// them. Use it as a log sink: log.SetOutput(io.MultiWriter(os.Stderr, buf)).
package logbuf

import (
	"strings"
	"sync"
)

// Buffer is a thread-safe ring of the most recent log lines.
type Buffer struct {
	mu    sync.Mutex
	keep  int
	lines []string
}

// New returns a buffer holding at most keep lines.
func New(keep int) *Buffer { return &Buffer{keep: keep} }

// Write implements io.Writer. It must not log itself.
func (b *Buffer) Write(p []byte) (int, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.lines = append(b.lines, strings.Split(strings.TrimRight(string(p), "\n"), "\n")...)
	if len(b.lines) > b.keep {
		b.lines = b.lines[len(b.lines)-b.keep:]
	}
	return len(p), nil
}

// Lines returns a copy of the buffered lines, oldest first.
func (b *Buffer) Lines() []string {
	b.mu.Lock()
	defer b.mu.Unlock()
	return append([]string(nil), b.lines...)
}
