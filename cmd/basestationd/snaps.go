package main

import (
	"errors"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

// snapStore keeps JPEG snapshots in a directory, named
// <YYYYMMDD-HHMMSS>-<reason>.jpg (reason: "motion" or "manual"), and keeps at
// most `keep` of them.
type snapStore struct {
	dir  string
	keep int
}

var errUnknownSnap = errors.New("unknown snapshot")

// save writes a snapshot and prunes the oldest ones beyond the limit.
func (s snapStore) save(frame []byte, reason string) (string, error) {
	name := time.Now().Format("20060102-150405") + "-" + reason + ".jpg"
	if err := os.WriteFile(filepath.Join(s.dir, name), frame, 0o644); err != nil {
		return "", err
	}
	files := s.list() // newest first
	for i := s.keep; i < len(files); i++ {
		os.Remove(filepath.Join(s.dir, files[i]))
	}
	return name, nil
}

// list returns snapshot names, newest first.
func (s snapStore) list() []string {
	entries, _ := os.ReadDir(s.dir)
	var names []string
	for _, e := range entries {
		if strings.HasSuffix(e.Name(), ".jpg") {
			names = append(names, e.Name())
		}
	}
	sort.Sort(sort.Reverse(sort.StringSlice(names)))
	return names
}

// delete removes one snapshot. Only names returned by list are accepted, so
// a request can never reach a path outside the directory.
func (s snapStore) delete(name string) error {
	if !contains(s.list(), name) {
		return errUnknownSnap
	}
	return os.Remove(filepath.Join(s.dir, name))
}

func (s snapStore) deleteAll() {
	for _, n := range s.list() {
		os.Remove(filepath.Join(s.dir, n))
	}
}
