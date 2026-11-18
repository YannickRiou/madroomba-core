// Package lidar implements a driver for the Hitachi-LG HLS-LFCD-LDS2
// (LDS-01, the TurtleBot3 lidar).
//
// Protocol (230400 baud, 8N1): once started with 'b', the sensor streams
// 42-byte frames. Each frame covers 6 degrees:
//
//	byte 0     : 0xFA sync
//	byte 1     : 0xA0 + index (index 0..59, base angle = index*6 degrees)
//	bytes 2-3  : rotation speed, uint16 LE, unit 0.1 RPM
//	bytes 4-39 : 6 measures of 6 bytes each:
//	               intensity uint16 LE,
//	               distance  uint16 LE (mm),
//	               reserved  2 bytes
//	bytes 40-41: checksum, both bytes equal to 0xFF - (sum of bytes 0-39)
//
// Measure j of frame index i is at angle index*6 + j degrees. Sources: the
// ROBOTIS "LDS Basic Specification" (HLS-LFCD2): 5 V, 400 mA (1 A at
// start), 230400 bps, 120 to 3500 mm, 300 +/- 10 RPM, 1 degree resolution,
// 'b' begins and 'e' pauses. The checksum formula was checked on frames
// captured from this robot's lidar.
package lidar

import (
	"bufio"
	"fmt"
	"io"

	"github.com/tarm/serial"
)

const (
	FrameSize        = 42
	SyncByte         = 0xFA
	IndexBase        = 0xA0
	FramesPerScan    = 60
	MeasuresPerFrame = 6

	// Baud is the fixed serial speed of the LDS-01.
	Baud = 230400

	cmdStart = 'b'
	cmdStop  = 'e'

	// MinRangeM / MaxRangeM bound the sensor's usable measurements.
	MinRangeM = 0.12
	MaxRangeM = 3.5
)

// Scan is a full 360 degree lidar revolution in the sensor frame.
// Index is the angle in degrees, following the sensor's rotation.
// A range of 0 means "no valid measurement at this angle".
type Scan struct {
	Ranges      [360]float64 // meters
	Intensities [360]uint16
	RPM         float64
}

// Frame is a single decoded 42-byte packet (6 consecutive degrees).
type Frame struct {
	Index       int // 0..59
	RPM         float64
	Ranges      [MeasuresPerFrame]float64 // meters
	Intensities [MeasuresPerFrame]uint16
}

// Device reads frames from an LDS-01.
type Device struct {
	rw io.ReadWriteCloser
	br *bufio.Reader

	// partial scan being assembled by ReadScan
	acc      Scan
	accSeen  int
	lastIdx  int
	rpmAccum float64
	rpmCount int
}

// Open opens the serial port of the lidar.
func Open(portName string) (*Device, error) {
	p, err := serial.OpenPort(&serial.Config{Name: portName, Baud: Baud})
	if err != nil {
		return nil, fmt.Errorf("lidar: opening %s: %w", portName, err)
	}
	return New(p), nil
}

// New wraps an already-open stream (used by tests and simulators).
func New(rw io.ReadWriteCloser) *Device {
	return &Device{rw: rw, br: bufio.NewReaderSize(rw, 4096), lastIdx: -1}
}

// Start spins the motor and starts the measurement stream.
func (d *Device) Start() error {
	_, err := d.rw.Write([]byte{cmdStart})
	return err
}

// Stop stops the motor.
func (d *Device) Stop() error {
	_, err := d.rw.Write([]byte{cmdStop})
	return err
}

// Close stops the motor then closes the port.
func (d *Device) Close() error {
	d.Stop()
	return d.rw.Close()
}

// ReadFrame reads and decodes the next valid frame, resynchronizing on the
// sync byte as needed.
func (d *Device) ReadFrame() (*Frame, error) {
	buf := make([]byte, FrameSize)
	for {
		b, err := d.br.ReadByte()
		if err != nil {
			return nil, err
		}
		if b != SyncByte {
			continue
		}
		buf[0] = b
		if _, err := io.ReadFull(d.br, buf[1:]); err != nil {
			return nil, err
		}
		idx := int(buf[1]) - IndexBase
		if idx < 0 || idx >= FramesPerScan || !checksumOK(buf) {
			// Not a real frame start (0xFA can appear inside data), or a
			// frame damaged on the line: resync.
			continue
		}
		f := &Frame{
			Index: idx,
			RPM:   float64(uint16(buf[2])|uint16(buf[3])<<8) / 10.0,
		}
		for j := 0; j < MeasuresPerFrame; j++ {
			off := 4 + j*6
			f.Intensities[j] = uint16(buf[off]) | uint16(buf[off+1])<<8
			f.Ranges[j] = float64(uint16(buf[off+2])|uint16(buf[off+3])<<8) / 1000.0
		}
		return f, nil
	}
}

// checksumOK tells whether a 42-byte frame carries the checksum of its
// first 40 bytes (the sensor writes it in byte 40 and again in byte 41).
func checksumOK(f []byte) bool {
	sum := 0
	for _, b := range f[:40] {
		sum += int(b)
	}
	c := byte(0xFF - sum&0xFF)
	return f[40] == c || f[41] == c
}

// ReadScan assembles frames into full revolutions. It returns a scan every
// time the frame index wraps around (about 5 times per second at 300 RPM).
func (d *Device) ReadScan() (*Scan, error) {
	for {
		f, err := d.ReadFrame()
		if err != nil {
			return nil, err
		}

		wrapped := f.Index < d.lastIdx
		d.lastIdx = f.Index

		var done *Scan
		if wrapped && d.accSeen > 0 {
			s := d.acc
			if d.rpmCount > 0 {
				s.RPM = d.rpmAccum / float64(d.rpmCount)
			}
			done = &s
			d.acc = Scan{}
			d.accSeen = 0
			d.rpmAccum = 0
			d.rpmCount = 0
		}

		base := f.Index * MeasuresPerFrame
		for j := 0; j < MeasuresPerFrame; j++ {
			r := f.Ranges[j]
			if r < MinRangeM || r > MaxRangeM {
				r = 0
			}
			d.acc.Ranges[base+j] = r
			d.acc.Intensities[base+j] = f.Intensities[j]
		}
		d.accSeen++
		d.rpmAccum += f.RPM
		d.rpmCount++

		if done != nil {
			return done, nil
		}
	}
}
