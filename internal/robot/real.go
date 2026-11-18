package robot

import (
	"encoding/binary"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"sync"
	"time"

	roomba "github.com/YannickRiou/roomba-go-lib"
	"github.com/tarm/serial"
)

// readTimeout bounds every OI read. The library reads with no timeout, so a
// Roomba that stops answering (asleep, unplugged) blocked a read forever
// while holding mu, and every other command, Safe mode included, waited.
const readTimeout = 300 * time.Millisecond

// Wheel geometry for encoder odometry: 72 mm wheels, 508.8 encoder counts
// per wheel turn. The Roomba 500 OI spec does not give them; these are the
// values of the iRobot Create 2, built on the same chassis.
const mmPerCount = math.Pi * 72.0 / 508.8

// packetLen is the reply size of each OI sensor packet (Roomba 500 Open
// Interface, packets 7 to 58). The library only knows packets up to 40.
var packetLen = map[byte]int{
	7: 1, 8: 1, 9: 1, 10: 1, 11: 1, 12: 1, 13: 1, 14: 1, 15: 1, 16: 1, 17: 1, 18: 1,
	19: 2, 20: 2, 21: 1, 22: 2, 23: 2, 24: 1, 25: 2, 26: 2,
	27: 2, 28: 2, 29: 2, 30: 2, 31: 2, 32: 1, 33: 2,
	34: 1, 35: 1, 36: 1, 37: 1, 38: 1, 39: 2, 40: 2, 41: 2, 42: 2,
	43: 2, 44: 2, 45: 1, 46: 2, 47: 2, 48: 2, 49: 2, 50: 2, 51: 2,
	52: 1, 53: 1, 54: 2, 55: 2, 56: 2, 57: 2, 58: 1,
}

// OI opcodes used directly (the library logs every write it makes).
const (
	opStart    = 128
	opSafe     = 131
	opFull     = 132
	opPower    = 133
	opSpot     = 134
	opMax      = 136
	opMotors   = 138
	opLEDs     = 139
	opSong     = 140
	opPlay     = 141
	opQuery    = 149
	songNumber = 3 // the Roomba 500 keeps 4 songs (0-3); 3 is ours
)

// Real drives a physical Roomba 500 through its SCI/OI serial port.
// OI reads and writes are serialized with a mutex: the OI answers requests
// in order on a single line, so concurrent queries would interleave.
type Real struct {
	mu         sync.Mutex
	r          *roomba.Roomba
	port       *serial.Port
	brc        int
	wheelBaseM float64

	// ext tells whether the robot answers packets 43 to 58: 0 not known
	// yet, 1 yes, -1 no (odometry then falls back to packets 19 and 20).
	ext            int
	encInit        bool
	encLeft, encRt uint16
}

// OpenReal opens the serial port and puts the robot in Safe mode, waking it
// first if a BRC GPIO is given (pass -1 to skip). On the dock or the
// charger it stays in Passive mode instead: the Roomba does not charge in
// Safe mode. wheelBaseM is the distance between the wheels, used by encoder
// odometry. A robot that does not answer is only logged: the daemon keeps
// running and Safe mode can be requested again from the page.
func OpenReal(port string, brcGPIO int, wheelBaseM float64) (*Real, error) {
	p, err := serial.OpenPort(&serial.Config{Name: port, Baud: 115200, ReadTimeout: readTimeout})
	if err != nil {
		return nil, fmt.Errorf("opening roomba on %s: %w", port, err)
	}
	r := &roomba.Roomba{PortName: port, S: p, StreamPaused: make(chan bool, 1)}
	rr := &Real{r: r, port: p, brc: brcGPIO, wheelBaseM: wheelBaseM}
	if rr.charging() {
		log.Println("madroombad: on the dock or charger, staying in Passive mode so the battery charges (Mode commande to drive)")
		return rr, nil
	}
	if err := rr.Reactivate(); err != nil {
		log.Printf("madroombad: %v", err)
	}
	return rr, nil
}

// charging starts the OI in Passive mode and tells whether the dock or the
// internal charger is powering the robot (packet 34).
func (rr *Real) charging() bool {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	if rr.send(opStart) != nil {
		return false
	}
	time.Sleep(50 * time.Millisecond)
	data, err := rr.query(34)
	return err == nil && data[0][0] != 0
}

// send writes raw OI bytes. Caller holds mu.
func (rr *Real) send(b ...byte) error {
	_, err := rr.port.Write(b)
	return err
}

// query asks for several sensor packets at once (opcode 149) and returns
// each packet's bytes. Caller holds mu. After a failed read it drops
// whatever arrives late, so the next query is not misread.
func (rr *Real) query(ids ...byte) ([][]byte, error) {
	total := 0
	for _, id := range ids {
		n, ok := packetLen[id]
		if !ok {
			return nil, fmt.Errorf("unknown OI packet %d", id)
		}
		total += n
	}
	if err := rr.send(append([]byte{opQuery, byte(len(ids))}, ids...)...); err != nil {
		return nil, err
	}
	buf := make([]byte, total)
	if _, err := io.ReadFull(rr.port, buf); err != nil {
		rr.port.Flush()
		return nil, fmt.Errorf("reading OI packets %v: %w", ids, err)
	}
	out := make([][]byte, len(ids))
	for i, id := range ids {
		out[i], buf = buf[:packetLen[id]], buf[packetLen[id]:]
	}
	return out, nil
}

// extended reports whether packets 43 to 58 are available, probing once.
// Caller holds mu.
func (rr *Real) extended() (bool, error) {
	if rr.ext != 0 {
		return rr.ext > 0, nil
	}
	if _, err := rr.query(35); err != nil {
		return false, err // robot silent: try again later
	}
	if _, err := rr.query(43, 44); err == nil {
		rr.ext = 1
		log.Println("madroombad: wheel encoders (packets 43-58) available, using them for odometry")
	} else {
		rr.ext = -1
		log.Println("madroombad: packets 43-58 not supported, odometry uses packets 19 and 20")
	}
	return rr.ext > 0, nil
}

// pulseBRC toggles the Baud Rate Change pin low for 100ms, which wakes a
// sleeping Roomba. Uses the legacy sysfs GPIO interface, still available
// on Raspberry Pi OS.
func pulseBRC(pin int) error {
	base := fmt.Sprintf("/sys/class/gpio/gpio%d", pin)
	if _, err := os.Stat(base); os.IsNotExist(err) {
		if err := os.WriteFile("/sys/class/gpio/export", []byte(fmt.Sprint(pin)), 0644); err != nil {
			return err
		}
		time.Sleep(100 * time.Millisecond)
	}
	if err := os.WriteFile(base+"/direction", []byte("out"), 0644); err != nil {
		return err
	}
	if err := os.WriteFile(base+"/value", []byte("0"), 0644); err != nil {
		return err
	}
	time.Sleep(100 * time.Millisecond)
	if err := os.WriteFile(base+"/value", []byte("1"), 0644); err != nil {
		return err
	}
	time.Sleep(500 * time.Millisecond)
	return nil
}

// Reactivate puts the OI back in Safe mode, the mode that accepts drive
// commands. It wakes the robot through BRC when that pin is wired, then
// reads the OI mode back, so a sleeping robot is reported, not ignored.
func (rr *Real) Reactivate() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	if rr.brc >= 0 {
		if err := pulseBRC(rr.brc); err != nil {
			return fmt.Errorf("BRC wake-up on GPIO%d: %w", rr.brc, err)
		}
	}
	if err := rr.send(opStart); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	if err := rr.send(opSafe); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	data, err := rr.query(35)
	if err != nil {
		return fmt.Errorf("no answer from the Roomba, it may be asleep: press Clean on the robot, or wire BRC and set -brc-gpio (%v)", err)
	}
	if mode := data[0][0]; mode != 2 {
		return fmt.Errorf("Roomba in OI mode %d instead of 2 (Safe)", mode)
	}
	return nil
}

// SetWheels drives each wheel at a speed in mm/s (OI Drive Direct, opcode 145).
func (rr *Real) SetWheels(leftMM, rightMM int16) error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.DirectDrive(rightMM, leftMM)
}

// Odometry uses the raw wheel encoders when the robot has them: packet 20
// only counts whole degrees, which loses most of a slow turn. Otherwise it
// falls back to packets 19 (mm) and 20 (degrees).
func (rr *Real) Odometry() (float64, float64, error) {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	ext, err := rr.extended()
	if err != nil {
		return 0, 0, err
	}
	if !ext {
		data, err := rr.query(19, 20)
		if err != nil {
			return 0, 0, err
		}
		dist := float64(int16(binary.BigEndian.Uint16(data[0]))) / 1000
		ang := float64(int16(binary.BigEndian.Uint16(data[1]))) * math.Pi / 180
		return dist, ang, nil
	}
	data, err := rr.query(43, 44) // right, then left
	if err != nil {
		return 0, 0, err
	}
	r, l := binary.BigEndian.Uint16(data[0]), binary.BigEndian.Uint16(data[1])
	dl := float64(int16(l-rr.encLeft)) * mmPerCount / 1000
	dr := float64(int16(r-rr.encRt)) * mmPerCount / 1000
	first := !rr.encInit
	rr.encLeft, rr.encRt, rr.encInit = l, r, true
	// First reading, or counters reset by a reboot of the robot: a wheel
	// cannot travel 10 cm between two polls at 10 Hz.
	if first || math.Abs(dl) > 0.1 || math.Abs(dr) > 0.1 {
		return 0, 0, nil
	}
	return (dl + dr) / 2, (dr - dl) / rr.wheelBaseM, nil
}

// Battery reads the charging state, voltage, current, temperature, charge and capacity (packets 21 to 26).
func (rr *Real) Battery() (Battery, error) {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	data, err := rr.query(21, 22, 23, 24, 25, 26)
	if err != nil {
		return Battery{}, err
	}
	return Battery{
		Charging:    data[0][0],
		VoltageMV:   binary.BigEndian.Uint16(data[1]),
		CurrentMA:   int16(binary.BigEndian.Uint16(data[2])),
		TempC:       int8(data[3][0]),
		ChargeMAh:   binary.BigEndian.Uint16(data[4]),
		CapacityMAh: binary.BigEndian.Uint16(data[5]),
	}, nil
}

// Sensors reads every other sensor (see the Sensors type), packets 43 to 58
// included when the robot has them.
func (rr *Real) Sensors() (Sensors, error) {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	var s Sensors
	d, err := rr.query(7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 27, 28, 29, 30, 31, 34, 35, 37)
	if err != nil {
		return s, err
	}
	u16 := func(b []byte) uint16 { return binary.BigEndian.Uint16(b) }
	s.BumpRight = d[0][0]&1 != 0
	s.BumpLeft = d[0][0]&2 != 0
	s.WheelDropRight = d[0][0]&4 != 0
	s.WheelDropLeft = d[0][0]&8 != 0
	s.Wall = d[1][0] != 0
	for i := 0; i < 4; i++ {
		s.Cliffs[i] = d[2+i][0] != 0
		s.CliffSignals[i] = u16(d[12+i])
	}
	s.VirtualWall = d[6][0] != 0
	s.Overcurrents = d[7][0]
	s.DirtDetect = d[8][0]
	s.IROmni = d[9][0]
	s.Buttons = d[10][0]
	s.WallSignal = u16(d[11])
	s.ChargingSources = d[16][0]
	s.OIMode = d[17][0]
	s.SongPlaying = d[18][0] != 0

	if ext, _ := rr.extended(); !ext {
		return s, nil
	}
	e, err := rr.query(43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58)
	if err != nil {
		return s, nil // keep the base sensors
	}
	s.Extended = true
	s.EncoderRight, s.EncoderLeft = u16(e[0]), u16(e[1]) // packet 43 is the right wheel
	s.LightBumper = e[2][0]
	for i := 0; i < 6; i++ {
		s.LightSignals[i] = u16(e[3+i])
	}
	s.IRLeft, s.IRRight = e[9][0], e[10][0]
	s.CurrentLeftMA = int16(u16(e[11]))
	s.CurrentRightMA = int16(u16(e[12]))
	s.CurrentMainBrushMA = int16(u16(e[13]))
	s.CurrentSideBrushMA = int16(u16(e[14]))
	s.Stasis = e[15][0]
	return s, nil
}

// Clean starts the built-in cleaning cycle (opcode 135); the robot goes Passive.
func (rr *Real) Clean() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.Clean()
}

// Dock makes the Roomba seek its dock (opcode 143); the robot goes Passive.
func (rr *Real) Dock() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.SeekDock()
}

// Action runs an OI virtual button or mode change. Spot, max and power_off
// leave the robot in Passive mode; full turns off the cliff and wheel-drop
// safety.
func (rr *Real) Action(name string) error {
	op := map[string]byte{"spot": opSpot, "max": opMax, "passive": opStart, "full": opFull, "power_off": opPower}[name]
	if op == 0 {
		return ErrUnknownAction
	}
	rr.mu.Lock()
	defer rr.mu.Unlock()
	if name == "full" {
		// Full is only accepted from Safe mode.
		if err := rr.send(opSafe); err != nil {
			return err
		}
		time.Sleep(20 * time.Millisecond)
	}
	return rr.send(op)
}

// SetMotors switches the main brush, vacuum and side brush (opcode 138, Safe or Full mode).
func (rr *Real) SetMotors(m Motors) error {
	var b byte
	if m.SideBrush {
		b |= 1
	}
	if m.Vacuum {
		b |= 2
	}
	if m.MainBrush {
		b |= 4
	}
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.send(opMotors, b)
}

// SetLEDs sets the debris, spot, dock and check robot LEDs and the power LED (opcode 139).
func (rr *Real) SetLEDs(l LEDs) error {
	var b byte
	if l.Debris {
		b |= 1
	}
	if l.Spot {
		b |= 2
	}
	if l.Dock {
		b |= 4
	}
	if l.CheckRobot {
		b |= 8
	}
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.send(opLEDs, b, l.PowerColor, l.PowerIntensity)
}

// PlaySong stores the notes as song 3 (opcode 140) and plays it (opcode 141, Safe or Full mode).
func (rr *Real) PlaySong(notes []Note) error {
	if len(notes) == 0 || len(notes) > MaxNotes {
		return fmt.Errorf("a song has 1 to %d notes, got %d", MaxNotes, len(notes))
	}
	cmd := []byte{opSong, songNumber, byte(len(notes))}
	for _, n := range notes {
		cmd = append(cmd, n.Pitch, n.Dur)
	}
	rr.mu.Lock()
	defer rr.mu.Unlock()
	if err := rr.send(cmd...); err != nil {
		return err
	}
	time.Sleep(20 * time.Millisecond)
	return rr.send(opPlay, songNumber)
}

// Close stops the wheels and leaves the robot in Passive mode.
func (rr *Real) Close() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	rr.r.DirectDrive(0, 0)
	// Back to passive so the robot can sleep and charge normally.
	return rr.send(opStart)
}
