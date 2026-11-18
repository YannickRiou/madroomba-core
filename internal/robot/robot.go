// Package robot abstracts the mobile base so the daemon can run either on
// the real Roomba (Open Interface over serial) or on the built-in simulator.
package robot

import "errors"

// Battery is a snapshot of the Roomba battery sensors.
type Battery struct {
	ChargeMAh   uint16 `json:"charge_mah"`
	CapacityMAh uint16 `json:"capacity_mah"`
	VoltageMV   uint16 `json:"voltage_mv"`
	CurrentMA   int16  `json:"current_ma"` // negative while running, positive while charging
	TempC       int8   `json:"temp_c"`
	// Charging state, OI packet 21: 0 none, 1 reconditioning, 2 full,
	// 3 trickle, 4 waiting, 5 fault.
	Charging byte `json:"charging"`
}

// Sensors holds every other OI sensor, decoded for a Roomba 500 (Open
// Interface). Fields from packets 43 to 58 are only filled when Extended is
// true: older firmwares answer packets 7 to 42 only.
type Sensors struct {
	OIMode byte `json:"oi_mode"` // packet 35: 0 off, 1 passive, 2 safe, 3 full

	BumpLeft       bool `json:"bump_left"` // packet 7
	BumpRight      bool `json:"bump_right"`
	WheelDropLeft  bool `json:"wheel_drop_left"`
	WheelDropRight bool `json:"wheel_drop_right"`

	Wall       bool   `json:"wall"`        // packet 8
	WallSignal uint16 `json:"wall_signal"` // packet 27, 0-1023
	// Cliffs, left to right: left, front left, front right, right
	// (packets 9 to 12), with their raw signals (28 to 31, 0-4095).
	Cliffs       [4]bool   `json:"cliffs"`
	CliffSignals [4]uint16 `json:"cliff_signals"`
	VirtualWall  bool      `json:"virtual_wall"` // packet 13

	// Overcurrents, packet 14: bit 0 side brush, 2 main brush, 3 right
	// wheel, 4 left wheel.
	Overcurrents byte `json:"overcurrents"`
	DirtDetect   byte `json:"dirt_detect"` // packet 15, 0-255
	IROmni       byte `json:"ir_omni"`     // packet 17, remote or dock code, 0 = none
	// Buttons, packet 18: bit 0 clean, 1 spot, 2 dock, 3 minute, 4 hour,
	// 5 day, 6 schedule, 7 clock.
	Buttons byte `json:"buttons"`
	// Charging sources, packet 34: bit 0 internal charger, 1 home base.
	ChargingSources byte `json:"charging_sources"`
	SongPlaying     bool `json:"song_playing"` // packet 37

	Extended     bool      `json:"extended"`
	EncoderLeft  uint16    `json:"encoder_left"`  // packet 44
	EncoderRight uint16    `json:"encoder_right"` // packet 43
	LightBumper  byte      `json:"light_bumper"`  // packet 45, one bit per sensor, left to right
	LightSignals [6]uint16 `json:"light_signals"` // packets 46 to 51
	IRLeft       byte      `json:"ir_left"`       // packet 52
	IRRight      byte      `json:"ir_right"`      // packet 53
	// Motor currents in mA (packets 54 to 57).
	CurrentLeftMA      int16 `json:"current_left_ma"`
	CurrentRightMA     int16 `json:"current_right_ma"`
	CurrentMainBrushMA int16 `json:"current_main_brush_ma"`
	CurrentSideBrushMA int16 `json:"current_side_brush_ma"`
	Stasis             byte  `json:"stasis"` // packet 58, bit 0 = moving forward
}

// Motors are the cleaning motors (OI opcode 138).
type Motors struct {
	MainBrush bool `json:"main_brush"`
	Vacuum    bool `json:"vacuum"`
	SideBrush bool `json:"side_brush"`
}

// LEDs are the Roomba 500 lights (OI opcode 139). Power color goes from 0
// (green) to 255 (red), intensity from 0 (off) to 255.
type LEDs struct {
	Debris         bool  `json:"debris"`
	Spot           bool  `json:"spot"`
	Dock           bool  `json:"dock"`
	CheckRobot     bool  `json:"check_robot"`
	PowerColor     uint8 `json:"power_color"`
	PowerIntensity uint8 `json:"power_intensity"`
}

// Note is one song note: Pitch is a MIDI note number (31-127, 60 = middle
// C, anything else is a rest) and Dur is in 1/64 s.
type Note struct {
	Pitch uint8 `json:"pitch"`
	Dur   uint8 `json:"dur"`
}

// MaxNotes is the longest song the OI accepts.
const MaxNotes = 16

// Actions accepted by Driver.Action: the OI virtual buttons and modes.
var Actions = []string{"spot", "max", "passive", "full", "power_off"}

// ErrUnknownAction is returned by Driver.Action for a name not in Actions.
var ErrUnknownAction = errors.New("unknown action")

// Driver is the mobile-base contract used by the daemon.
type Driver interface {
	// SetWheels sets left/right wheel velocities in mm/s (-500..500).
	SetWheels(leftMM, rightMM int16) error
	// Odometry returns the distance (m) and rotation (rad, counter-
	// clockwise positive) since the previous call.
	Odometry() (distM, angleRad float64, err error)
	Battery() (Battery, error)
	Sensors() (Sensors, error)
	// Clean starts the built-in cleaning behavior (robot goes passive).
	Clean() error
	// Dock sends the robot to its charging base (robot goes passive).
	Dock() error
	// Reactivate puts the robot back under direct control (Safe mode).
	Reactivate() error
	// Action runs one of Actions.
	Action(name string) error
	SetMotors(Motors) error
	SetLEDs(LEDs) error
	// PlaySong stores the notes (at most MaxNotes) and plays them.
	PlaySong([]Note) error
	Close() error
}
