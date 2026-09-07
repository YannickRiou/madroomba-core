// Package robot abstracts the mobile base so the daemon can run either on
// the real Roomba (Open Interface over serial) or on the built-in simulator.
package robot

// Battery is a snapshot of the Roomba battery sensors.
type Battery struct {
	ChargeMAh   uint16 `json:"charge_mah"`
	CapacityMAh uint16 `json:"capacity_mah"`
	VoltageMV   uint16 `json:"voltage_mv"`
	CurrentMA   int16  `json:"current_ma"`
	TempC       int8   `json:"temp_c"`
	// Charging state, OI packet 21: 0 none, 1 reconditioning, 2 full,
	// 3 trickle, 4 waiting, 5 fault.
	Charging byte `json:"charging"`
}

// Driver is the minimal mobile-base contract used by the daemon.
type Driver interface {
	// SetWheels sets left/right wheel velocities in mm/s (-500..500).
	SetWheels(leftMM, rightMM int16) error
	// Odometry returns distance (mm) and angle (degrees, CCW positive)
	// accumulated since the previous call.
	Odometry() (distMM, angleDeg int16, err error)
	Battery() (Battery, error)
	// Clean starts the built-in cleaning behavior (robot goes passive).
	Clean() error
	// Dock sends the robot to its charging base (robot goes passive).
	Dock() error
	// Reactivate puts the robot back under direct control (Safe mode).
	Reactivate() error
	Close() error
}
