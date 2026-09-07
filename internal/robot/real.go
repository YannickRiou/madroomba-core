package robot

import (
	"encoding/binary"
	"fmt"
	"os"
	"sync"
	"time"

	roomba "github.com/YannickRiou/roomba-go-lib"
	"github.com/YannickRiou/roomba-go-lib/constants"
)

// Real drives a physical Roomba 500 through its SCI/OI serial port.
// OI reads and writes are serialized with a mutex: the OI answers requests
// in order on a single line, so concurrent Sensors calls would interleave.
type Real struct {
	mu sync.Mutex
	r  *roomba.Roomba
}

// OpenReal opens the serial port, wakes the robot if a BRC GPIO is given
// (pass -1 to skip), and puts it in Safe mode.
func OpenReal(port string, brcGPIO int) (*Real, error) {
	if brcGPIO >= 0 {
		if err := pulseBRC(brcGPIO); err != nil {
			return nil, fmt.Errorf("BRC wake-up on GPIO%d: %w", brcGPIO, err)
		}
	}
	r, err := roomba.MakeRoomba(port)
	if err != nil {
		return nil, fmt.Errorf("opening roomba on %s: %w", port, err)
	}
	rr := &Real{r: r}
	if err := rr.Reactivate(); err != nil {
		return nil, err
	}
	return rr, nil
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

func (rr *Real) Reactivate() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	if err := rr.r.Start(); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	if err := rr.r.Safe(); err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	return nil
}

func (rr *Real) SetWheels(leftMM, rightMM int16) error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.DirectDrive(rightMM, leftMM)
}

func (rr *Real) Odometry() (int16, int16, error) {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	data, err := rr.r.QueryList([]byte{constants.SENSOR_DISTANCE, constants.SENSOR_ANGLE})
	if err != nil {
		return 0, 0, err
	}
	dist := int16(binary.BigEndian.Uint16(data[0]))
	ang := int16(binary.BigEndian.Uint16(data[1]))
	return dist, ang, nil
}

func (rr *Real) Battery() (Battery, error) {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	data, err := rr.r.QueryList([]byte{
		constants.SENSOR_CHARGING,
		constants.SENSOR_VOLTAGE,
		constants.SENSOR_CURRENT,
		constants.SENSOR_TEMPERATURE,
		constants.SENSOR_BATTERY_CHARGE,
		constants.SENSOR_BATTERY_CAPACITY,
	})
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

func (rr *Real) Clean() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.Clean()
}

func (rr *Real) Dock() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	return rr.r.SeekDock()
}

func (rr *Real) Close() error {
	rr.mu.Lock()
	defer rr.mu.Unlock()
	rr.r.DirectDrive(0, 0)
	// Back to passive so the robot can sleep and charge normally.
	return rr.r.Start()
}
