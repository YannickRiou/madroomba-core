package main

import (
	"encoding/json"
	"fmt"
	"log"
	"os"
)

// camSettings are the tunable raspivid parameters. They are persisted as
// JSON in <data-dir>/settings.json and edited from the web page.
type camSettings struct {
	Brightness int    `json:"brightness"` // 0..100
	Contrast   int    `json:"contrast"`   // -100..100
	Saturation int    `json:"saturation"` // -100..100
	AWB        string `json:"awb"`        // auto white balance mode
	Exposure   string `json:"exposure"`   // exposure mode
	HFlip      bool   `json:"hflip"`
	VFlip      bool   `json:"vflip"`
}

// Modes accepted by raspivid (-awb / -ex). Settings are validated against
// these lists because they end up on a command line.
var (
	awbModes = []string{"auto", "off", "sun", "cloud", "shade", "tungsten", "fluorescent", "incandescent", "flash", "horizon"}
	expModes = []string{"auto", "night", "nightpreview", "backlight", "spotlight", "sports", "snow", "beach", "verylong", "fixedfps", "antishake", "fireworks"}
)

func defaultSettings() camSettings {
	return camSettings{Brightness: 50, AWB: "auto", Exposure: "auto"}
}

func contains(list []string, v string) bool {
	for _, s := range list {
		if s == v {
			return true
		}
	}
	return false
}

func (c camSettings) valid() error {
	switch {
	case c.Brightness < 0 || c.Brightness > 100:
		return fmt.Errorf("brightness must be 0..100")
	case c.Contrast < -100 || c.Contrast > 100:
		return fmt.Errorf("contrast must be -100..100")
	case c.Saturation < -100 || c.Saturation > 100:
		return fmt.Errorf("saturation must be -100..100")
	case !contains(awbModes, c.AWB):
		return fmt.Errorf("unknown awb mode %q", c.AWB)
	case !contains(expModes, c.Exposure):
		return fmt.Errorf("unknown exposure mode %q", c.Exposure)
	}
	return nil
}

// args are the raspivid flags for these settings.
func (c camSettings) args() []string {
	a := []string{"-br", fmt.Sprint(c.Brightness), "-co", fmt.Sprint(c.Contrast),
		"-sa", fmt.Sprint(c.Saturation), "-awb", c.AWB, "-ex", c.Exposure}
	if c.HFlip {
		a = append(a, "-hf")
	}
	if c.VFlip {
		a = append(a, "-vf")
	}
	return a
}

// loadJSON fills v from path. A missing file is fine; a corrupt one is logged
// and ignored so the daemon still starts with defaults.
func loadJSON(path string, v interface{}) {
	if b, err := os.ReadFile(path); err == nil {
		if err := json.Unmarshal(b, v); err != nil {
			log.Printf("basestationd: ignoring %s: %v", path, err)
		}
	}
}

func saveJSON(path string, v interface{}) {
	b, _ := json.Marshal(v)
	if err := os.WriteFile(path, b, 0o644); err != nil {
		log.Printf("basestationd: saving %s: %v", path, err)
	}
}
