package main

import (
	"net/http"
	"strconv"
	"time"
)

// historyPoints caps the samples sent to the page for one chart.
const historyPoints = 1000

// registerBattery serves GET /api/battery?hours=N: the history of the
// last N hours (24 by default, at most 7 days) as columns, plus the
// lifetime counters.
func (a *app) registerBattery() {
	a.srv.Handle("/api/battery", func(w http.ResponseWriter, r *http.Request) {
		hours, err := strconv.ParseFloat(r.URL.Query().Get("hours"), 64)
		if err != nil || hours <= 0 {
			hours = 24
		}
		if hours > 7*24 {
			hours = 7 * 24
		}
		samples := a.battery.Since(time.Now().Add(-time.Duration(hours*float64(time.Hour))), historyPoints)
		cols := struct {
			T        []int64   `json:"t"`        // unix seconds
			Pct      []float64 `json:"pct"`      // state of charge, %
			MV       []uint16  `json:"mv"`       // voltage
			MA       []int16   `json:"ma"`       // mean current, negative when discharging
			Charging []byte    `json:"charging"` // OI charging state
			TempC    []int8    `json:"temp_c"`
		}{}
		for _, s := range samples {
			cols.T = append(cols.T, s.Time.Unix())
			cols.Pct = append(cols.Pct, float64(int(s.Percent()*10))/10)
			cols.MV = append(cols.MV, s.VoltageMV)
			cols.MA = append(cols.MA, s.CurrentMA)
			cols.Charging = append(cols.Charging, s.Charging)
			cols.TempC = append(cols.TempC, s.TempC)
		}
		writeJSON(w, map[string]interface{}{"history": cols, "lifetime": a.lifetime.Get()})
	})
}
