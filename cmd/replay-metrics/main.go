// Command replay-metrics runs a deterministic airborne-style replay under the
// snapshot-only and persistence-window policies and prints comparative metrics.
//
// It is a local analysis tool, not part of the server API.
package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"math"
	"os"

	"gnss-integrity/internal/monitor"
	"gnss-integrity/internal/profile"
	"gnss-integrity/internal/sim"
)

type metrics struct {
	epochs      int
	firstAlarm  int64
	alarms      int
	falseAlarms int
	available   int
}

func main() {
	seed := flag.Uint64("seed", 20260930, "RNG seed")
	epochs := flag.Int("epochs", 600, "epochs to replay")
	sigma := flag.Float64("sigma", 0.6, "pseudorange noise standard deviation (m)")
	faultStart := flag.Int("fault-start", 201, "one-based fault start epoch; 0 disables")
	faultEnd := flag.Int("fault-end", 300, "one-based fault end epoch")
	satellites := flag.Int("satellites", 5, "visible satellites in the replay")
	bias := flag.Float64("bias", 120, "injected pseudorange bias in metres")
	hal := flag.Float64("hal", 15, "horizontal alert limit in metres for this analysis")
	burstsEvery := flag.Int("false-burst-every", 60, "inject one-epoch false bias bursts at this interval; 0 disables")
	burstBias := flag.Float64("false-burst-bias", 90, "one-epoch false bias in metres")
	out := flag.String("csv", "", "optional CSV output path")
	flag.Parse()

	snapshotProfile := profile.Builtins()["terminal"]
	snapshotProfile.HorizontalAlertLimit = *hal
	snapshotProfile.Name = "snapshot-analysis"
	snapshotProfile.AlarmEpochs = 1
	snapshotProfile.ClearAlarmEpochs = 1
	snapshotProfile.RecoveryEpochs = 1
	persistentProfile := snapshotProfile
	persistentProfile.Name = "persistent-analysis"
	persistentProfile.AlarmEpochs = 3
	persistentProfile.ClearAlarmEpochs = 3
	persistentProfile.RecoveryEpochs = 5

	snapshotReplay := run(snapshotProfile, *epochs, *seed, *sigma, *faultStart, *faultEnd, *bias, *satellites, *burstsEvery, *burstBias)
	persistentReplay := run(persistentProfile, *epochs, *seed, *sigma, *faultStart, *faultEnd, *bias, *satellites, *burstsEvery, *burstBias)

	fmt.Printf("seed=%d epochs=%d satellites=%d sigma=%.2fm fault=[%d,%d] bias=%.0fm\n",
		*seed, *epochs, *satellites, *sigma, *faultStart, *faultEnd, *bias)
	fmt.Printf("%-24s %12s %12s %12s %12s\n", "policy", "firstAlarm", "alarms", "falseAlarms", "availability")
	printMetrics("snapshot(1 epoch)", snapshotReplay)
	printMetrics("persistent(3 alarm)", persistentReplay)
	if *out != "" {
		if err := writeCSV(*out, snapshotReplay, persistentReplay); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
	}
}

func run(p profile.Profile, n int, seed uint64, sigma float64, faultStart, faultEnd int, bias float64, satelliteCount, burstsEvery int, burstBias float64) metrics {
	sky := sim.DefaultSky()
	if satelliteCount < len(sky) {
		sky = sky[:satelliteCount]
	}
	st := monitor.NewState(p)
	m := metrics{epochs: n, firstAlarm: math.MaxInt64}
	rng := sim.NewRNG(seed)
	for i := 1; i <= n; i++ {
		biasPRN, thisBias := 0, 0.0
		var faulty *int
		if faultStart > 0 && i >= faultStart && i <= faultEnd {
			biasPRN, thisBias = 4, bias
			prn := 4
			faulty = &prn
		} else if burstsEvery > 0 && i%burstsEvery == 0 {
			// One-epoch multipath-like burst. It is deliberately not declared as a
			// true fault, so alarms raised from it count as false alarms.
			biasPRN = 1 + (i/burstsEvery)%satelliteCount
			thisBias = burstBias
		}
		e := monitor.Epoch{
			Timestamp:    int64(i),
			Measurements: sim.Measurements(sky, sigma, 0, biasPRN, thisBias, rng),
			Initial:      sim.ApproximatePosition(),
			FaultyPRN:    faulty,
		}
		r, err := st.Process(e)
		if err != nil {
			panic(err)
		}
		if r.AlarmRaised {
			m.alarms++
			if int64(i) < m.firstAlarm {
				m.firstAlarm = int64(i)
			}
		}
		m.falseAlarms = st.Stats.FalseAlarmCount
		if !r.AlarmActive {
			m.available++
		}
	}
	if m.firstAlarm == math.MaxInt64 {
		m.firstAlarm = 0
	}
	return m
}

func printMetrics(name string, m metrics) {
	avail := 100 * float64(m.available) / float64(m.epochs)
	fmt.Printf("%-24s %12d %12d %12d %11.2f%%\n", name, m.firstAlarm, m.alarms, m.falseAlarms, avail)
}

func writeCSV(path string, rows ...metrics) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := csv.NewWriter(f)
	defer w.Flush()
	if err := w.Write([]string{"policy", "first_alarm_epoch", "alarm_count", "false_alarm_count", "availability"}); err != nil {
		return err
	}
	for i, m := range rows {
		name := "snapshot"
		if i == 1 {
			name = "persistent"
		}
		if err := w.Write([]string{
			name,
			fmt.Sprint(m.firstAlarm),
			fmt.Sprint(m.alarms),
			fmt.Sprint(m.falseAlarms),
			fmt.Sprintf("%.6f", float64(m.available)/float64(m.epochs)),
		}); err != nil {
			return err
		}
	}
	return nil
}
