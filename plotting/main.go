package main

import (
	"bufio"
	"fmt"
	"io/ioutil"
	"os"
	"sort"
	"strconv"
	"strings"

	"github.com/gonum/plot"
    "github.com/gonum/plot/plotter"
    "github.com/gonum/plot/vg"
)

const (
	dataRoot = "../data/"
)

type Experiment map[string]*DataCollection

type DataCollection struct {
	Name string
	Params map[string]string
	DataPoints []DataPoint
}

type DataPoint struct {
	Solved bool
	Length float64
	Time float64
}

func parseResultLine(line string) DataPoint {
	values := strings.Split(line, ";")

	length, err := strconv.ParseFloat(strings.TrimSpace(values[7]), 64)
	if err != nil {
		panic(err)
	}
	statusInt, err := strconv.ParseInt(strings.TrimSpace(values[10]), 10, 64)
	if err != nil {
		panic(err)
	}
	solved :=  statusInt == 6
	time, err := strconv.ParseFloat(strings.TrimSpace(values[11]), 64)
	if err != nil {
		panic(err)
	}

	return DataPoint{ Solved : solved, Length : length, Time : time }
}

func parseParams(data []string, keys []string) map[string]string {
	params := map[string]string{}
	for _, line := range(data) {
		for _, key := range(keys) {
			if strings.Contains(line, key) {
				params[key] = strings.Split(line, " = ")[1]
			}
		}
	}
	return params
}

func parseFBiasedRRTParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius"}
	return parseParams(data, keys)
}

func parseFBiasedRRTShellParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius", "shell_preference", "shell_depth"}
	return parseParams(data, keys)
}

func parsePlakuRRTParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius", "alpha", "b"}
	return parseParams(data, keys)
}

func parseAstarParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius", "weight"}
	return parseParams(data, keys)
}

func parseDijkstraParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius"}
	return parseParams(data, keys)
}

func parseGreedyParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius"}
	return parseParams(data, keys)
}

func parseSpeedyParams(data []string) map[string]string {
	keys := []string{"num_prm_edges", "omega", "prm_size", "state_radius", "use_d_or_e"}
	return parseParams(data, keys)
}

func mungeParams(planner string, params map[string]string) string {
	keys := sort.StringSlice{}
	for k := range params {
    	keys = append(keys, k)
	}

	sort.Sort(keys)

	munged := planner

	for _, k := range keys {
		munged += "_" + k + ":" + params[k]
	}

	return munged
}

func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment *Experiment) {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel

	w := vg.Points(20)

	plotters := []plot.Plotter{}	
	labels := []string{}
	i := 0.
	for _, algorithm := range *experiment {
		data := plotter.Values{}
		skip := false
		for _, point := range algorithm.DataPoints {
			if !point.Solved {
				skip = true
				break
			}

			if key == "Length" {
				data = append(data, point.Length)
			} else {
				data = append(data, point.Time)
			}
		}

		if skip {
			continue
		}

		box, err := plotter.NewBoxPlot(w, i, data)
		if err != nil {
			panic(err)
		}

		plotters = append(plotters, box)
		labels = append(labels, strings.Replace(algorithm.Name, " ", "\n", -1))
		i++
	}

	p.Add(plotters...)
    p.NominalX(labels...)

    filename := strings.Replace(title, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

    if err := p.Save(vg.Length(width) * vg.Points(15 * float64(len(*experiment))), vg.Length(height) * vg.Inch, filename); err != nil {
        panic(err)
    }
}

func main() {
	files, err := ioutil.ReadDir(dataRoot)
	if err != nil {
		panic(err)
	}

	mappedData := map[string]*Experiment{}

	for _, file := range files {
		openFile, err := os.Open(dataRoot + file.Name())
		if err != nil {
			fmt.Println(err)
			continue
		}

		scanner := bufio.NewScanner(openFile)
		data := []string{}
		experiment := ""
		capture := false
		for scanner.Scan() {
			str := scanner.Text()

			if strings.HasPrefix(str, "Experiment ") {
				experiment = strings.Split(str, " ")[1]
			}

			if capture {
				data = append(data, str)
			}

			capture = capture || (str == "1 planners")
		}

		plannerName := strings.Replace(data[0], "control_", "", -1)

		var params map[string]string
		if plannerName == "FBiased RRT" {
			params = parseFBiasedRRTParams(data)
		} else if plannerName == "FBiased RRT Shell" {
			params = parseFBiasedRRTShellParams(data)
		} else if plannerName == "Plaku RRT" {
			params = parsePlakuRRTParams(data)
		} else if plannerName == "Dijkstra" {
			params = parseDijkstraParams(data)
		} else if plannerName == "A*" {
			params = parseAstarParams(data)
			if params["weight"] != "1" {
				weight, _ := strconv.ParseFloat(params["weight"], 64)
				plannerName = fmt.Sprintf("WA* (w=%.2f)", weight)
			}
		} else if plannerName == "Speedy" {
			params = parseSpeedyParams(data)
			plannerName = fmt.Sprintf("Speedy (%s)", params["use_d_or_e"])
		} else if plannerName == "Greedy" {
			params = parseGreedyParams(data)
		}

		dataPoint := parseResultLine(data[len(data) - 2])

		munged := mungeParams(plannerName, params)

		if _, ok := mappedData[experiment]; !ok {
			mappedData[experiment] = &Experiment{}
		}

		experimentData := *mappedData[experiment]

		if collection, ok := experimentData[munged]; ok {
			collection.DataPoints = append(collection.DataPoints, dataPoint)
		} else {
			experimentData[munged] = &DataCollection{
				Name : plannerName,
				Params : params,
				DataPoints : []DataPoint{dataPoint},
			}
		}

		openFile.Close()
	}


//func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment *Experiment) {
	for domain, experiment := range mappedData {
		makeBoxPlot(domain, "Time (sec)", "Time", ".pdf", 4, 4, experiment)
		makeBoxPlot(domain, "Solution Length", "Length", ".pdf", 4, 4, experiment)
		// makeBoxPlot(domain, "\% Solved", "Length", ".pdf", 4, 14, experiment)

	}
}

/*
0 approximate solution BOOLEAN
1 correct solution BOOLEAN
2 graph motions INTEGER
3 graph states INTEGER
4 memory REAL
5 solution clearance REAL
6 solution difference REAL
7 solution length REAL
8 solution segments INTEGER
9 solved BOOLEAN
10 status ENUM
11 time REAL
12 valid segment fraction REAL

0 Unknown status
1 Invalid start
2 Invalid goal
3 Unrecognized goal type
4 Timeout
5 Approximate solution
6 Exact solution
7 Crash
8 Unknown status
*/