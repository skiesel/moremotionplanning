package main

import (
	// "bufio"
	// "fmt"
	// "io/ioutil"
	// "math"
	// "os"
	// "sort"
	// "strconv"
	// "strings"

	
	"github.com/skiesel/expsys/rdb"
)

const (
	dataRoot = "../experiments/data/"
)


func main() {
	dss := map[string]map[string]string{}
	rdb.GetDatasetsFromNonRDBFormat(dataRoot, dss, true, nonRDBReader)

	// 	useStackedBoxes := false
	// 	if time, ok := params["sampler_initialization_time"]; ok {
	// 		dataPoint.Precomputation, _ = strconv.ParseFloat(time, 64)
	// 		useStackedBoxes = true
	// 	}

	// 	munged := mungeParams(plannerName, params)

	// 	if _, ok := mappedData[experiment]; !ok {
	// 		mappedData[experiment] = &Experiment{}
	// 	}

	// 	experimentData := *mappedData[experiment]

	// 	if collection, ok := experimentData[munged]; ok {
	// 		collection.DataPoints = append(collection.DataPoints, dataPoint)
	// 	} else {
	// 		experimentData[munged] = &DataCollection{
	// 			Name:            plannerName,
	// 			Params:          params,
	// 			DataPoints:      []DataPoint{dataPoint},
	// 			UseStackedBoxes: useStackedBoxes,
	// 		}
	// 	}

	// //func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment *Experiment) {
	// for domain, experiment := range mappedData {
	// 	makeBoxPlot(domain, "Time (log10 sec)", "Time", ".pdf", 4, 4, experiment, true)
	// 	// makeBoxPlot(domain, "Solution Length", "Length", ".pdf", 4, 4, experiment)
	// 	// makeBoxPlot(domain, "\% Solved", "Length", ".pdf", 4, 14, experiment)

	// }
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
