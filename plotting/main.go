package main

import (
	"fmt"
	"strconv"
	
	"github.com/skiesel/expsys/rdb"
)

const (
	dataRoot = "../experiments/data/"
)


func main() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
//		"PPRM (sr=2)" : map[string]string { "planner" : "PlakuRRT", "stateradius" : "2" },
//		"PPRM (sr=4)" : map[string]string { "planner" : "PlakuRRT", "stateradius" : "4" },
		"PPRM (sr=6)" : map[string]string { "planner" : "PlakuRRT", "stateradius" : "6" },

	}


	sr:=6
	ed:=5
	prm:=1000
	// for sr := 2; sr <= 6; sr += 2 {
	// 	for ed := 5; ed <= 10; ed += 5 {
	// 		for prm := 100; prm <= 10000; prm*=10 {
				label := fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
	// 		}
	// 	}
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Quadrotor", "CPU Time", "Solving Time", ".pdf", 5, 5, dss, false)
}