package main

import (
	// "fmt"
	"strconv"
	
	"github.com/skiesel/expsys/rdb"
)

const (
	dataRoot = "../experiments/data/"
	plottype = ".pdf"
	plotWidth = 5.
	plotHeight = 5.
)

func main() {
	// Quadrotor()
	// Blimp()
	// DynamicCar()
	// KinematicCar()
	// Hovercraft()

	Anytime()
}

func Anytime() {
	filters := map[string]map[string]string {
		"SST" : map[string]string { "planner" : "SST" },
		"SST*" : map[string]string { "planner" : "SSTStar" },
		"Restarting RRT with Pruning" : map[string]string { "planner" : "RestartingRRTWithPruning" },
	}

	// // for sr := 2; sr <= 6; sr += 2 {
	// // 	for ed := 5; ed <= 10; ed += 5 {
	// // 		for prm := 1000; prm <= 10000; prm*=10 {

	// sr := 4
	// ed := 5
	// prm := 1000

	// 			label := fmt.Sprintf("A-BEAST sr=%d ed=%d prm=%d", sr, ed, prm)
	// 			filters[label] = map[string]string {
	// 				"planner" : "AnytimeBeast",
	// 				"stateradius" : strconv.Itoa(sr),
	// 				"numprmedges" : strconv.Itoa(ed),
	// 				"prmsize" : strconv.Itoa(prm),
	// 			}
	// // 		}
	// // 	}
	// // }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "StraightLine"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonAnytimeRDBReader)
	makeAnytimePlot(dss, "StraightLine", ".", "solution", "solution time", "solution cost", "CPU Time", "Solution Quality", plottype, 0, 60, 15, plotWidth, plotHeight)
}

func Quadrotor() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
	}

	// for sr := 2; sr <= 6; sr += 2 {
		// for ed := 5; ed <= 10; ed += 5 {
			// for prm := 1000; prm <= 10000; prm*=10 {

	sr := 6
	ed := 5
	prm := 1000

				label := "BEAST"//fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}

	sr = 6
	ed = 10
	prm = 10000

				label = "P-PRM"//fmt.Sprintf("PPRM\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters["P-PRM"] = map[string]string {
					"planner" : "PlakuRRT",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
			// }
		// }
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "Quadrotor"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Quadrotor", "CPU Time", "Solving Time", plottype, plotWidth, plotHeight, dss, false, false)
}

func Blimp() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
	}

	// for sr := 2; sr <= 6; sr += 2 {
		// for ed := 5; ed <= 10; ed += 5 {
			// for prm := 1000; prm <= 10000; prm*=10 {

	sr := 6
	ed := 5
	prm := 1000

				label := "BEAST"//fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}

	sr = 6
	ed = 10
	prm = 10000

				label = "P-PRM"//fmt.Sprintf("PPRM\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "PlakuRRT",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
			// }
		// }
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "Blimp"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Blimp", "CPU Time", "Solving Time", plottype, plotWidth, plotHeight, dss, false, false)
}

func DynamicCar() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
	}

	// for sr := 2; sr <= 6; sr += 2 {
		// for ed := 5; ed <= 10; ed += 5 {
			// for prm := 100; prm <= 10000; prm*=10 {

	sr := 4
	ed := 5
	prm := 100

				label := "BEAST"//fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}

	sr = 4
	ed = 10
	prm = 10000

				label = "P-PRM"//fmt.Sprintf("P-PRM\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "PlakuRRT",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
			// }
		// }
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "DynamicCar"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Dynamic Car", "CPU Time", "Solving Time", plottype, plotWidth, plotHeight, dss, false, false)
}

func KinematicCar() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
	}

	// for sr := 2; sr <= 6; sr += 2 {
		// for ed := 5; ed <= 10; ed += 5 {
			// for prm := 100; prm <= 10000; prm*=10 {

	sr := 4
	ed := 5
	prm := 100

				label := "BEAST"//fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}

	sr = 4
	ed = 10
	prm = 10000

				label = "P-PRM"//fmt.Sprintf("P-PRM\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "PlakuRRT",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
			// }
		// }
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "KinematicCar"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Kinematic Car", "CPU Time", "Solving Time", plottype, plotWidth, plotHeight, dss, false, false)
}

func Hovercraft() {
	filters := map[string]map[string]string {
		"RRT" : map[string]string { "planner" : "RRT" },
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
	}

	// for sr := 2; sr <= 6; sr += 2 {
		// for ed := 5; ed <= 10; ed += 5 {
			// for prm := 100; prm <= 10000; prm*=10 {

	sr := 6
	ed := 5
	prm := 1000

				label := "BEAST"//fmt.Sprintf("New\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "NewPlanner",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}

	sr = 6
	ed = 10
	prm = 10000

				label = "P-PRM"//fmt.Sprintf("P-PRM\nsr=%d\ned=%d\nprm=%d", sr, ed, prm)
				filters[label] = map[string]string {
					"planner" : "PlakuRRT",
					"stateradius" : strconv.Itoa(sr),
					"numprmedges" : strconv.Itoa(ed),
					"prmsize" : strconv.Itoa(prm),
				}
			// }
		// }
	// }

	for key := range filters {
		filters[key]["timeout"] = "60"
		filters[key]["domain"] = "Hovercraft"
	}

	dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonRDBReader)

	makeBoxPlot("Hovercraft", "CPU Time", "Solving Time", plottype, plotWidth, plotHeight, dss, false, false)
}