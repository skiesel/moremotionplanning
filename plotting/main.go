package main

import (
	"fmt"
	"strings"
	"strconv"
	"os"
	
	// "github.com/skiesel/expsys/plots"
	"github.com/skiesel/expsys/rdb"
)

const (
	dataRoot = "../experiments/data/"
	plottype = ".eps"
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
		"KPIECE" : map[string]string { "planner" : "KPIECE" },
		"P-PRM" : map[string]string { "planner" : "PlakuRRT" },
		"SST" : map[string]string { "planner" : "SST" },
		
		"SST*" : map[string]string { "planner" : "SSTStar",
										"n0" : "100000",
										"xi" : "0.99",
									},
		// "SST*2" : map[string]string { "planner" : "SSTStar", "n0" : "100000", "xi" : "0.95" },
		// "SST*3" : map[string]string { "planner" : "SSTStar", "n0" : "100000", "xi" : "0.9" },

		"Restarting RRT with Pruning" : map[string]string { "planner" : "RestartingRRTWithPruning" },

		"BEAST" : map[string]string { "planner" : "BEAST", "searchtype" : "single" },

		// "A-BEAST Switched" : map[string]string { "planner" : "AnytimeBEAST_1" },


		"A-BEAST (SST)" : map[string]string { "planner" : "AnytimeBEAST_1",
										"sstpruning" : "SST",
										"costpruning" : "G",
										"sampler" : "BEAST",
									},

		"A-BEAST (SST*)" : map[string]string { "planner" : "AnytimeBEAST_1",
										"sstpruning" : "SSTStar",
										"costpruning" : "G",
										"sampler" : "BEAST",
									},
		// "A-BEAST (SST*2)" : map[string]string { "planner" : "AnytimeBEAST_0",
		// 								"sstpruning" : "SSTStar",
		// 								"n0" : "100000",
		// 								"xi" : "0.95",
		// 								"costpruning" : "G",
		// 								"sampler" : "BEAST",
		// 							},
		// "A-BEAST (SST*3)" : map[string]string { "planner" : "AnytimeBEAST_0",
		// 								"sstpruning" : "SSTStar",
		// 								"n0" : "100000",
		// 								"xi" : "0.9",
		// 								"costpruning" : "G",
		// 								"sampler" : "BEAST",
		// 							},
		// "A-BEAST (SST*4)" : map[string]string { "planner" : "AnytimeBEAST_0",
		// 								"sstpruning" : "SSTStar",
		// 								"n0" : "50000",
		// 								"xi" : "0.99",
		// 								"costpruning" : "G",
		// 								"sampler" : "BEAST",
		// 							},
		// "A-BEAST (SST*5)" : map[string]string { "planner" : "AnytimeBEAST_0",
		// 								"sstpruning" : "SSTStar",
		// 								"n0" : "50000",
		// 								"xi" : "0.95",
		// 								"costpruning" : "G",
		// 								"sampler" : "BEAST",
		// 							},
		// "A-BEAST (SST*6)" : map[string]string { "planner" : "AnytimeBEAST_0",
		// 								"sstpruning" : "SSTStar",
		// 								"n0" : "50000",
		// 								"xi" : "0.9",
		// 								"costpruning" : "G",
		// 								"sampler" : "BEAST",
		// 							},
	}

	domainTypes := map[string][]string {
		"2D" : []string{"KinematicCar", "DynamicCar", "Hovercraft"},
		"3D" : []string{"Quadrotor", "Blimp"},
	}

	maps := map[string][]string {
		"2D" : []string{"forest.dae", "single-wall.dae", "3-ladder.dae", "parking-lot.dae", "intersection.dae"},
		"3D" : []string{"forest.dae", "fifthelement.dae"},
	}

	meshes := map[string][]string {
		"KinematicCar" : []string{"car2_planar_robot.dae", "car2_planar_robot_SCALED.dae"},
		"DynamicCar" : []string{"car2_planar_robot.dae", "car2_planar_robot_SCALED.dae"},
		"Hovercraft" : []string{"car2_planar_robot.dae", "car2_planar_robot_SCALED.dae"},
		"Quadrotor" : []string{"quadrotor.dae"},
		"Blimp" : []string{"blimp.dae"},
	}

	latex, err := os.Create("plots.tex")
	if err != nil {
		panic(err)
	}
	defer latex.Close()

	fmt.Fprintf(latex, "\\documentclass{article}\n\\usepackage[letterpaper, landscape, margin=0.5in]{geometry}\n\\usepackage{graphicx}\n\\usepackage{placeins}\n\\begin{document}\n")

	for domainType, domains := range domainTypes {
		for _, domain := range domains {
			fmt.Fprintf(latex, "\\section{%s}\n", domain)

			for _, mmap := range maps[domainType] {
				for _, mesh := range meshes[domain] {
					for key := range filters {
						filters[key]["timeout"] = "300"
						filters[key]["domain"] = domain
						filters[key]["map"] = mmap
						filters[key]["agent"] = mesh
					}

					dss := rdb.GetDatasetsFromNonRDBFormat(dataRoot, filters, true, nonAnytimeRDBReader)

					// for i, ds := range dss {
					// 	dss[i] = ds.FilterDataset(func(val string) bool {
					// 		return datautils.ParseIntOrFail(val) <= 10
					// 	}, "seed")
					// }

					filename := fmt.Sprintf("%s - %s - %s", domain, strings.Replace(mmap, ".dae", "", -1), strings.Replace(mesh, ".dae", "", -1))

					fmt.Println(filename)

					solvedCounts := map[string]float64{}
					include := true
					for _, ds := range dss {
						if ds.GetSize() == 0 {
							include = false
						}

						dsValues := ds.GetColumnValuesWithKey("solution", "inst", "solution cost")

						count := 0.
						for _, dfValues := range dsValues {
							if len(dfValues[0]) > 0 {
								count++
							}
						}

						fmt.Printf("\t%s: %d / %d\n", ds.GetName(), int(count), ds.GetSize())

						solvedCounts[ds.GetName()] = count / float64(len(dsValues))
					}
					fmt.Println()

					if !include {
						fmt.Println("Skipping!!\n")
						continue
					}

					// p1 := makeBarPlot(solvedCounts, "Solved Instances", ".", filename, "Percent Solved", plottype, plotWidth, plotHeight)
					p1 := makeCostOverTimetPlot(dss, "Cost", ".", filename, "solution", "solution time", "solution cost", "CPU Time", "Solution Cost", plottype, 0, 300, 15, plotWidth, plotHeight)
					p2 := makeAnytimeSolutionQualityPlot(dss, "Anytime Solution Quality", ".", filename, "solution", "solution time", "solution cost", "CPU Time", "Solution Quality", plottype, 0, 300, 15, plotWidth, plotHeight)
					p3 := makeAnytimeSolvedPlot(dss, "Coverage", ".", filename, "solution", "solution time", "solution cost", "CPU Time", "Coverage", plottype, 0, 300, 15, plotWidth, plotHeight)
					p4 := makeGoalAchievementOverTimePlot(dss, "Goal Achievement Time", ".", filename, "solution", "solution time", "solution cost", "CPU Time", "Goal Achievement Time", plottype, 0, 300, 15, plotWidth, plotHeight)
					p5 := makeOracleGoalAchievementTimePlot(dss, "Oracle Goal Achievement Time", ".", filename+"2", "solution", "solution time", "solution cost", "CPU Time", "Goal Achievement Time", plottype, 0, 300, 15, plotWidth, plotHeight)
					p6 := makeRegretPlot(dss, "Time Delta From Last Found Solution", ".", filename, "solution", "solution time", "solution cost", "CPU Time", "Time Since Last Solution", plottype, 0, 300, 1, plotWidth, plotHeight)

					fmt.Fprintf(latex, "\\subsection{%s}\n", mmap)

					fmt.Fprintf(latex, "\\begin{figure}[!htb]\n\\centering\n\\begin{tabular}{c c}\n")

					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} &\n", p3)
					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} \\\\\n", p1)
					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} &\n", p4)
					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} \\\\\n", p5)
					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} &\n", p2)
					fmt.Fprintf(latex, "\\includegraphics[width=3in]{./%s} \\\\\n", p6)
    
	    			fmt.Fprintf(latex, "\\end{tabular}\n") //\\caption{Sample images of the environments used in the experiments}\n
					fmt.Fprintf(latex, "\\end{figure}\n\\FloatBarrier\\clearpage\n")
				}
			}
		}
	}
	fmt.Fprintf(latex, "\\end{document}\n")
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