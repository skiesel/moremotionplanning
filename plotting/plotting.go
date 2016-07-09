package main

import (
	"fmt"
	"math"
	"os"
	"sort"
	"strings"

	"github.com/skiesel/plot"
	"github.com/skiesel/plot/plotter"
	"github.com/skiesel/plot/plotutil"
	"github.com/skiesel/plot/vg"

	"github.com/skiesel/expsys/rdb"
	"github.com/skiesel/expsys/plots"
)

type ParallelSlices struct {
	Values []float64
	Labels []string
	Extra []float64
}

func (a ParallelSlices) Len() int {
	return len(a.Values)
}

func (a ParallelSlices) Less(i, j int) bool {
	return a.Values[i] < a.Values[j]
}

func (a ParallelSlices) Swap(i, j int) {
	a.Values[i], a.Values[j] = a.Values[j], a.Values[i]
	a.Labels[i], a.Labels[j] = a.Labels[j], a.Labels[i]
	a.Extra[i], a.Extra[j] = a.Extra[j], a.Extra[i]
}

func makeOracleGoalAchievementTimePlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel

	algorithmGoalAchievementTimes := map[string]map[string]map[string]float64{}
	associatedSolvedTimes := map[string]map[string]map[string]float64{}
	instanceSeeds := map[string]map[string]bool{}

	dsSizes := map[string]float64{}

	yMax := 0.

	slices := ParallelSlices{
		Values: []float64{},
		Labels: []string{},
		Extra: []float64{},
	}

	for _, ds := range dss {
		algorithmName := ds.GetName()

		dsSizes[algorithmName] = float64(ds.GetSize())

		dsValues := ds.GetColumnValuesWithKeys(tableName, []string{"inst", "seed", "Precomputation Time"}, xValues, yValues)

		algorithmGoalAchievementTimes[algorithmName] = map[string]map[string]float64{}
		associatedSolvedTimes[algorithmName] = map[string]map[string]float64{}

		for _, dfValues := range dsValues {

			if len(dfValues[0]) == 0 {
				continue
			}

			inst := dfValues[2][0]
			seed := dfValues[3][0]

			if _, ok := instanceSeeds[inst]; !ok {
				instanceSeeds[inst] = map[string]bool{}
			}
			instanceSeeds[inst][seed] = true

			minGoalAchievementTime := math.Inf(1)
			associatedSolveTime := math.Inf(1)

			// fmt.Println("----------------------")
			for i := range dfValues[0] {

				// fmt.Println(dfValues[2][i] + " " + dfValues[3][i] + " " + dfValues[0][i] + " " + dfValues[1][i] + " " + dfValues[4][i])

				precomputationTime := datautils.ParseFloatOrFail(dfValues[4][i])
				solutionTime := datautils.ParseFloatOrFail(dfValues[0][i]) + precomputationTime
				solutionCost  := datautils.ParseFloatOrFail(dfValues[1][i])

				goalAchievementTime := solutionCost + solutionTime

				if minGoalAchievementTime > goalAchievementTime {
					minGoalAchievementTime = goalAchievementTime
					associatedSolveTime = solutionTime
				}
			}
			// fmt.Println("----------------------")
			
			if _, ok := algorithmGoalAchievementTimes[algorithmName][inst]; !ok {
				algorithmGoalAchievementTimes[algorithmName][inst] = map[string]float64{}
				associatedSolvedTimes[algorithmName][inst] = map[string]float64{}
			}
			if minGoalAchievementTime > yMax {
				yMax = minGoalAchievementTime
			}

			algorithmGoalAchievementTimes[algorithmName][inst][seed] = minGoalAchievementTime
			associatedSolvedTimes[algorithmName][inst][seed] = associatedSolveTime
		}

		slices.Labels = append(slices.Labels, algorithmName)
	}

	for _, algorithmName := range slices.Labels {
		mean := 0.
		times := 0.
		for inst := range instanceSeeds {
			for seed := range instanceSeeds[inst] {
				if val, ok := algorithmGoalAchievementTimes[algorithmName][inst][seed]; ok {
					mean += val
					times += associatedSolvedTimes[algorithmName][inst][seed]
					// fmt.Println(val)
				} else {
					mean += math.Inf(1)
					times += math.Inf(1)
				}
			}
		}

		// fmt.Println("---------------------------")
		// fmt.Println(mean)
		// fmt.Println(dsSizes[algorithmName])
		// fmt.Println(mean / dsSizes[algorithmName])

		slices.Values = append(slices.Values, mean / dsSizes[algorithmName])
		slices.Extra = append(slices.Extra, times / dsSizes[algorithmName])
	}

	sort.Sort(slices)

	w := vg.Points(20)

	i := 0.
	plotters := []plot.Plotter{}
	labels := []string{}
	for slicesIndex, algorithmName := range slices.Labels {
		data := plotter.Values{}

		for inst := range instanceSeeds {
			for seed := range instanceSeeds[inst] {
				if val, ok := algorithmGoalAchievementTimes[algorithmName][inst][seed]; ok {
					data = append(data, val)
				} else {
					data = append(data, math.MaxFloat64)
				}
			}
		}

		solveTimeSuffix := "\n(-)"
		if slices.Extra[slicesIndex] < endTime {
			solveTimeSuffix = fmt.Sprintf("\n(%.2f)", slices.Extra[slicesIndex])
		} else {
			for i := range data {
				data[i] = math.MaxFloat64
			}
		}
		
		box, err := plotter.NewBoxPlotWithConfidenceIntervals(w, i, data)

		if err != nil {
			fmt.Println(algorithmName)
			panic(err)
		}

		plotters = append(plotters, box)
		

		labels = append(labels, strings.Replace(algorithmName, " ", "\n", -1) + solveTimeSuffix)
		i++
	}

	p.Add(plotters...)
	p.NominalX(labels...)
	p.Y.Max = yMax

	filename = strings.Replace(filename, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	fmt.Println(filename)

	size := vg.Length(width)*vg.Points(15*float64(len(plotters)))

	p.Title.TextStyle.Font, err = vg.MakeFont(plot.DefaultFont, 20)
	p.X.Tick.Label.Font, err = vg.MakeFont(plot.DefaultFont, 15)
	p.Y.Label.TextStyle.Font, err = vg.MakeFont(plot.DefaultFont, 20)
	p.Y.Tick.Label.Font, err = vg.MakeFont(plot.DefaultFont, 16)



	if err := p.Save(size, size, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}

func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment []*rdb.Dataset, log10, tryStacked bool) string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel

	slices := ParallelSlices{
		Values: []float64{},
		Labels: []string{},
	}

	for _, ds := range experiment {
		algorithmName := ds.GetName()
		mean := 0.

		if !ds.TestDataset(func(val string) bool { return val == "true" }, "Solved") {
			continue
		}

		precomputationTime := ds.GetDatasetFloatValues("Precomputation Time")
		for i, val := range ds.GetDatasetFloatValues(key) {
			if log10 == true {
				if tryStacked {
					mean = mean + math.Log10(val)
				} else {
					mean = mean + math.Log10(val + precomputationTime[i])
				}
			} else {
				mean = mean + val
				if !tryStacked {
					mean = mean + precomputationTime[i]
				}
			}
		}

		slices.Labels = append(slices.Labels, algorithmName)
		slices.Values = append(slices.Values, mean / float64(ds.GetSize()))
	}

	sort.Sort(slices)

	w := vg.Points(20)

	i := 0.
	plotters := []plot.Plotter{}
	labels := []string{}
	for _, algorithmName := range slices.Labels {
		var algorithm *rdb.Dataset
		for _, ds := range experiment {
			if ds.GetName() == algorithmName {
				algorithm = ds
				break
			}
		}

		if algorithm == nil {
			panic("failed to find algorithm in dataset")
		}

		data := plotter.Values{}
		data2 := plotter.Values{}

		precomputationTime := algorithm.GetDatasetFloatValues("Precomputation Time")
		for i, val := range algorithm.GetDatasetFloatValues(key) {
			if !algorithm.TestDataset(func(val string) bool { return val == "true" }, "Solved") {
				fmt.Println(algorithm.GetName())
				continue
			}

			if log10 == true {
				if key == "Solving Time" {
					data = append(data, math.Log10(val + precomputationTime[i]))
					data2 = append(data2, math.Log10(val))
				} else {
					data = append(data, math.Log10(val))
				}
			} else {
				if key == "Solving Time" {
					data = append(data, val + precomputationTime[i])
					data2 = append(data2, val)
				} else {
					data = append(data, val)
				}
			}
		}

		var box plot.Plotter
		if key == "Solving Time" && tryStacked {
			box, err = plotter.NewBoxPlotWithConfidenceIntervalsStacked(w, i, data, data2)
		} else {
			box, err = plotter.NewBoxPlotWithConfidenceIntervals(w, i, data)
		}
		if err != nil {
			fmt.Println(len(data))
			fmt.Println(len(data2))
			fmt.Println(algorithm.GetName())
			panic(err)
		}

		plotters = append(plotters, box)
		labels = append(labels, strings.Replace(algorithm.GetName(), " ", "\n", -1))
		i++
	}

	p.Add(plotters...)
	p.NominalX(labels...)
	p.Y.Max = 2

	filename := strings.Replace(title, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + "_cropped" + format


	fmt.Println(filename)

	if err := p.Save(vg.Length(width)*vg.Points(15*float64(len(plotters))), vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}

func savePlot(title, directory string, plot *plot.Plot) {
	_, err := os.Stat(directory)
	if os.IsNotExist(err) {
		os.MkdirAll(directory, 0755)
	}

	plotFilename := strings.Replace(directory+"/"+title+".pdf", " ", "", -1)

	err = plot.Save(vg.Length(5)*vg.Inch, vg.Length(5)*vg.Inch, plotFilename)
	if err != nil {
		panic(err)
	}
}

func makeAnytimeSolvedPlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	return makeAnytimePlot(dss, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format, startTime, endTime, timeIncrement, width, height, true, false)
}

func makeAnytimeSolutionQualityPlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	return makeAnytimePlot(dss, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format, startTime, endTime, timeIncrement, width, height, false, false)
}

func makeAnytimePlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64, solvedPlot, goalAchievementPlot bool) string {
	bestSolutions := map[string]map[string]float64{}

	anytimeData := map[string][][][]string{}

	algorithms := []string{}

	for _, ds := range dss {
		dsName := ds.GetName()

		anytimeData[dsName] = ds.GetColumnValuesWithKeys(tableName, []string{"inst", "seed", "Precomputation Time"}, xValues, yValues)
		algorithms = append(algorithms, dsName)

		for _, dfValues := range anytimeData[dsName] {

			if len(dfValues[0]) == 0 {
				continue
			}

			inst := dfValues[2][0]
			seed := dfValues[3][0]
			solution := datautils.ParseFloatOrFail(dfValues[1][len(dfValues[1])-1])
			precomputationTime := datautils.ParseFloatOrFail(dfValues[4][0])

			if solvedPlot {
				solution = 1.
			} else if goalAchievementPlot {
				solution = math.Inf(1)
				for i := range dfValues[1] {
					gat := precomputationTime + datautils.ParseFloatOrFail(dfValues[0][i]) + datautils.ParseFloatOrFail(dfValues[1][i])
					if gat < solution {
						solution = gat
					}
				}
			}

			_, ok := bestSolutions[inst]
			if !ok {
				bestSolutions[inst] = map[string]float64{}
			}
			seedVal, ok := bestSolutions[inst][seed]
			if !ok || solution < seedVal {
				bestSolutions[inst][seed] = solution
			}
		}
	}

	sort.Strings(algorithms)

	var plottingPointArgs []interface{}
	var plottingErrorArgs []interface{}

	meanValues := map[string]float64{}

	for i, algorithmName := range algorithms {

		//Build a function that maps x -> y's across the entire dataset
		generator := func(val float64) []float64 {
			dsValues := anytimeData[algorithmName]
			sampledPoints := make([]float64, len(dsValues))
			for i := range sampledPoints {
				sampledPoints[i] = 0
			}

			for i, dfValues := range dsValues {

				if len(dfValues[0]) == 0 {
					continue
				}

				curPoint := 0

				precomputationTime := datautils.ParseFloatOrFail(dfValues[4][0])

				for ; curPoint < (len(dfValues[0])-1) && datautils.ParseFloatOrFail(dfValues[0][curPoint+1]) + precomputationTime <= val; curPoint++ {
				}
				if curPoint >= len(dfValues[0]) {
					curPoint = len(dfValues[0]) - 1
				}

				inst := dfValues[2][0]
				seed := dfValues[3][0]

				best := bestSolutions[inst][seed]

				if curPoint == 0 && datautils.ParseFloatOrFail(dfValues[0][curPoint]) > val {
					sampledPoints[i] = best / math.Inf(1)
				} else {
					if solvedPlot {
						sampledPoints[i] = best
					} else if goalAchievementPlot {
						sampledPoints[i] = best / (datautils.ParseFloatOrFail(dfValues[1][curPoint]) + val)
					} else {
						sampledPoints[i] = best / datautils.ParseFloatOrFail(dfValues[1][curPoint])
					}
				}
			}

			return sampledPoints
		}

		points, errorBars, err := plotutil.NewErrorPointsSpaced(plotutil.MeanAndConf95,
			int64(i), int64(len(dss)),
			1000, 5,
			startTime, endTime,
			generator,
			startTime, endTime)
		if err != nil {
			panic(err)
		}

		var ds *rdb.Dataset
		for _, ds = range dss {
			if ds.GetName() == algorithmName {
				break
			}
		}

		if ds.TestDataset(func(solved string) bool { return solved == "true"; }, "Solved") {
			algorithmName += "{+}"
			plottingPointArgs = append(plottingPointArgs, algorithmName, points)	
		} else {
			plottingPointArgs = append(plottingPointArgs, algorithmName, points)
		}

		for _,point := range *points {
			meanValues[algorithmName] += point.Y
		}
		meanValues[algorithmName] /= float64(len(*points))

		plottingErrorArgs = append(plottingErrorArgs, errorBars)
	}

	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	plotutil.AddLines(p, plottingPointArgs...)
	plotutil.AddErrorBars(p, plottingErrorArgs...)

	p.Title.Text = title
	p.X.Label.Text = xLabel
	p.Y.Label.Text = yLabel
	p.X.Min = startTime
	p.X.Max = endTime
	p.Y.Min = 0.0
	p.Y.Max = 1.0

	p.Legend.SortLegend(func(a, b string) bool {
		return meanValues[a] > meanValues[b]
	})

	filename = strings.Replace(filename, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	fmt.Println(filename)

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}

func makeBarPlot(data map[string]float64, title, directory, filename, yLabel, format string, width, height float64) string {
	p, err := plot.New()
	if err != nil {
	    panic(err)
	}

	w := vg.Length(width / (float64(len(data)) + 2.))*vg.Inch
	
	algorithms := []string{}
	for algorithmName, _ := range data {
		algorithms = append(algorithms, algorithmName)
	}

	sort.Strings(algorithms)

	algIndex := 0
	for _, algorithmName := range algorithms {
		solvedCount := data[algorithmName]
		
		bar, err := plotter.NewBarChart(plotter.Values{solvedCount}, w)
		if err != nil {
			panic(err)
		}		
		bar.LineStyle.Width = vg.Length(0)
		bar.Color = plotutil.Color(algIndex)
		bar.Offset = w * vg.Length(len(data) / 2 - algIndex)

		p.Add(bar)
		if solvedCount == 1. {
			p.Legend.Add(algorithmName + "{+}", bar)
		} else {
			p.Legend.Add(algorithmName, bar)
		}

		algIndex++
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel
	p.Y.Min = 0.0
	p.Y.Max = 1.0
    p.Legend.Top = true	
    p.NominalX("")

    filename = strings.Replace(filename, " ", "", -1) + "_bars" + format

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}

func makeRegretPlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	var plottingPointArgs []interface{}
	var plottingErrorArgs []interface{}

	meanValues := map[string]float64{}

	algorithms := []string{}
	for _, ds := range dss {
		algorithms = append(algorithms, ds.GetName())
	}

	sort.Strings(algorithms)

	for i, algorithmName := range algorithms {
		var ds *rdb.Dataset
		for _, ds = range dss {
			if ds.GetName() == algorithmName {
				break
			}
		}

		generator := func(val float64) []float64 {
			dsValues := ds.GetColumnValuesWithKeys(tableName, []string{"Precomputation Time"}, xValues, yValues)
			sampledPoints := make([]float64, len(dsValues))
			for i := range sampledPoints {
				sampledPoints[i] = val
			}

			for i, dfValues := range dsValues {

				if len(dfValues[0]) == 0 {
					continue
				}

				curPoint := 0
				precomputationTime := datautils.ParseFloatOrFail(dfValues[2][0])

				for ; curPoint < (len(dfValues[0])-1) && datautils.ParseFloatOrFail(dfValues[0][curPoint+1]) + precomputationTime <= val; curPoint++ {
				}
				if curPoint >= len(dfValues[0]) {
					curPoint = len(dfValues[0]) - 1
				}

				solutionTime := datautils.ParseFloatOrFail(dfValues[0][curPoint]) 
				if solutionTime <= val {
					sampledPoints[i] = val - solutionTime
				}
			}

			return sampledPoints
		}


	    points, errorBars, err := plotutil.NewErrorPointsSpaced(plotutil.MeanAndConf95,
			int64(i), int64(len(dss)),
			1000, 5,
			startTime, endTime,
			generator,
			startTime, endTime)

		// mean95, err := plotutil.NewErrorPoints(plotutil.MeanAndConf95, pts...)
	    if err != nil {
	        panic(err)
	    }

	    algorithmName := ds.GetName()
		if ds.TestDataset(func(solved string) bool { return solved == "true"; }, "Solved") {
			algorithmName += "{+}"
			plottingPointArgs = append(plottingPointArgs, algorithmName, points)	
		} else {
			plottingPointArgs = append(plottingPointArgs, algorithmName, points)
		}

		for _,point := range *points {
			meanValues[algorithmName] += point.Y
		}
		meanValues[algorithmName] /= float64(len(*points))

		plottingErrorArgs = append(plottingErrorArgs, errorBars)
	}

	plotutil.AddLines(p, plottingPointArgs...)
	plotutil.AddErrorBars(p, plottingErrorArgs...)

	p.Title.Text = title
	p.X.Label.Text = xLabel
	p.Y.Label.Text = yLabel
	p.X.Min = startTime
	p.X.Max = endTime
	p.Y.Min = startTime
	p.Y.Max = endTime

	p.Legend.SortLegend(func(a, b string) bool {
		return meanValues[a] > meanValues[b]
	})

	filename = strings.Replace(filename, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	fmt.Println(filename)

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}

func makeCostOverTimetPlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	return makeRawXOverTimePlot(dss, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format, startTime, endTime, timeIncrement, width, height, false)
}

func makeGoalAchievementOverTimePlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) string {
	return makeRawXOverTimePlot(dss, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format, startTime, endTime, timeIncrement, width, height, true)
}

func makeRawXOverTimePlot(dss []*rdb.Dataset, title, directory, filename, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64, goalAchievementTime bool) string {
	p, err := plot.New()
	if err != nil {
		panic(err)
	}

	var plottingPointArgs []interface{}
	var plottingErrorArgs []interface{}

	meanValues := map[string]float64{}

	algorithms := []string{}
	for _, ds := range dss {
		algorithms = append(algorithms, ds.GetName())
	}

	sort.Strings(algorithms)

	realMaxY := 0.

	for i, algorithmName := range algorithms {
		var ds *rdb.Dataset
		for _, ds = range dss {
			if ds.GetName() == algorithmName {
				break
			}
		}

		dsValues := ds.GetColumnValuesWithKeys(tableName, []string{"Precomputation Time"}, xValues, yValues)

		firstSolutionTime := 0.
		solvedAll := true

		for _, dfValues := range dsValues {
			if len(dfValues[0]) == 0 {
				solvedAll = false
				break
			} else {
				precomputationTime := datautils.ParseFloatOrFail(dfValues[2][0])
				solutionTime := datautils.ParseFloatOrFail(dfValues[0][0])
				if firstSolutionTime < precomputationTime + solutionTime {
					firstSolutionTime = precomputationTime + solutionTime
				}
			}
		}

		if !solvedAll {
			firstSolutionTime = endTime + 1
		}

		generator := func(val float64) []float64 {
			dsValues := ds.GetColumnValuesWithKeys(tableName, []string{"Precomputation Time"}, xValues, yValues)
			sampledPoints := make([]float64, len(dsValues))
			
			for i, dfValues := range dsValues {

				if len(dfValues[0]) == 0 {
					sampledPoints[i] = math.Inf(1)
					continue
				}

				curPoint := 0

				precomputationTime := datautils.ParseFloatOrFail(dfValues[2][0])

				for ; curPoint < (len(dfValues[0])-1) && datautils.ParseFloatOrFail(dfValues[0][curPoint+1]) + precomputationTime <= val; curPoint++ {
				}
				if curPoint >= len(dfValues[0]) {
					curPoint = len(dfValues[0]) - 1
				}

				if curPoint == 0 && datautils.ParseFloatOrFail(dfValues[0][curPoint]) > val {
					sampledPoints[i] = math.Inf(1)
				} else {
					if goalAchievementTime {
						sampledPoints[i] = datautils.ParseFloatOrFail(dfValues[1][curPoint]) + precomputationTime + val
					} else {
						sampledPoints[i] = datautils.ParseFloatOrFail(dfValues[1][curPoint])
					}
					
				}
			}

			return sampledPoints
		}

	    points, errorBars, err := plotutil.NewErrorPointsSpaced(plotutil.MeanAndConf95,
			int64(i), int64(len(dss)),
			1000, 5,
			firstSolutionTime, endTime,
			generator,
			firstSolutionTime, endTime)

	    if err != nil {
	        panic(err)
	    }

	    algorithmName := ds.GetName()

	    if len(*points) > 0 {
	    	paddedPoints := make(plotter.XYs, 1)
	    	paddedPoints[0].X = firstSolutionTime
	    	paddedPoints[0].Y = 10000
	    	paddedPoints = append(paddedPoints, *points...)

	    	for _, pt := range *points {
	    		realMaxY = math.Max(realMaxY, pt.Y)
	    	}

	    	for _, yErr := range errorBars.YErrors {
	    		realMaxY = math.Max(realMaxY, yErr.High)
	    	}

			if ds.TestDataset(func(solved string) bool { return solved == "true"; }, "Solved") {
				algorithmName += "{+}"
				plottingPointArgs = append(plottingPointArgs, algorithmName, paddedPoints)	
			} else {
				plottingPointArgs = append(plottingPointArgs, algorithmName, paddedPoints)
			}

	    } else {
	    	if ds.TestDataset(func(solved string) bool { return solved == "true"; }, "Solved") {
	    		algorithmName += "{+}"
				plottingPointArgs = append(plottingPointArgs, algorithmName, points)	
			} else {
				plottingPointArgs = append(plottingPointArgs, algorithmName, points)
			}
	    }

	    min := 100000.
	    for _, val := range *points {
	    	if val.Y < min {
	    		min = val.Y
	    	}
	    }
	    // fmt.Printf("%s %g\n", algorithmName, min)

	    meanValues[algorithmName] += firstSolutionTime * 1000
	    for _,point := range *points {
			meanValues[algorithmName] += point.Y
		}
		meanValues[algorithmName] /= float64(len(*points))

		plottingErrorArgs = append(plottingErrorArgs, errorBars)
	}

	plotutil.AddLines(p, plottingPointArgs...)
	plotutil.AddErrorBars(p, plottingErrorArgs...)

	p.Title.Text = title
	p.X.Label.Text = xLabel
	p.Y.Label.Text = yLabel
	p.X.Min = startTime
	p.X.Max = endTime
	p.Y.Max = realMaxY

	p.Legend.SortLegend(func(a, b string) bool {
		return meanValues[a] > meanValues[b]
	})

	filename = strings.Replace(filename, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	fmt.Println(filename)

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}

	return strings.Replace(filename, format, "", -1)
}