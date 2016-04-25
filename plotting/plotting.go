package main

import (
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
	"strconv"

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
}

func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment []*rdb.Dataset, log10, tryStacked bool) {
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

func makeAnytimePlot(dss []*rdb.Dataset, title, directory, tableName, xValues, yValues, xLabel, yLabel, format string, startTime, endTime, timeIncrement, width, height float64) {

	bestSolutions := map[string]map[string]float64{}

	anytimeData := map[string][][][]string{}

	algorithms := []string{}

	for _, ds := range dss {
		dsName := ds.GetName()

		anytimeData[dsName] = ds.GetColumnValuesWithKeys(tableName, []string{"inst", "seed"}, xValues, yValues)
		algorithms = append(algorithms, dsName)

		for _, dfValues := range anytimeData[dsName] {

			if len(dfValues[0]) == 0 {
				continue
			}

			inst := dfValues[2][0]
			seed := dfValues[3][0]

			_, err := strconv.ParseFloat(dfValues[1][len(dfValues[1])-1], 64)
			if err != nil {
				fmt.Println(dsName)
				fmt.Println(inst)
				fmt.Println(seed)
				errstr := fmt.Sprintf("could not parse %s\n", dfValues[1][len(dfValues[1])-1])
				panic(errstr)
			}


			solution := datautils.ParseFloatOrFail(dfValues[1][len(dfValues[1])-1])
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

				for ; curPoint < (len(dfValues[0])-1) && datautils.ParseFloatOrFail(dfValues[0][curPoint+1]) <= val; curPoint++ {
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
					sampledPoints[i] = best / datautils.ParseFloatOrFail(dfValues[1][curPoint])
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

		plottingPointArgs = append(plottingPointArgs, algorithmName, points)
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

	filename := strings.Replace(title, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	fmt.Println(filename)

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}
}

func makeBarPlot(data map[string]float64, title, directory, yLabel, format string, width, height float64) {
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
		p.Legend.Add(algorithmName, bar)

		algIndex++
	}

	p.Title.Text = title
	p.Y.Label.Text = yLabel
	p.Y.Min = 0.0
	p.Y.Max = 1.0
    p.Legend.Top = true	
    p.NominalX("")

    filename := strings.Replace(title, " ", "", -1) + "_bars" + format

	if err := p.Save(vg.Length(width)*vg.Inch, vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}
}

