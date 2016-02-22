package main

import (
	"math"
	"sort"
	"strings"

	"github.com/skiesel/plot"
	"github.com/skiesel/plot/plotter"
	"github.com/skiesel/plot/vg"

	"github.com/skiesel/expsys/rdb"
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

func makeBoxPlot(title, yLabel, key, format string, width, height float64, experiment []*rdb.Dataset, log10 bool) {
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

		if ds.TestDataset(func(val string) bool { return val == "true" }, "Solved") {
			continue
		}

		for _, val := range ds.GetDatasetFloatValues(key) {
			if log10 == true {
				mean = mean + math.Log10(val)
			} else {
				mean = mean + val
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
			if algorithm.TestDataset(func(val string) bool { return val == "true" }, "Solved") {
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
		if key == "Solving Time" /*&& algorithm.UseStackedBoxes*/ {
			box, err = plotter.NewBoxPlotWithConfidenceIntervalsStacked(w, i, data, data2)
		} else {
			box, err = plotter.NewBoxPlotWithConfidenceIntervals(w, i, data)
		}
		if err != nil {
			panic(err)
		}

		plotters = append(plotters, box)
		labels = append(labels, strings.Replace(algorithm.GetName(), " ", "\n", -1))
		i++
	}

	p.Add(plotters...)
	p.NominalX(labels...)

	filename := strings.Replace(title, " ", "", -1) + strings.Replace(yLabel, " ", "", -1) + format

	if err := p.Save(vg.Length(width)*vg.Points(15*float64(len(plotters))), vg.Length(height)*vg.Inch, filename); err != nil {
		panic(err)
	}
}