#!/bin/bash

TIMEOUT=30
MEMORY=1000
RUNS=25

pwd=${PWD}

declare -a DOMAINS=("Blimp" "Quadrotor" "KinematicCar" "DynamicCar")
declare -a PLANNERS=("RRT" "KPIECE" "SyclopRRT" "SyclopEST" "FBiasedRRT" "FBiasedShellRRT" "PlakuRRT")


for DOMAIN in "${DOMAINS[@]}"
do
	for PLANNER in "${PLANNERS[@]}"
	do
		for COUNTER in $(seq 1 $RUNS)
		do
			OUTPUT="data/$DOMAIN$PLANNER$COUNTER"
			echo $pwd/build/MoreMotionPlanning -domain $DOMAIN -planner $PLANNER -timeout $TIMEOUT -memory $MEMORY -output $OUTPUT -seed $COUNTER -runs 1
		done
	done
done