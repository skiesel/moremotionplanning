#!/bin/bash

TIMEOUT=30
MEMORY=1000
RUNS=25

pwd=${PWD}

TEST="test\ntest2"

echo printf \"$TEST\"

declare -a DOMAINS=("Blimp" "Quadrotor" "KinematicCar" "DynamicCar")
declare -a PLANNERS=("RRT" "KPIECE" "SyclopRRT" "SyclopEST" "FBiasedRRT" "FBiasedShellRRT" "PlakuRRT")

FBiasedRRTConfig="Omega ? 8\nStateRadius ? 1\n"
FBiasedShellRRTConfig="Omega ? 8\nStateRadius ? 1\nShellPreference ? 0.9\nShellRadius ? 1\n"
PlakuRRTConfig="Alpha ? 0.85\nB ? 0.85\nStateRadius ? 1\n"

for DOMAIN in "${DOMAINS[@]}"
do

	for PLANNER in "${PLANNERS[@]}"
	do

		for COUNTER in $(seq 1 $RUNS)
		do
			OUTPUT="data/$DOMAIN$PLANNER$COUNTER"

			PARAMFILE="Timeout ? $TIMEOUT\n"
			PARAMFILE=$PARAMFILE"Memory ? $MEMORY\n"
			PARAMFILE=$PARAMFILE"Seed ? $COUNTER\n"
			PARAMFILE=$PARAMFILE"Runs ? 1\n"
			PARAMFILE=$PARAMFILE"Output ? $OUTPUT\n"

			PARAMFILE=$PARAMFILE"Domain ? $DOMAIN\n"
			PARAMFILE=$PARAMFILE"Planner ? $PLANNER\n"
			
			if [ "$PLANNER" = "FBiasedRRT" ];
			then
				PARAMFILE=$PARAMFILE$FBiasedRRTConfig
			elif [ "$PLANNER" = "FBiasedShellRRT" ];
			then
				PARAMFILE=$PARAMFILE$FBiasedShellRRTConfig
			elif [ "$PLANNER" = "PlakuRRT" ];
			then
				PARAMFILE=$PARAMFILE$PlakuRRTConfig
			fi

			echo printf \"$PARAMFILE\" "|" $pwd/build/MoreMotionPlanning
			exit
		done
	done
done