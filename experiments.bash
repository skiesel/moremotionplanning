#!/bin/bash

TIMEOUT=30
MEMORY=1000
RUNS=25

pwd=${PWD}

EXPORT_CMD="export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${pwd}/build; "

declare -a DOMAINS=("Blimp" "Quadrotor" "KinematicCar" "DynamicCar")
declare -a PLANNERS=("FBiasedRRT" "FBiasedShellRRT" "PlakuRRT")
#"RRT" "KPIECE" "SyclopRRT" "SyclopEST" 

FBiasedRRTConfig="Omega ? 8\nStateRadius ? 1\n"
FBiasedShellRRTConfig="Omega ? 8\nStateRadius ? 1\nShellPreference ? 0.9\nShellRadius ? 1\n"
PlakuRRTConfig="Alpha ? 0.85\nB ? 0.85\nStateRadius ? 1\n"

CHEAT=1

for DOMAIN in "${DOMAINS[@]}"
do

	for PLANNER in "${PLANNERS[@]}"
	do

		for COUNTER in $(seq 1 $RUNS)
		do
			OUTPUT="$pwd/data/$DOMAIN$PLANNER$COUNTER"			

			PARAMFILE="Timeout ? $TIMEOUT\n"
			PARAMFILE=$PARAMFILE"Memory ? $MEMORY\n"
			PARAMFILE=$PARAMFILE"Seed ? $COUNTER\n"
			PARAMFILE=$PARAMFILE"Runs ? 1\n"

			if [ "$CHEAT" -eq "1" ];
			then
				PARAMFILE=$PARAMFILE"Cheat ? true\n"
				OUTPUT=$OUTPUT"_cheat"
			fi

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

			echo $EXPORT_CMD printf \"$PARAMFILE\" "|" $pwd/build/MoreMotionPlanning
		done
	done
done
