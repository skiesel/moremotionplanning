#!/bin/bash

PLANNER=(SST*)
PLANNER_C=-1

SELECTIONRADIUS=(2.0)
SELECTIONRADIUS_C=0

PRUNINGRADIUS=(1.0)
PRUNINGRADIUS_C=0

N0=(100)
N0_C=0

XI=(0.99)
XI_C=0

ALG_TOTAL=$((${#PLANNER[@] * ${#SELECTIONRADIUS[@]} * ${#PRUNINGRADIUS[@] * ${#N0[@]} * ${#XI[@]}))
ALG_TOTAL_C=0

function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=true
	if [ "$ALG_TOTAL_C" -ge "$ALG_TOTAL" ];
	then
		hasMore=false
	fi
	((ALG_TOTAL_C++))

	old=$PLANNER_C
	increment PLANNER[@] PLANNER_C
	if [ "$old" -gt "$PLANNER_C" ] || [ "$old" -eq "$PLANNER_C" ]
	then
		old=$SELECTIONRADIUS_C
		increment SELECTIONRADIUS[@] SELECTIONRADIUS_C
		if [ "$old" -gt "$SELECTIONRADIUS_C" ] || [ "$old" -eq "$SELECTIONRADIUS_C" ]
		then
			old=$PRUNINGRADIUS_C
			increment PRUNINGRADIUS[@] PRUNINGRADIUS_C
			if [ "$old" -gt "$PRUNINGRADIUS_C" ] || [ "$old" -eq "$PRUNINGRADIUS_C" ]
			then
				old=$N0_C
				increment N0[@] N0_C
				if [ "$old" -gt "$N0_C" ] || [ "$old" -eq "$N0_C" ]
				then
					increment XI[@] XI_C
				fi
			fi
		fi
	fi

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=SSTStar"
	path="$path selectionradius=${SELECTIONRADIUS[$SELECTIONRADIUS_C]}"
	path="$path pruningradius=${PRUNINGRADIUS[$PRUNINGRADIUS_C]}"
	path="$path n0=${N0[$N0_C]}"
	path="$path xi=${XI[$XI_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}SelectionRadius ? ${SELECTIONRADIUS[$SELECTIONRADIUS_C]}\n"
	param="${param}PruningRadius ? ${PRUNINGRADIUS[$PRUNINGRADIUS_C]}\n"
	param="${param}N0 ? ${N0[$N0_C]}\n"
	param="${param}Xi ? ${XI[$XI_C]}\n"

	eval $__resultvar="'$param'" 
}