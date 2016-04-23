#!/bin/bash

PLANNER=(SST)
PLANNER_C=-1

SELECTIONRADIUS=(2.0)
SELECTIONRADIUS_C=0

PRUNINGRADIUS=(1.0)
PRUNINGRADIUS_C=0

ALG_TOTAL=$((${#PLANNER[@] * ${#SELECTIONRADIUS[@]} * ${#PRUNINGRADIUS[@]}))
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
			increment PRUNINGRADIUS[@] PRUNINGRADIUS_C
		fi
	fi

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}"
	path="$path selectionradius=${SELECTIONRADIUS[$SELECTIONRADIUS_C]}"
	path="$path pruningradius=${PRUNINGRADIUS[$PRUNINGRADIUS_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}SelectionRadius ? ${SELECTIONRADIUS[$SELECTIONRADIUS_C]}\n"
	param="${param}PruningRadius ? ${PRUNINGRADIUS[$PRUNINGRADIUS_C]}\n"

	eval $__resultvar="'$param'" 
}