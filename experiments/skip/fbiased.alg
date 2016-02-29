#!/bin/bash

PLANNER=(FBiasedRRT)
PLANNER_C=-1

PRMSIZE=(100 10000)
PRMSIZE_C=-1

OMEGA=(4 8 16)
OMEGA_C=-1

STATERADIUS=(2)
STATERADIUS_C=-1

NUMPRMEDGES=(10)
NUMPRMEDGES_C=-1

ALG_TOTAL=$((${#PLANNER[@]} * ${#PRMSIZE[@]} * ${#OMEGA[@]} * ${#STATERADIUS[@]} * ${#NUMPRMEDGES[@]}))
ALG_TOTAL_C=0

function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=true
	if [ "$ALG_TOTAL_C" -ge "$ALG_TOTAL" ];
	then
		hasMore=false
	fi
	((ALG_TOTAL_C++))

	increment PLANNER[@] PLANNER_C
	increment PRMSIZE[@] PRMSIZE_C
	increment OMEGA[@] OMEGA_C
	increment STATERADIUS[@] STATERADIUS_C
	increment NUMPRMEDGES[@] NUMPRMEDGES_C

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}"
	path="$path prmsize=${PRMSIZE[$PRMSIZE_C]}"
	path="$path omega=${OMEGA[$OMEGA_C]}"
	path="$path stateradius=${STATERADIUS[$STATERADIUS_C]}"
	path="$path numprmedges=${NUMPRMEDGES[$NUMPRMEDGES_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}PRMSize ? ${PRMSIZE[$PRMSIZE_C]}\n"
	param="${param}Omega ? ${OMEGA[$OMEGA_C]}\n"
	param="${param}StateRadius ? ${STATERADIUS[$STATERADIUS_C]}\n"
	param="${param}NumEdges ? ${NUMPRMEDGES[$NUMPRMEDGES_C]}\n"

	eval $__resultvar="'$param'" 
}