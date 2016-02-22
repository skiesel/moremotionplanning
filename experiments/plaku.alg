#!/bin/bash

PLANNER=(PlakuRRT)
PLANNER_C=-1

PRMSIZE=(10000)
PRMSIZE_C=-1

ALPHA=(0.85)
ALPHA_C=-1

B=(5)
B_C=-1

STATERADIUS=(2)
STATERADIUS_C=-1

NUMPRMEDGES=(10)
NUMPRMEDGES_C=-1

function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=false

	incrementAndHasMore PLANNER[@] PLANNER_C hasMore
	incrementAndHasMore PRMSIZE[@] PRMSIZE_C hasMore
	incrementAndHasMore ALPHA[@] ALPHA_C hasMore
	incrementAndHasMore B[@] B_C hasMore
	incrementAndHasMore STATERADIUS[@] STATERADIUS_C hasMore
	incrementAndHasMore NUMPRMEDGES[@] NUMPRMEDGES_C hasMore

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}"
	path="$path prmsize=${PRMSIZE[$PRMSIZE_C]}"
	path="$path alpha=${ALPHA[$OMEGA_C]}"
	path="$path b=${B[$B_C]}"
	path="$path stateradius=${STATERADIUS[$STATERADIUS_C]}"
	path="$path numprmedges=${NUMPRMEDGES[$NUMPRMEDGES_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}PRMSize ? ${PRMSIZE[$PRMSIZE_C]}\n"
	param="${param}Alpha ? ${ALPHA[$ALPHA_C]}\n"
	param="${param}B ? ${B[$B_C]}\n"
	param="${param}StateRadius ? ${STATERADIUS[$STATERADIUS_C]}\n"
	param="${param}NumEdges ? ${NUMPRMEDGES[$NUMPRMEDGES_C]}\n"

	eval $__resultvar="'$param'" 
}