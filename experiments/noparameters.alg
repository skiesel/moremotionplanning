#!/bin/bash

PLANNER=(RRT EST SST KPIECE SyclopRRT SyclopEST PDST)
PLANNER_C=-1

function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=false

	incrementAndHasMore PLANNER[@] PLANNER_C hasMore

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"

	eval $__resultvar="'$param'" 
}