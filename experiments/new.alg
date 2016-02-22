#!/bin/bash

PLANNER=(NewPlanner)
PLANNER_C=-1

PRMSIZE=(100 10000)
PRMSIZE_C=-1

STATERADIUS=(2)
STATERADIUS_C=-1

NUMPRMEDGES=(10)
NUMPRMEDGES_C=-1

WHICHSEARCH=("D*" Dijkstra)
WHICHSEARCH_C=-1

EDGEDISTRIBUTION_PATH[0]="validalpha=6 validbeta=1 invalidalpha=6 invalidbeta=1"
EDGEDISTRIBUTION_PARAM[0]="ValidEdgeDistributionAlpha ? 6\nValidEdgeDistributionBeta 1\nInvalidEdgeDistributionAlpha ? 1\nInvalidEdgeDistributionBeta 6"
EDGEDISTRIBUTION_C=-1


function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=false

	incrementAndHasMore PLANNER[@] PLANNER_C hasMore
	incrementAndHasMore PRMSIZE[@] PRMSIZE_C hasMore
	incrementAndHasMore STATERADIUS[@] STATERADIUS_C hasMore
	incrementAndHasMore WHICHSEARCH[@] WHICHSEARCH_C hasMore
	incrementAndHasMore NUMPRMEDGES[@] NUMPRMEDGES_C hasMore
	incrementAndHasMore EDGEDISTRIBUTION_PATH[@] EDGEDISTRIBUTION_C hasMore

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}"
	path="$path prmsize=${PRMSIZE[$PRMSIZE_C]}"
	path="$path stateradius=${STATERADIUS[$STATERADIUS_C]}"
	path="$path whichsearch=${WHICHSEARCH[$WHICHSEARCH_C]}"
	path="$path numprmedges=${NUMPRMEDGES[$NUMPRMEDGES_C]}"
	path="$path ${EDGEDISTRIBUTION_PATH[$EDGEDISTRIBUTION_PATH_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}PRMSize ? ${PRMSIZE[$PRMSIZE_C]}\n"
	param="${param}StateRadius ? ${STATERADIUS[$STATERADIUS_C]}\n"
	param="${param}WhichSearch ? ${WHICHSEARCH[$WHICHSEARCH_C]}\n"
	param="${param}NumEdges ? ${NUMPRMEDGES[$NUMPRMEDGES_C]}\n"
	param="${param}${EDGEDISTRIBUTION_PARAM[$EDGEDISTRIBUTION_PATH_C]}\n"

	eval $__resultvar="'$param'" 
}