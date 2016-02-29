#!/bin/bash

PLANNER=(NewPlanner)
PLANNER_C=0

ABSTRACTION_PATH=("abstractiontype=prm numprmedges=10 prmsize=100" #
				  "abstractiontype=prm numprmedges=10 prmsize=1000" #
				  "abstractiontype=prm numprmedges=10 prmsize=10000" #
				  "abstractiontype=prm numprmedges=5 prmsize=100" #
				  "abstractiontype=prm numprmedges=5 prmsize=1000" #
				  "abstractiontype=prm numprmedges=5 prmsize=10000" #
#				  "abstractiontype=grid gridhintsize=0.5 gridconnectivity=fourway" #
#				  "abstractiontype=grid gridhintsize=0.5 gridconnectivity=diagonal"
				)
ABSTRACTION_PARAM=("AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 100" #
				   "AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 1000" #
				   "AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 10000" #
				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 100" #
				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 1000" #
				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 10000" #
#				   "AbstractionType ? GRID\nGridHintSize ? 0.5\nGridConnectivity ? FourWay" #
#				   "AbstractionType ? GRID\nGridHintSize ? 0.5\nGridConnectivity ? Diagonals" #
				  )
ABSTRACTION_C=0

STATERADIUS=("2" "4" "6")
STATERADIUS_C=0

WHICHSEARCH=("D*")
#Dijkstra
WHICHSEARCH_C=0

EDGEDISTRIBUTION_PATH[0]="validalpha=10 validbeta=1 invalidalpha=10 invalidbeta=1"
EDGEDISTRIBUTION_PARAM[0]="ValidEdgeDistributionAlpha ? 10\nValidEdgeDistributionBeta ? 1\nInvalidEdgeDistributionAlpha ? 1\nInvalidEdgeDistributionBeta ? 10"
EDGEDISTRIBUTION_C=0

ALG_TOTAL=$((${#PLANNER[@]} * ${#ABSTRACTION_PATH[@]} * ${#STATERADIUS[@]} * ${#WHICHSEARCH[@]} * ${#EDGEDISTRIBUTION_PATH[@]}))
ALG_TOTAL_C=0

ALG_PATHS=()
ALG_PARAMS=()
for i in "${!ABSTRACTION_PATH[@]}"
do
	for j in "${!STATERADIUS[@]}"
	do 
		ALG_PATHS+=("${ABSTRACTION_PATH[$i]} stateradius=${STATERADIUS[$j]} validalpha=10 validbeta=1 invalidalpha=10 invalidbeta=1 whichsearch=D*")
		ALG_PARAMS+=("${ABSTRACTION_PATH[$i]}\nStateRadius ? ${STATERADIUS[$j]}\nValidEdgeDistributionAlpha ? 10\nValidEdgeDistributionBeta ? 1\nInvalidEdgeDistributionAlpha ? 1\nInvalidEdgeDistributionBeta ? 10\nWhichSearch ? D*")
	done
done

function morePlannerSettings() {
	local  __resultvar=$1
	local hasMore=true
	if [ "$ALG_TOTAL_C" -ge "$ALG_TOTAL" ];
	then
		hasMore=false
	fi
	((ALG_TOTAL_C++))

	# increment PLANNER[@] PLANNER_C
	# increment ABSTRACTION_PATH[@] ABSTRACTION_C
	# increment STATERADIUS[@] STATERADIUS_C
	# increment WHICHSEARCH[@] WHICHSEARCH_C
	# increment EDGEDISTRIBUTION_PATH[@] EDGEDISTRIBUTION_C

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	index=$((ALG_TOTAL_C - 1))
	path=ALG_PATHS[$index]

	# path="$path planner=${PLANNER[$PLANNER_C]}"
	# path="$path ${ABSTRACTION_PATH[$ABSTRACTION_C]}"
	# path="$path stateradius=${STATERADIUS[$STATERADIUS_C]}"
	# path="$path whichsearch=${WHICHSEARCH[$WHICHSEARCH_C]}"
	# path="$path ${EDGEDISTRIBUTION_PATH[$EDGEDISTRIBUTION_PATH_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""

	index=$((ALG_TOTAL_C - 1))
	param=ALG_PARAMS[$index]
	
	# param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	# param="${param}${ABSTRACTION_PARAM[$ABSTRACTION_C]}\n"
	# param="${param}StateRadius ? ${STATERADIUS[$STATERADIUS_C]}\n"
	# param="${param}WhichSearch ? ${WHICHSEARCH[$WHICHSEARCH_C]}\n"
	# param="${param}${EDGEDISTRIBUTION_PARAM[$EDGEDISTRIBUTION_PATH_C]}\n"

	eval $__resultvar="'$param'" 
}