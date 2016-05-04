#!/bin/bash

PLANNER=(AnytimeBEAST)
PLANNER_C=-1

BEASTVERSION="_4"

ABSTRACTION_PATH=(
#				  "abstractiontype=prm numprmedges=10 prmsize=100" #
#				  "abstractiontype=prm numprmedges=10 prmsize=1000" #
#				  "abstractiontype=prm numprmedges=10 prmsize=10000" #
#				  "abstractiontype=prm numprmedges=5 prmsize=100" #
				  "abstractiontype=prm numprmedges=5 prmsize=1000" #
#				  "abstractiontype=prm numprmedges=5 prmsize=10000" #
#				  "abstractiontype=grid gridhintsize=0.5 gridconnectivity=fourway" #
#				  "abstractiontype=grid gridhintsize=0.5 gridconnectivity=diagonal"
				)
ABSTRACTION_PARAM=(
#				   "AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 100" #
#				   "AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 1000" #
#				   "AbstractionType ? PRM\nNumEdges ? 10\nPRMSize ? 10000" #
#				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 100" #
				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 1000" #
#				   "AbstractionType ? PRM\nNumEdges ? 5\nPRMSize ? 10000" #
#				   "AbstractionType ? GRID\nGridHintSize ? 0.5\nGridConnectivity ? FourWay" #
#				   "AbstractionType ? GRID\nGridHintSize ? 0.5\nGridConnectivity ? Diagonals" #
				  )
ABSTRACTION_C=0

#2 4 
STATERADIUS=(6)
STATERADIUS_C=0

WHICHSEARCH=("D*")
#Dijkstra
WHICHSEARCH_C=0

EDGEDISTRIBUTION_PATH[0]="validalpha=10 validbeta=1 invalidalpha=10 invalidbeta=1"
EDGEDISTRIBUTION_PARAM[0]="ValidEdgeDistributionAlpha ? 10\nValidEdgeDistributionBeta ? 1\nInvalidEdgeDistributionAlpha ? 1\nInvalidEdgeDistributionBeta ? 10"
EDGEDISTRIBUTION_C=0

#0.005
SELECTIONRADIUS=(2.0)
SELECTIONRADIUS_C=0
#0.0025
PRUNINGRADIUS=(1.0)
PRUNINGRADIUS_C=0

N0=(100)
N0_C=0

XI=(0.5)
XI_C=0

SSTPRUNING=(SST SSTStar None)
SSTPRUNING_C=0

COSTPRUNING=(G F None)
COSTPRUNING_C=0

SAMPLER=(BEAST Uniform)
SAMPLER_C=0

ALG_TOTAL=$((${#PLANNER[@]} * ${#ABSTRACTION_PATH[@]} * ${#STATERADIUS[@]} * ${#WHICHSEARCH[@]} * ${#EDGEDISTRIBUTION_PATH[@]} * ${#SELECTIONRADIUS[@]} * ${#PRUNINGRADIUS[@] * ${#N0[@]} * ${#XI[@]} * ${#SSTPRUNING[@]} * ${#COSTPRUNING[@]} * ${#SAMPLER[@]}))
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
		old=$ABSTRACTION_C
		increment ABSTRACTION_PATH[@] ABSTRACTION_C
		if [ "$old" -gt "$ABSTRACTION_C" ] || [ "$old" -eq "$ABSTRACTION_C" ]
		then
			old=$STATERADIUS_C
			increment STATERADIUS[@] STATERADIUS_C
			if [ "$old" -gt "$STATERADIUS_C" ] || [ "$old" -eq "$STATERADIUS_C" ]
			then
				old=$WHICHSEARCH_C
				increment WHICHSEARCH[@] WHICHSEARCH_C
				if [ "$old" -gt "$STATERADIUS_C" ] || [ "$old" -eq "$STATERADIUS_C" ]
				then
					old=$EDGEDISTRIBUTION_C
					increment EDGEDISTRIBUTION_PATH[@] EDGEDISTRIBUTION_C
					if [ "$old" -gt "$EDGEDISTRIBUTION_C" ] || [ "$old" -eq "$EDGEDISTRIBUTION_C" ]
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
									old=$XI_C
									increment XI[@] XI_C
									if [ "$old" -gt "$XI_C" ] || [ "$old" -eq "$XI_C" ]
									then
										old=$SSTPRUNING_C
										increment SSTPRUNING[@] SSTPRUNING_C
										if [ "$old" -gt "$SSTPRUNING_C" ] || [ "$old" -eq "$SSTPRUNING_C" ]
										then
											old=$COSTPRUNING_C
											increment COSTPRUNING[@] COSTPRUNING_C
											if [ "$old" -gt "$COSTPRUNING_C" ] || [ "$old" -eq "$COSTPRUNING_C" ]
											then
												increment SAMPLER[@] SAMPLER_C
											fi
										fi
									fi
								fi
							fi
						fi
					fi
				fi
			fi
		fi
	fi

	eval $__resultvar="'$hasMore'"
}

function getPlannerPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path planner=${PLANNER[$PLANNER_C]}$BEASTVERSION"
	path="$path ${ABSTRACTION_PATH[$ABSTRACTION_C]}"
	path="$path stateradius=${STATERADIUS[$STATERADIUS_C]}"
	path="$path whichsearch=${WHICHSEARCH[$WHICHSEARCH_C]}"
	path="$path ${EDGEDISTRIBUTION_PATH[$EDGEDISTRIBUTION_PATH_C]}"
	path="$path selectionradius=${SELECTIONRADIUS[$SELECTIONRADIUS_C]}"
	path="$path pruningradius=${PRUNINGRADIUS[$PRUNINGRADIUS_C]}"
	path="$path n0=${N0[$N0_C]}"
	path="$path xi=${XI[$XI_C]}"
	path="$path sstpruning=${SSTPRUNING[$SSTPRUNING_C]}"
	path="$path costpruning=${COSTPRUNING[$COSTPRUNING_C]}"
	path="$path sampler=${SAMPLER[$SAMPLER_C]}"

	eval $__resultvar="'$path'" 
}

function getPlannerParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Planner ? ${PLANNER[$PLANNER_C]}\n"
	param="${param}${ABSTRACTION_PARAM[$ABSTRACTION_C]}\n"
	param="${param}StateRadius ? ${STATERADIUS[$STATERADIUS_C]}\n"
	param="${param}WhichSearch ? ${WHICHSEARCH[$WHICHSEARCH_C]}\n"
	param="${param}${EDGEDISTRIBUTION_PARAM[$EDGEDISTRIBUTION_PATH_C]}\n"
	param="${param}SelectionRadius ? ${SELECTIONRADIUS[$SELECTIONRADIUS_C]}\n"
	param="${param}PruningRadius ? ${PRUNINGRADIUS[$PRUNINGRADIUS_C]}\n"
	param="${param}N0 ? ${N0[$N0_C]}\n"
	param="${param}Xi ? ${XI[$XI_C]}\n"
	param="${param}SSTStyle ? ${SSTPRUNING[$SSTPRUNING_C]}\n"
	param="${param}CostPruningStyle ? ${COSTPRUNING[$COSTPRUNING_C]}\n"
	param="${param}Sampler ? ${SAMPLER[$SAMPLER_C]}\n"

	eval $__resultvar="'$param'" 
}