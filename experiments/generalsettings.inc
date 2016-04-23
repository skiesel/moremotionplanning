#!/bin/bash

source helpers.inc

TIMEOUT=(60)
TIMEOUT_C=-1

MEMORY=(1000)
MEMORY_C=0

INTERMEDIATESTATES=(true)
INTERMEDIATESTATES_C=0

declare -a SEED
SEED_C=0

for i in {0..24}
do
	SEED[$i]=$((i+1))
done

GEN_TOTAL=$((${#TIMEOUT[@]} * ${#MEMORY[@]} * ${#SEED[@]}))
GEN_TOTAL_C=0

function moreGeneralSettings() {
	local  __resultvar=$1
	local hasMore=true
	if [ "$GEN_TOTAL_C" -ge "$GEN_TOTAL" ];
	then
		hasMore=false
	fi
	((GEN_TOTAL_C++))

	old=$TIMEOUT_C
	increment TIMEOUT[@] TIMEOUT_C
	if [ "$old" -gt "$TIMEOUT_C" ] || [ "$old" -eq "$TIMEOUT_C" ]
	then
		old=$MEMORY_C
		increment MEMORY[@] MEMORY_C
		if [ "$old" -gt "$MEMORY_C" ] || [ "$old" -eq "$MEMORY_C" ]
		then
			old=$INTERMEDIATESTATES_C
			increment INTERMEDIATESTATES[@] INTERMEDIATESTATES_C
			if [ "$old" -gt "$INTERMEDIATESTATES_C" ] || [ "$old" -eq "$INTERMEDIATESTATES_C" ]
			then
				increment SEED[@] SEED_C
			fi
		fi
	fi

	eval $__resultvar="'$hasMore'"
}

function getGeneralPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path timeout=${TIMEOUT[$TIMEOUT_C]}"
	path="$path memory=${MEMORY[$MEMORY_C]}"
	path="$path intermediatestates=${INTERMEDIATESTATES[$INTERMEDIATESTATES_C]}"
	path="$path seed=${SEED[$SEED_C]}"

	eval $__resultvar="'$path'" 
}

function getGeneralParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Timeout ? ${TIMEOUT[$TIMEOUT_C]}\n"
	param="${param}Memory ? ${MEMORY[$MEMORY_C]}\n"
	param="${param}AddIntermediateStates ? ${INTERMEDIATESTATES[$INTERMEDIATESTATES_C]}\n"
	param="${param}Seed ? ${SEED[$SEED_C]}\n"

	#this is just an ompl artifact
	param="${param}Runs ? 1\n"

	eval $__resultvar="'$param'" 
}