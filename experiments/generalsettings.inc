#!/bin/bash

source helpers.inc

TIMEOUT=(30 60)
TIMEOUT_C=-1

MEMORY=(1000)
MEMORY_C=-1

declare -a SEED
SEED_C=-1

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

	increment TIMEOUT[@] TIMEOUT_C
	increment MEMORY[@] MEMORY_C
	increment SEED[@] SEED_C

	eval $__resultvar="'$hasMore'"
}

function getGeneralPathAttrs() {
	local  __resultvar=$1
	local path=""
	
	path="$path timeout=${TIMEOUT[$TIMEOUT_C]}"
	path="$path memory=${MEMORY[$MEMORY_C]}"
	path="$path seed=${SEED[$SEED_C]}"

	eval $__resultvar="'$path'" 
}

function getGeneralParamFile() {
	local  __resultvar=$1
	local param=""
	
	param="${param}Timeout ? ${TIMEOUT[$TIMEOUT_C]}\n"
	param="${param}Memory ? ${MEMORY[$MEMORY_C]}\n"
	param="${param}Seed ? ${SEED[$SEED_C]}\n"

	#this is just an ompl artifact
	param="${param}Runs ? 1\n"

	eval $__resultvar="'$param'" 
}