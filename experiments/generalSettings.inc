#!/bin/bash

source helpers.inc

TIMEOUT=(30)
TIMEOUT_C=-1

MEMORY=(1000)
MEMORY_C=-1

declare -a SEED
SEED_C=-1

for i in {0..24}
do
	SEED[$i]=$((i+1))
done

function moreGeneralSettings() {
	local  __resultvar=$1
	local hasMore=false

	incrementAndHasMore TIMEOUT[@] TIMEOUT_C hasMore
	incrementAndHasMore MEMORY[@] MEMORY_C hasMore
	incrementAndHasMore SEED[@] SEED_C hasMore

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