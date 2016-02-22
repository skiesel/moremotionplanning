#!/bin/bash

function incrementAndHasMore() {
	declare -a __array=("${!1}")
	local __incrementMe=$2
	local __hasMore=$3

	local incrementedValue=$__incrementMe

	((incrementedValue++))

	if [ "$incrementedValue" -ge "${#__array[@]}" ];
	then
		incrementedValue=0
	else
		eval $__hasMore="'true'"
	fi

	eval $__incrementMe="'$incrementedValue'" 
}