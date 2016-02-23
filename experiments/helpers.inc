#!/bin/bash

function increment() {
	declare -a __array=("${!1}")
	local __incrementMe=$2

	local incrementedValue=$__incrementMe

	((incrementedValue++))

	if [ "$incrementedValue" -ge "${#__array[@]}" ];
	then
		incrementedValue=0
	fi

	eval $__incrementMe="'$incrementedValue'" 
}