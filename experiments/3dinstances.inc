#!/bin/bash

function getInstanceNumber() {
	local  __index=$1
	local  __instanceNumber=$2
	local  instanceNumber=$(($__index % 5))
	eval $__instanceNumber="'$instanceNumber'"
}

# FOREST MAP

MAP[0]="forest.dae"
STARTGOAL[0]="Start ? 0 0 0\nGoal ? 0 -25 0\n"
BOUNDS[0]="-30 30 -30 30 -5 5"

MAP[1]="forest.dae"
STARTGOAL[1]="Start ? 0.1 0.1 -1\nGoal ? 0 -26 -1\n"
BOUNDS[1]="-30 30 -30 30 -5 5"

MAP[2]="forest.dae"
STARTGOAL[2]="Start ? 3 -5 1\nGoal ? 0 -25 0\n"
BOUNDS[2]="-30 30 -30 30 -5 5"

MAP[3]="forest.dae"
STARTGOAL[3]="Start ? -3 -5 1\nGoal ? 1 -25 1\n"
BOUNDS[3]="-30 30 -30 30 -5 5"

MAP[4]="forest.dae"
STARTGOAL[4]="Start ? -5 -5 -2\nGoal ? 5 -25 -3\n"
BOUNDS[4]="-30 30 -30 30 -5 5"

# FIFTH ELEMENT

MAP[5]="fifthelement.dae"
STARTGOAL[5]="Start ? 15 7 5\nGoal ? -15 7 5\n"
BOUNDS[5]="-30 30 -30 30 0 15"

MAP[6]="fifthelement.dae"
STARTGOAL[6]="Start ? 15 -7 5\nGoal ? -15 -7 5\n"
BOUNDS[6]="-30 30 -30 30 0 15"

MAP[7]="fifthelement.dae"
STARTGOAL[7]="Start ? 15 -7 5\nGoal ? -15 7 5\n"
BOUNDS[7]="-30 30 -30 30 0 15"

MAP[8]="fifthelement.dae"
STARTGOAL[8]="Start ? 15 7 5\nGoal ? -15 -7 5\n"
BOUNDS[8]="-30 30 -30 30 0 15"

MAP[9]="fifthelement.dae"
STARTGOAL[9]="Start ? 7 15 5\nGoal ? -7 -15 5\n"
BOUNDS[9]="-30 30 -30 30 0 15"

# # Hallways

# MAP[10]="hallway.dae"
# STARTGOAL[10]="Start ? -10 13 5\nGoal ? -10 -13 5\n"
# BOUNDS[10]="-15 15 -15 15 0 15"

# MAP[11]="hallway.dae"
# STARTGOAL[11]="Start ? -13 13 5\nGoal ? -10 -11 5\n"
# BOUNDS[11]="-15 15 -15 15 0 15"

# MAP[12]="hallway.dae"
# STARTGOAL[12]="Start ? -12 13 5\nGoal ? -13 -12 5\n"
# BOUNDS[12]="-15 15 -15 15 0 15"

# MAP[13]="hallway.dae"
# STARTGOAL[13]="Start ? -13.5 13.5 5\nGoal ? -13.5 -13.5 5\n"
# BOUNDS[13]="-15 15 -15 15 0 15"

# MAP[14]="hallway.dae"
# STARTGOAL[14]="Start ? -12 12 5\nGoal ? -12 -10 5\n"
# BOUNDS[14]="-15 15 -15 15 0 15"