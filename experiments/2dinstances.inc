#!/bin/bash

function getInstanceNumber() {
	local  __index=$1
	local  __instanceNumber=$2
	local  instanceNumber=$(($__index % 5))
	eval $__instanceNumber="'$instanceNumber'"
}

# Hallways

MAP[0]="hallway.dae"
STARTGOAL[0]="Start ? -10 13\nGoal ? -10 -13\n"
BOUNDS[0]="-15 15 -15 15"

MAP[1]="hallway.dae"
STARTGOAL[1]="Start ? -13 13\nGoal ? -10 -11\n"
BOUNDS[1]="-15 15 -15 15"

MAP[2]="hallway.dae"
STARTGOAL[2]="Start ? -12 13\nGoal ? -13 -12\n"
BOUNDS[2]="-15 15 -15 15"

MAP[3]="hallway.dae"
STARTGOAL[3]="Start ? -13.5 13.5\nGoal ? -13.5 -13.5\n"
BOUNDS[3]="-15 15 -15 15"

MAP[4]="hallway.dae"
STARTGOAL[4]="Start ? -12 12\nGoal ? -12 -10\n"
BOUNDS[4]="-15 15 -15 15"

# # FOREST MAP

# MAP[0]="forest.dae"
# STARTGOAL[0]="Start ? 0 0\nGoal ? 0 -25\n"
# BOUNDS[0]="-30 30 -30 30"

# MAP[1]="forest.dae"
# STARTGOAL[1]="Start ? 0.1 0.1\nGoal ? 0 -26\n"
# BOUNDS[1]="-30 30 -30 30"

# MAP[2]="forest.dae"
# STARTGOAL[2]="Start ? 3 -5\nGoal ? 0 -25\n"
# BOUNDS[2]="-30 30 -30 30"

# MAP[3]="forest.dae"
# STARTGOAL[3]="Start ? -3 -5\nGoal ? 1 -25\n"
# BOUNDS[3]="-30 30 -30 30"

# MAP[4]="forest.dae"
# STARTGOAL[4]="Start ? -5 -5\nGoal ? 5 -25\n"
# BOUNDS[4]="-30 30 -30 30"

# # SINGLE WALL

# MAP[5]="single-wall.dae"
# STARTGOAL[5]="Start ? -13 -12\nGoal ? -13 13\n"
# BOUNDS[5]="-15 15 -15 15"

# MAP[6]="single-wall.dae"
# STARTGOAL[6]="Start ? -13 -13\nGoal ? -12 10\n"
# BOUNDS[6]="-15 15 -15 15"

# MAP[7]="single-wall.dae"
# STARTGOAL[7]="Start ? -13 -11\nGoal ? -10 10\n"
# BOUNDS[7]="-15 15 -15 15"

# MAP[8]="single-wall.dae"
# STARTGOAL[8]="Start ? -10 -12\nGoal ? -13 13\n"
# BOUNDS[8]="-15 15 -15 15"

# MAP[9]="single-wall.dae"
# STARTGOAL[9]="Start ? -5 -12\nGoal ? -13 13\n"
# BOUNDS[9]="-15 15 -15 15"

# # LADDER

# MAP[10]="3-ladder.dae"
# STARTGOAL[10]="Start ? -13 -12\nGoal ? -13 12\n"
# BOUNDS[10]="-15 15 -15 15"

# MAP[11]="3-ladder.dae"
# STARTGOAL[11]="Start ? -13 -13\nGoal ? -13 12\n"
# BOUNDS[11]="-15 15 -15 15"

# MAP[12]="3-ladder.dae"
# STARTGOAL[12]="Start ? -13 -11\nGoal ? -13 12\n"
# BOUNDS[12]="-15 15 -15 15"

# MAP[13]="3-ladder.dae"
# STARTGOAL[13]="Start ? -13 -12\nGoal ? -13 13\n"
# BOUNDS[13]="-15 15 -15 15"

# MAP[14]="3-ladder.dae"
# STARTGOAL[14]="Start ? -13 -12\nGoal ? -13 11\n"
# BOUNDS[14]="-15 15 -15 15"

# # PARKING

# MAP[15]="parking-lot.dae"
# STARTGOAL[15]="Start ? -13 -12\nGoal ? 5 5\n"
# BOUNDS[15]="-15 15 -15 15"

# MAP[16]="parking-lot.dae"
# STARTGOAL[16]="Start ? -13 -12\nGoal ? 8 3\n"
# BOUNDS[16]="-15 15 -15 15"

# MAP[17]="parking-lot.dae"
# STARTGOAL[17]="Start ? -13 -12\nGoal ? -4 0\n"
# BOUNDS[17]="-15 15 -15 15"

# MAP[18]="parking-lot.dae"
# STARTGOAL[18]="Start ? -13 -12\nGoal ? 5 -2\n"
# BOUNDS[18]="-15 15 -15 15"

# MAP[19]="parking-lot.dae"
# STARTGOAL[19]="Start ? -13 -12\nGoal ? 13 12\n"
# BOUNDS[19]="-15 15 -15 15"

# # INTERSECTION

# MAP[20]="intersection.dae"
# STARTGOAL[20]="Start ? -13 13\nGoal ? 13 -2\n"
# BOUNDS[20]="-15 15 -15 15"

# MAP[21]="intersection.dae"
# STARTGOAL[21]="Start ? -13 13\nGoal ? -3 -13\n"
# BOUNDS[21]="-15 15 -15 15"

# MAP[22]="intersection.dae"
# STARTGOAL[22]="Start ? -13 13\nGoal ? -13 0\n"
# BOUNDS[22]="-15 15 -15 15"

# MAP[23]="intersection.dae"
# STARTGOAL[23]="Start ? -13 13\nGoal ? 3 -8\n"
# BOUNDS[23]="-15 15 -15 15"

# MAP[24]="intersection.dae"
# STARTGOAL[24]="Start ? -13 13\nGoal ? 3 2\n"
# BOUNDS[24]="-15 15 -15 15"

# # Hallways

# MAP[25]="hallway.dae"
# STARTGOAL[25]="Start ? -10 13\nGoal ? -10 -13\n"
# BOUNDS[25]="-15 15 -15 15"

# MAP[26]="hallway.dae"
# STARTGOAL[26]="Start ? -13 13\nGoal ? -10 -11\n"
# BOUNDS[26]="-15 15 -15 15"

# MAP[27]="hallway.dae"
# STARTGOAL[27]="Start ? -12 13\nGoal ? -13 -12\n"
# BOUNDS[27]="-15 15 -15 15"

# MAP[28]="hallway.dae"
# STARTGOAL[28]="Start ? -13.5 13.5\nGoal ? -13.5 -13.5\n"
# BOUNDS[28]="-15 15 -15 15"

# MAP[29]="hallway.dae"
# STARTGOAL[29]="Start ? -12 12\nGoal ? -12 -10\n"
# BOUNDS[29]="-15 15 -15 15"