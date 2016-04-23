#!/bin/bash

PROJECT_ROOT="${HOME}/gopath/src/github.com/skiesel/moremotionplanning"

DATA_ROOT="${PROJECT_ROOT}/experiments/data"
EXEC="${PROJECT_ROOT}/build/MoreMotionPlanning"

SEARCH="${HOME}/gopath/src/github.com/skiesel/search"
dfcmplt="$SEARCH/rdb/dfcmplt"
pathattrs="$SEARCH/rdb/pathattrs"
pathfor="$SEARCH/rdb/pathfor"
withattrs="$SEARCH/rdb/withattrs"

EXPORT_CMD="export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PROJECT_ROOT}/build; "
#EXPORT_CMD=""

SEARCHTYPE="anytime"

source generalsettings.inc
moreGeneralSettings moreGen

echo "sendSlackNotification.bash \"#experiments\" \"experiment_bot\" \"Scott just started running experiments.\""

while [ "$moreGen" = true ];
do
	getGeneralPathAttrs generalPathAttrs
	getGeneralParamFile generalParamFile
	moreGeneralSettings moreGen

	generalPathAttrs="searchtype=${SEARCHTYPE} ${generalPathAttrs}"

	for DOMAIN_INC in `ls *.dom`;
	do
		source $DOMAIN_INC
		moreDomainSettings moreDS

		while [ "$moreDS" = true ];
		do
			getDomainPathAttrs domainPathAttrs
			getDomainParamFile domainParamFile
			moreDomainSettings moreDS

			for PLANNER_INC in `ls *.alg`;
			do
				source $PLANNER_INC
				morePlannerSettings morePS

				while [ "$morePS" = true ];
				do
					getPlannerPathAttrs plannerPathAttrs
					getPlannerParamFile plannerParamFile
					morePlannerSettings morePS

					outputFile=`$pathfor $DATA_ROOT $generalPathAttrs $domainPathAttrs $plannerPathAttrs`

					inputFile="printf \"$generalParamFile $domainParamFile $plannerParamFile Output ? ${outputFile}\n\""
					echo "if [ ! -s $outputFile ]; then $EXPORT_CMD $inputFile | $EXEC; fi"
				done
			done
		done
	done
done

echo "sendSlackNotification.bash \"#experiments\" \"experiment_bot\" \"Scott's experiments just finished.\""
