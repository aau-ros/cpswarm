#!/bin/bash


usage()
{
cat << EOF
usage: $0 options

This script executes multiple runs of the same simulation

OPTIONS:
-h          Show this message
-t          Timeout when to abort a simulation run (in seconds)
-n          Number of robots (required)
-r          Number of runs (required)
-m          The map to be used
-e          Emulate multicast communication
-s	    Show simulation screen stage
-o          Enable output to screen
-f          Select the exploration strategy 1..7
EOF
}


. ~/ros_hydro/devel/setup.bash

map=
number_of_robots=
timeout=3600
number_of_runs=
emulate=
screen=
output=
strategy=

while getopts "n:m:r::t::esof::" OPTION; do
    case $OPTION in 
        h) usage
            ;;
        n) number_of_robots=$OPTARG
            ;;
        m) map=$OPTARG
            ;;
        r) number_of_runs=$OPTARG
            ;;
        t) timeout=$OPTARG
            ;;
        e) emulate="--emulate-comm"
            ;;
	s) screen="--gui"
	    ;;
	o) output="--screen-output"
	    ;;
	f) strategy="--exp-strategy $OPTARG"
	    ;;
        ?) usage
	   exit
    esac
done;

if [[ -z $number_of_robots ]] || [[ -z $number_of_runs ]]; then
    echo -e "\e[1;34mInvalid input arguments: Please specify the number of robots and runs.\e[0m"
    usage
    exit
fi
echo -e "\e[1;34m"
echo "Number of runs  	:" $number_of_runs
echo "Number of robots	:" $number_of_robots
echo "Map             	:" $map
echo "Timeout         	:" $timeout
echo "Emulate         	:" $emulate
echo "Screen          	:" $screen
echo "Output          	:" $output
echo "Frontier strategy	:" $strategy
echo -e "\e[0m"



STATUS_PATH=`rospack find multi_robot_analyzer`"/simulation_status"
if [ ! -d "$STATUS_PATH" ]; then
	mkdir "$STATUS_PATH"
fi
echo -e "\e[1;34mChecking for files in directoy $STATUS_PATH\e[0m"
rm $STATUS_PATH/* &> /dev/null
echo

echo -e "\e[1;34mStart simulation [y/n]?\e[0m"
read INPUT

if [ "$INPUT" = "y" ]; then
	echo -e "\e[1;34mStarting..\e[0m"
else
	echo -e "\e[1;34mAborting..\e[0m"
	exit
fi




DATE=`date +"%y-%m-%d"`
TIME=`date +"%k-%M-%S"`

echo -e "\e[1;34mDATE: $DATE\e[0m"
echo -e "\e[1;34mTIME: $TIME\e[0m"

for RUN_NUMBER in $(seq 1 $number_of_runs); do
	echo -e "\e[1;34mSTARTING RUN $RUN_NUMBER\e[0m"
	roscd multi_robot_simulation
	cd scripts
        if [[ -z $map ]]; then
	    ./simulate_exploration.sh -n $number_of_robots $screen $output -i $RUN_NUMBER -d $DATE -t $TIME $emulate $strategy&
        else
	    ./simulate_exploration.sh -n $number_of_robots $screen $output -i $RUN_NUMBER -d $DATE -t $TIME $emulate -m $map $strategy&
        fi

        START_TIME=`date +%s`
	FILES=0
        DURATION=0
	while [ "$FILES" -lt "$number_of_robots" -a "$DURATION" -lt "$timeout" ]; do
            NOW=`date +%s`
            DURATION=$(($NOW-$START_TIME))
            echo -e "\e[1;34mRunning ($RUN_NUMBER/$number_of_runs) for $DURATION seconds (max = $timeout seconds). Found $FILES of $number_of_robots expected files to indicate end.\e[0m"
	    FILES=`ls -l "$STATUS_PATH"|wc -l`
	    FILES=$(($FILES-1))
	    sleep 5	# test every 5s if simulation completed
	done;
        if [ "$timeout" -lt "$DURATION" ]; then
            echo -e "\e[1;34mExceeded maximum simulation duration of $timeout seconds.\e[0m"
        else
            echo -e "\e[1;34mFound all files.\e[0m"
        fi
        echo -e "\e[1;34mWaiting 15s and then killing simulation.\e[0m"
        sleep 15
	echo -e "\e[1;34mKilling jobs...\e[0m"
	killall -9 rosmaster roslaunch rosout adhoc_communication map_merger explorer stageros move_base slam_gmapping
	pkill -9 -f adhoc_communication_emulator
	pkill -9 -f connection_manager
	echo -e "\e[1;34mAll jobs killed. Sleeping for another 15s before continuing.\e[0m"
	rm $STATUS_PATH/* &> /dev/null
	sleep 15 # wait time for everything to "shutdown"
done

roscd multi_robot_simulation
rm launch/now.launch
echo -e "\e[1;34mFinished execution of $number_of_runs runs including $number_of_robots robots.\e[0m"
echo -e "\e[0m"



