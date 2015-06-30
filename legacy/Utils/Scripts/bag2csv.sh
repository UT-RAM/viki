#!/bin/bash
## This script is intended to convert data logged in a ROS .bag file into a .csv file.
# For each topic present into the bag file (or specified as arguments) a new .csv file is created into the specified folder
# Alex Capra - a_DOT_capra_AT_utwente_DOT_nl
## 

functioname=`echo $0 | rev | cut -d '/' -f 1 | rev`;

explainString="This script is intended to convert data logged in a ROS .bag file into multiple .csv files\n\
If called with two arguments it will convert all the ___non___ image-related topics, otherwise only the specified topics.\n\
For each topic a .csv file is created. In case the specified folder does not exist, it will be created.\n\n\
Syntax: $functioname <bagpath> <folderpath> ( <topics> ... )";

# if minimum number of args is not provided print syntax
if [[ $# < 2 ]]; then
	echo -e $explainString;
	exit 1
fi

bagfile=$1
folder=$2

# if filename does not exist, exit
if [ ! -e "$bagfile" ]; then
	echo "$bagfile does not exist...exiting!";
	exit 2
fi

# remove the trailing part after the dot and absolute path, if present
bagfilename=`echo "$bagfile" | rev | cut -d '/' -f 1 | rev | cut -d '.' -f 1`;

# if specified folder does not exist, create it
if [ ! -d "$folder" ]; then
	echo "$folder is not a valid folder...creating folder!";
	mkdir "$folder"; #if permission is denied it should exit on error by itself
fi

bagTopicList=`rostopic list -b "$bagfile"`;	# just do it once, time consuming
topicList="";

if [[ $# > 2 ]]; then
	# shift the arguments and store requested topics
	shift 2
	reqTopics=$@;
	for topic in $reqTopics; do
		if ! [[ `echo $bagTopicList | grep $topic` == "" ]]; then
			topicList="$topicList $topic";
		fi
	done

	echo "Will extract topics: $topicList";
else
	topicList=$bagTopicList;
fi

for topic in $topicList; 
do 
	# discard topics referring to images
	if ! [[ $topic == *"image"* ]]; then  
		rostopic echo -p -b "$bagfile" $topic > "$folder"/"$bagfilename"\_${topic//\//_}.csv ;
	fi
done