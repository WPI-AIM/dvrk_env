#!/bin/bash
varname=y

if [ ! -d ~/.gazebo1 ]; then
	echo "~/.gazebo directory doesn't exist. Are you sure gazebo is installed?"
fi

if [ -d ~/.gazebo/models/psm ] || [ -d ~/.gazebo/models/mtm ] || [ -d ~/.gazebo/models/ecm ]\
	|| [ -d ~/.gazebo/models/suj ] || [ -d ~/.gazebo/models/dvrk ]
then
	echo There are existing models in ~/.gazebo/models. Overwrite\? [y/n]
	read varname
fi 

if [ $varname == "y" ]; then
	if [ ! -d ~/.gazebo/models ]; then
		mkdir -p ~/.gazebo/models
	fi
 	cp -a dvrk/ ecm/ mtm/ psm/ suj/ tool_lnd/ ~/.gazebo/models
 	echo Models copied to: ~/.gazebo/models
fi
