#!/bin/bash
varname=y
if [ -d ~/.gazebo/models/psm ] || [ -d ~/.gazebo/models/mtm ] || [ -d ~/.gazebo/models/ecm ]\
	|| [ -d ~/.gazebo/models/suj ] || [ -d ~/.gazebo/models/dvrk ]
then
	echo There are existing models in ~/.gazebo/models. Overwrite\? [y/n]
	read varname
fi 

if [ $varname == "y" ]; then
 	cp -a dvrk/ ecm/ mtm/ psm/ suj/ tool_lnd/ ~/.gazebo/models
 	echo Models copied to: ~/.gazebo/models
fi