#!/bin/bash

BASEDIR=$(cd $(dirname $0) && pwd)
cd $BASEDIR
cd ..
COMPANY=$(ls exts)

rm -rf exts/$COMPANY/$COMPANY/*

mv $BASEDIR/*.py exts/$COMPANY/$COMPANY

rm -rf Issac_Sim_Template

