#!/bin/bash

TEST=broadphasetest
COMPILER_GCC=g++
COMPILER_INTEL=/opt/intel/cc/10.0.023/bin/icpc
COMPILER=$COMPILER_GCC

if [ "$1" == "h" ] ; then
    echo "Usage:"
    echo "go.sh [evclh] [gi] [<Binary=$TEST>]"
    echo -e "\te = only execute (default)"
    echo -e "\tv = calls Valgrind"
    echo -e "\tc = calls Cachegrind"
    echo -e "\tl = calls Callgrind"
    echo 
    echo -e "\ti = use intel"
    echo -e "\tg = use g++ (default)"
    echo 
    echo -e "\th = this help"
    exit
fi

if [ "$2" == "i" ] ; then
    COMPILER=$COMPILER_INTEL
elif [ "$2" == "g" ] ; then 
    COMPILER=$COMPILER_GCC
elif [ "$2" != "" ] ; then
    TEST=$2
fi

if [ "$3" != "" ] ; then
    TEST=$3
fi

rm -f $TEST
rm -f callgrind.*
rm -f cachegrind.*

$COMPILER $TEST.cpp -I../d-collide -I.. -L../d-collide/ -ldcollide -lpthread -g -O0 -o $TEST

if [ "$1" == "v" ] ; then
    valgrind --leak-check=full ./$TEST
elif [ "$1" == "c" ] ; then
    valgrind --tool=cachegrind ./$TEST 2>tt
    PID=`head -1 tt | sed -e 's/==//g' | sed -e 's/Cache.*$//g'`
    rm -f tt
    cg_annotate --$PID $TEST.cpp
elif [ "$1" == "l" ] ; then
    valgrind --tool=callgrind ./$TEST 2>tt
    PID=`head -1 tt | sed -e 's/==//g' | sed -e 's/Call.*$//g'`
    rm -f tt
    kcachegrind callgrind.out.$PID &
elif [ "$1" == "e" ] ; then
    ./$TEST
else
    ./go.sh h
fi
