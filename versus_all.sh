#!/bin/bash

player1=$1
cmd1=$2
player2=$3
cmd2=$4

echo $player1
echo $cmd1
echo $player2
echo $cmd2

result_dir=result_${player1}_${player2}

parallel --progress --results $result_dir python3 nodeless.py $player1 "'$cmd1'" $player2 "'$cmd2'" --seed {1} --nodes {2} --prob {3} ::: `seq 1 20` ::: 30 50 100 200 400 ::: 0.1 0.2 0.3