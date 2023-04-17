#!/bin/bash

export NS_LOG=snowErrorDistancePlot=level_error
echo "Running game with interval 10"
./ns3 run "snow-test-multiple-nodes --interval=10"
#sleep 6m
echo "Running game with interval 20"
./ns3 run "snow-test-multiple-nodes --interval=20"
#sleep 6m
echo "Running game with interval 30"
./ns3 run "snow-test-multiple-nodes --interval=30"
#sleep 6m
echo "Running game with interval 40"
./ns3 run "snow-test-multiple-nodes --interval=40"
#sleep 6m
echo "Running game with interval 50"
./ns3 run "snow-test-multiple-nodes --interval=50"
#sleep 6m
echo "Running game with interval 60"
./ns3 run "snow-test-multiple-nodes --interval=60"
#sleep 6m
#> log.out 2>&1

#./ns3 run snow-test-multiple-nodes> log2.out 2>&1
#NS_LOG=snowErrorDistancePlot=level_all:snowPhy=level_all:snowMac=level_all



