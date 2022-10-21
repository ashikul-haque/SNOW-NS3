#!/bin/bash
cd ..
: '
export NS_LOG=snowErrorDistancePlot=level_all
echo "Running wait game 20"
./ns3 run snow-test> ../two_fold_bs_wait_time_20/log.out 2>&1


export NS_LOG=snowErrorDistancePlot=level_all
echo "Running wait game 30"
#./ns3 run snow-test_limit_30
./ns3 run snow-test_limit_30> ../two_fold_game_time_limit_30/log.out 2>&1

export NS_LOG=snowErrorDistancePlot=level_all
echo "Running jammer energy limit game"
./ns3 run snow-test-jammers-energy-limited> ../two_fold_jammers_energy_capped/log.out 2>&1
'

export NS_LOG=snowErrorDistancePlot=level_all
echo "Running only hopping game"
./ns3 run snow-test-only-hopping> ../only_hopping/log.out 2>&1

export NS_LOG=snowErrorDistancePlot=level_all
echo "Running only tx game"
./ns3 run snow-test-only-tx> ../only_tx/log.out 2>&1




