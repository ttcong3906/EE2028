#!/bin/bash
# Logic tests of Core/Src/samplecode.c on the PC (WSL/Linux, gcc). No board needed.
#   bash tools/sim/run_tests.sh
set -e
cd "$(dirname "$0")"
B=build/proj
mkdir -p $B/Core/Src $B/Drivers/BSP/B-L4S5I-IOT01
cp ../../Core/Src/samplecode.c $B/Core/Src/main.c
for h in stm32l4s5i_iot01 stm32l4s5i_iot01_accelero stm32l4s5i_iot01_gyro stm32l4s5i_iot01_magneto \
         stm32l4s5i_iot01_tsensor stm32l4s5i_iot01_hsensor stm32l4s5i_iot01_psensor; do
  echo "/* empty: the HAL/BSP stubs live in stub/main.h */" > $B/Drivers/BSP/B-L4S5I-IOT01/$h.h
done
gcc -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -g -Istub -Ibuild -o build/sim sim.c -lm
./build/sim
