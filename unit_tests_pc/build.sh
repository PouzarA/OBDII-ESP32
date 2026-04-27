#!/usr/bin/env bash
# Jednoducha alternativa ke CMake: primo vyvola gcc s vsemi zdroji.
# Vyhoda: necim prejit na jinou platformu a nic neinstalovat.
#
# Pouziti (z adresare unit_tests_pc/):
#   ./build.sh          # prelozi
#   ./build.sh run      # prelozi a spusti testy
#   ./build.sh memcheck # prelozi s AddressSanitizer (alternativa Valgrindu) a spusti
#
set -euo pipefail
cd "$(dirname "$0")"

# SRC ukazuje o úroveň výš (do kořenu projektu)
SRC=".."
OUT="run_tests.exe"
# Výchozí flagy
CFLAGS="-std=c11 -Wall -Wextra -Werror -Wno-unused-parameter -g"
# Pokud je první argument "memcheck", zapneme AddressSanitizer
if [[ "${1:-}" == "memcheck" ]]; then
    echo "Zapinam AddressSanitizer (Memory Check)..."
    CFLAGS="$CFLAGS -fsanitize=address"
fi
gcc $CFLAGS \
    -I mocks -I "$SRC/src/core" -I "$SRC/src/isotp" -I . \
    -DUNIT_TEST_BUILD=1 \
    "$SRC/src/isotp/isotp.c" \
    "$SRC/src/core/obd2.c" \
    "$SRC/src/core/obd2_pids.c" \
    "$SRC/src/core/obd2_diag.c" \
    mocks/mock_twai.c \
    unity_lite.c \
    tests/test_isotp.c \
    tests/test_obd2_pids.c \
    tests/test_obd2_diag.c \
    tests/test_obd2.c \
    tests/test_obd2_modes.c \
    tests/test_main.c \
    -lm \
    -o "$OUT"

echo "Built: $OUT"

if [[ "${1:-}" == "run" || "${1:-}" == "memcheck" ]]; then
    ./"$OUT"
fi
