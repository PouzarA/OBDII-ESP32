#
# CMake build pro PC-hostovane unit testy ISO-TP a OBD-II vrstvy.
#
# Pouziti (z adresare unit_tests_pc/):
#   cmake -S . -B build -G "MinGW Makefiles"
#   cmake --build build
#   ./build/run_tests.exe
#
# Alternativne jeden binarka bez CMake: viz build.sh
#
cmake_minimum_required(VERSION 3.15)
project(obd2_isotp_unit_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -g)
endif()

# Zdrojovy kod pod testem — prekladame primo z nadrazeneho adresare,
# bez jakychkoli uprav. Tim je zaruceno, ze testy opravdu zkousi
# produkcni soubory a ne zkopirovanou verzi.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(UNDER_TEST_SOURCES
    ${SRC_DIR}/src/isotp/isotp.c
    ${SRC_DIR}/src/core/obd2.c
    ${SRC_DIR}/src/core/obd2_pids.c
    ${SRC_DIR}/src/core/obd2_diag.c
)

set(MOCK_SOURCES
    mocks/mock_twai.c
)

set(TEST_SOURCES
    unity_lite.c
    tests/test_isotp.c
    tests/test_obd2_pids.c
    tests/test_obd2_diag.c
    tests/test_obd2.c
    tests/test_obd2_modes.c
    tests/test_main.c
)

add_executable(run_tests
    ${UNDER_TEST_SOURCES}
    ${MOCK_SOURCES}
    ${TEST_SOURCES}
)

# Cesty: nejprve mocks/ (aby se nasly driver/twai.h a freertos/*.h
# misto skutecneho ESP-IDF), pak zdroj pod testem, pak sami testovy kod.
target_include_directories(run_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${SRC_DIR}
    ${SRC_DIR}/src/core
    ${SRC_DIR}/src/isotp
    ${SRC_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Na ESP32 je Arduino.h dostupne, na PC ne — vypne se pres #ifndef v testech.
target_compile_definitions(run_tests PRIVATE UNIT_TEST_BUILD=1)

enable_testing()
add_test(NAME run_tests COMMAND run_tests)

# --- Interaktivni interpret ---
add_executable(obd_interp
    ${UNDER_TEST_SOURCES}
    ${MOCK_SOURCES}
    interpreter/obd_interpreter.c   # <--- TADY PŘIDÁNA CESTA
)
target_include_directories(obd_interp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${SRC_DIR}
    ${SRC_DIR}/src/core
    ${SRC_DIR}/src/isotp
    ${SRC_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/interpreter )
    
target_compile_definitions(obd_interp PRIVATE UNIT_TEST_BUILD=1)
