/**
 * @file test_main.c
 * @brief Hlavni runner vsech testovych suit.
 *
 * Kazda .c soubor v tests/ registruje sve testy do funkce run_XXX_tests(),
 * ktera se zavola z tohoto runneru. Vyhoda: pridani nove suity vyzaduje
 * jen jednu novou extern deklaraci a RUN_TEST blok nize, ne editaci
 * CMakeLists ani build.sh.
 *
 * Navratova hodnota procesu:
 *   0  = vsechny testy prosly
 *   >0 = pocet padlych testu (1..N)
 *
 * Tohle umoznuje CI/CD pipeline pouzit:
 *   ./run_tests.exe && echo "PASS" || echo "FAIL"
 */

#include "unity_lite.h"

/* Extern deklarace runneru z jednotlivych souboru v tests/. */
extern void run_isotp_tests(void);
extern void run_obd2_pids_tests(void);
extern void run_obd2_diag_tests(void);
extern void run_obd2_tests(void);
extern void run_obd2_modes_tests(void);

int main(void)
{
    UNITY_BEGIN();

    /* ------------------------------------------------------------ */
    /*  1) Transportni vrstva (ISO-TP nad mockovanym TWAI)          */
    /* ------------------------------------------------------------ */
    printf("\n### ISO-TP transport layer ###\n");
    run_isotp_tests();

    /* ------------------------------------------------------------ */
    /*  2) PID dekodery (ciste, bez komunikace)                     */
    /* ------------------------------------------------------------ */
    printf("\n### OBD-II PID decoders ###\n");
    run_obd2_pids_tests();

    /* ------------------------------------------------------------ */
    /*  3) DTC dekodery (ciste)                                     */
    /* ------------------------------------------------------------ */
    printf("\n### OBD-II DTC decoders ###\n");
    run_obd2_diag_tests();

    /* ------------------------------------------------------------ */
    /*  4) OBD-II integracni testy nad mockovanou ISO-TP            */
    /* ------------------------------------------------------------ */
    printf("\n### OBD-II integration (mocked ISO-TP) ###\n");
    run_obd2_tests();

    /* ------------------------------------------------------------ */
    /*  5) OBD-II ostatni mody, NRC, ECU binding, multi-ECU         */
    /* ------------------------------------------------------------ */
    printf("\n### OBD-II modes 02/07/0A, NRC, binding, multi-ECU ###\n");
    run_obd2_modes_tests();

    return UNITY_END();
}
