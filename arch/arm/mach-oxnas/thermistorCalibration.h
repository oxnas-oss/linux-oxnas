#ifndef __THERMISTOR_LOOKUP_TABLE_10K3A_H
#define __THERMISTOR_LOOKUP_TABLE_10K3A_H

/* Thermistor is a 10K3A  */
/* THERM_COEF_A == 0.001129241*/
/* THERM_COEF_B == 0.0002341077. */
/* THERM_COEF_C == 0.00000008775468. */

/* Capacitor is 100 nF */
/* Stepped frequency increment is 128000 Hz */
/* Schmitdt trigger threshold assumed 1.65 / 3.3 V */


/* Inverse C.ln(V') == 113 */

#define THERM_INTERPOLATION_STEP        8
#define THERM_ENTRIES_IN_CALIB_TABLE       128
static const unsigned long TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE] = {
    446,    /* == 172.51 deg C: Count == 0, R == 113 Ohms */
    415,    /* == 141.66 deg C: Count == 8, R == 225 Ohms */
    398,    /* == 125.43 deg C: Count == 16, R == 338 Ohms */
    388,    /* == 114.61 deg C: Count == 24, R == 451 Ohms */
    380,    /* == 106.59 deg C: Count == 32, R == 564 Ohms */
    373,    /* == 100.26 deg C: Count == 40, R == 676 Ohms */
    368,    /* == 95.06 deg C: Count == 48, R == 789 Ohms */
    364,    /* == 90.66 deg C: Count == 56, R == 902 Ohms */
    360,    /* == 86.86 deg C: Count == 64, R == 1014 Ohms */
    357,    /* == 83.52 deg C: Count == 72, R == 1127 Ohms */
    354,    /* == 80.55 deg C: Count == 80, R == 1240 Ohms */
    351,    /* == 77.88 deg C: Count == 88, R == 1353 Ohms */
    348,    /* == 75.45 deg C: Count == 96, R == 1465 Ohms */
    346,    /* == 73.23 deg C: Count == 104, R == 1578 Ohms */
    344,    /* == 71.19 deg C: Count == 112, R == 1691 Ohms */
    342,    /* == 69.3 deg C: Count == 120, R == 1803 Ohms */
    341,    /* == 67.54 deg C: Count == 128, R == 1916 Ohms */
    339,    /* == 65.89 deg C: Count == 136, R == 2029 Ohms */
    337,    /* == 64.35 deg C: Count == 144, R == 2142 Ohms */
    336,    /* == 62.9 deg C: Count == 152, R == 2254 Ohms */
    335,    /* == 61.53 deg C: Count == 160, R == 2367 Ohms */
    333,    /* == 60.23 deg C: Count == 168, R == 2480 Ohms */
    332,    /* == 59 deg C: Count == 176, R == 2592 Ohms */
    331,    /* == 57.83 deg C: Count == 184, R == 2705 Ohms */
    330,    /* == 56.72 deg C: Count == 192, R == 2818 Ohms */
    329,    /* == 55.65 deg C: Count == 200, R == 2930 Ohms */
    328,    /* == 54.63 deg C: Count == 208, R == 3043 Ohms */
    327,    /* == 53.65 deg C: Count == 216, R == 3156 Ohms */
    326,    /* == 52.71 deg C: Count == 224, R == 3269 Ohms */
    325,    /* == 51.81 deg C: Count == 232, R == 3381 Ohms */
    324,    /* == 50.95 deg C: Count == 240, R == 3494 Ohms */
    323,    /* == 50.11 deg C: Count == 248, R == 3607 Ohms */
    322,    /* == 49.3 deg C: Count == 256, R == 3719 Ohms */
    322,    /* == 48.52 deg C: Count == 264, R == 3832 Ohms */
    321,    /* == 47.77 deg C: Count == 272, R == 3945 Ohms */
    320,    /* == 47.04 deg C: Count == 280, R == 4058 Ohms */
    319,    /* == 46.33 deg C: Count == 288, R == 4170 Ohms */
    319,    /* == 45.65 deg C: Count == 296, R == 4283 Ohms */
    318,    /* == 44.98 deg C: Count == 304, R == 4396 Ohms */
    317,    /* == 44.34 deg C: Count == 312, R == 4508 Ohms */
    317,    /* == 43.71 deg C: Count == 320, R == 4621 Ohms */
    316,    /* == 43.1 deg C: Count == 328, R == 4734 Ohms */
    316,    /* == 42.51 deg C: Count == 336, R == 4847 Ohms */
    315,    /* == 41.93 deg C: Count == 344, R == 4959 Ohms */
    314,    /* == 41.36 deg C: Count == 352, R == 5072 Ohms */
    314,    /* == 40.82 deg C: Count == 360, R == 5185 Ohms */
    313,    /* == 40.28 deg C: Count == 368, R == 5297 Ohms */
    313,    /* == 39.76 deg C: Count == 376, R == 5410 Ohms */
    312,    /* == 39.25 deg C: Count == 384, R == 5523 Ohms */
    312,    /* == 38.75 deg C: Count == 392, R == 5636 Ohms */
    311,    /* == 38.26 deg C: Count == 400, R == 5748 Ohms */
    311,    /* == 37.78 deg C: Count == 408, R == 5861 Ohms */
    310,    /* == 37.32 deg C: Count == 416, R == 5974 Ohms */
    310,    /* == 36.86 deg C: Count == 424, R == 6086 Ohms */
    309,    /* == 36.41 deg C: Count == 432, R == 6199 Ohms */
    309,    /* == 35.97 deg C: Count == 440, R == 6312 Ohms */
    309,    /* == 35.55 deg C: Count == 448, R == 6425 Ohms */
    308,    /* == 35.12 deg C: Count == 456, R == 6537 Ohms */
    308,    /* == 34.71 deg C: Count == 464, R == 6650 Ohms */
    307,    /* == 34.31 deg C: Count == 472, R == 6763 Ohms */
    307,    /* == 33.91 deg C: Count == 480, R == 6875 Ohms */
    307,    /* == 33.52 deg C: Count == 488, R == 6988 Ohms */
    306,    /* == 33.14 deg C: Count == 496, R == 7101 Ohms */
    306,    /* == 32.76 deg C: Count == 504, R == 7213 Ohms */
    305,    /* == 32.39 deg C: Count == 512, R == 7326 Ohms */
    305,    /* == 32.03 deg C: Count == 520, R == 7439 Ohms */
    305,    /* == 31.67 deg C: Count == 528, R == 7552 Ohms */
    304,    /* == 31.32 deg C: Count == 536, R == 7664 Ohms */
    304,    /* == 30.98 deg C: Count == 544, R == 7777 Ohms */
    304,    /* == 30.64 deg C: Count == 552, R == 7890 Ohms */
    303,    /* == 30.31 deg C: Count == 560, R == 8002 Ohms */
    303,    /* == 29.98 deg C: Count == 568, R == 8115 Ohms */
    303,    /* == 29.66 deg C: Count == 576, R == 8228 Ohms */
    302,    /* == 29.34 deg C: Count == 584, R == 8341 Ohms */
    302,    /* == 29.02 deg C: Count == 592, R == 8453 Ohms */
    302,    /* == 28.72 deg C: Count == 600, R == 8566 Ohms */
    301,    /* == 28.41 deg C: Count == 608, R == 8679 Ohms */
    301,    /* == 28.11 deg C: Count == 616, R == 8791 Ohms */
    301,    /* == 27.82 deg C: Count == 624, R == 8904 Ohms */
    301,    /* == 27.53 deg C: Count == 632, R == 9017 Ohms */
    300,    /* == 27.24 deg C: Count == 640, R == 9130 Ohms */
    300,    /* == 26.96 deg C: Count == 648, R == 9242 Ohms */
    300,    /* == 26.68 deg C: Count == 656, R == 9355 Ohms */
    299,    /* == 26.4 deg C: Count == 664, R == 9468 Ohms */
    299,    /* == 26.13 deg C: Count == 672, R == 9580 Ohms */
    299,    /* == 25.86 deg C: Count == 680, R == 9693 Ohms */
    299,    /* == 25.6 deg C: Count == 688, R == 9806 Ohms */
    298,    /* == 25.34 deg C: Count == 696, R == 9919 Ohms */
    298,    /* == 25.08 deg C: Count == 704, R == 10031 Ohms */
    298,    /* == 24.82 deg C: Count == 712, R == 10144 Ohms */
    298,    /* == 24.57 deg C: Count == 720, R == 10257 Ohms */
    297,    /* == 24.33 deg C: Count == 728, R == 10369 Ohms */
    297,    /* == 24.08 deg C: Count == 736, R == 10482 Ohms */
    297,    /* == 23.84 deg C: Count == 744, R == 10595 Ohms */
    297,    /* == 23.6 deg C: Count == 752, R == 10708 Ohms */
    296,    /* == 23.36 deg C: Count == 760, R == 10820 Ohms */
    296,    /* == 23.13 deg C: Count == 768, R == 10933 Ohms */
    296,    /* == 22.9 deg C: Count == 776, R == 11046 Ohms */
    296,    /* == 22.67 deg C: Count == 784, R == 11158 Ohms */
    295,    /* == 22.44 deg C: Count == 792, R == 11271 Ohms */
    295,    /* == 22.22 deg C: Count == 800, R == 11384 Ohms */
    295,    /* == 22 deg C: Count == 808, R == 11496 Ohms */
    295,    /* == 21.78 deg C: Count == 816, R == 11609 Ohms */
    295,    /* == 21.57 deg C: Count == 824, R == 11722 Ohms */
    294,    /* == 21.35 deg C: Count == 832, R == 11835 Ohms */
    294,    /* == 21.14 deg C: Count == 840, R == 11947 Ohms */
    294,    /* == 20.93 deg C: Count == 848, R == 12060 Ohms */
    294,    /* == 20.73 deg C: Count == 856, R == 12173 Ohms */
    294,    /* == 20.52 deg C: Count == 864, R == 12285 Ohms */
    293,    /* == 20.32 deg C: Count == 872, R == 12398 Ohms */
    293,    /* == 20.12 deg C: Count == 880, R == 12511 Ohms */
    293,    /* == 19.92 deg C: Count == 888, R == 12624 Ohms */
    293,    /* == 19.72 deg C: Count == 896, R == 12736 Ohms */
    293,    /* == 19.53 deg C: Count == 904, R == 12849 Ohms */
    292,    /* == 19.34 deg C: Count == 912, R == 12962 Ohms */
    292,    /* == 19.15 deg C: Count == 920, R == 13074 Ohms */
    292,    /* == 18.96 deg C: Count == 928, R == 13187 Ohms */
    292,    /* == 18.77 deg C: Count == 936, R == 13300 Ohms */
    292,    /* == 18.59 deg C: Count == 944, R == 13413 Ohms */
    291,    /* == 18.4 deg C: Count == 952, R == 13525 Ohms */
    291,    /* == 18.22 deg C: Count == 960, R == 13638 Ohms */
    291,    /* == 18.04 deg C: Count == 968, R == 13751 Ohms */
    291,    /* == 17.86 deg C: Count == 976, R == 13863 Ohms */
    291,    /* == 17.69 deg C: Count == 984, R == 13976 Ohms */
    291,    /* == 17.51 deg C: Count == 992, R == 14089 Ohms */
    290,    /* == 17.34 deg C: Count == 1000, R == 14202 Ohms */
    290,    /* == 17.17 deg C: Count == 1008, R == 14314 Ohms */
    290,    /* == 16.99 deg C: Count == 1016, R == 14427 Ohms */
};
#endif  // __THERMISTOR_LOOKUP_TABLE_10K3A_H

