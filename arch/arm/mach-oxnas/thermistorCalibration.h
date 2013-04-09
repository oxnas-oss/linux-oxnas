#ifndef __THERMISTOR_LOOKUP_TABLE_10K3A_H
#define __THERMISTOR_LOOKUP_TABLE_10K3A_H

/* Thermistor is a 10K3A*/
/* THERM_COEF_A == 0.001129241*/
/* THERM_COEF_B == 0.0002341077. */
/* THERM_COEF_C == 0.00000008775468. */

/* Capacitor is 100 nF */
/* Stepped frequency increment is 128000 Hz */
/* Schmitdt trigger threshold assumed:  1 / 3.3 V */


/* Inverse C.ln(V') == 216 */
#define THERM_INTERPOLATION_STEP  8
#define THERM_ENTRIES_IN_CALIB_TABLE  57
static const unsigned long TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE] =
{
16337 ,//    416,    /* == 143.37 	deg C: Count ==   0, R == 216 Ohms */
8707  ,//    340,    /* ==  67.07 	deg C: Count ==   8, R == 1948 Ohms */
6959  ,//    323,    /* ==  49.59 	deg C: Count ==  16, R == 3679 Ohms */
5976  ,//    313,    /* ==  39.76 	deg C: Count ==  24, R == 5410 Ohms */
5300  ,//    306,    /* ==  33 			deg C: Count ==  32, R == 7141 Ohms */
4790  ,//    301,    /* ==  27.9 		deg C: Count ==  40, R == 8873 Ohms */
4382  ,//    297,    /* ==  23.82 	deg C: Count ==  48, R == 10604 Ohms */
4043  ,//    293,    /* ==  20.43 	deg C: Count ==  56, R == 12335 Ohms */
3755  ,//    291,    /* ==  17.55 	deg C: Count ==  64, R == 14066 Ohms */
3504  ,//    288,    /* ==  15.04 	deg C: Count ==  72, R == 15798 Ohms */
3282  ,//    286,    /* ==  12.82 	deg C: Count ==  80, R == 17529 Ohms */
3084  ,//    284,    /* ==  10.84 	deg C: Count ==  88, R == 19260 Ohms */
2904  ,//    282,    /* ==   9.04 	deg C: Count ==  96, R == 20991 Ohms */
2741  ,//    280,    /* ==   7.41 	deg C: Count == 104, R == 22722 Ohms */
2591  ,//    279,    /* ==   5.91 	deg C: Count == 112, R == 24454 Ohms */
2453  ,//    278,    /* ==   4.53 	deg C: Count == 120, R == 26185 Ohms */
2325  ,//    276,    /* ==   3.25 	deg C: Count == 128, R == 27916 Ohms */
2205  ,//    275,    /* ==   2.05 	deg C: Count == 136, R == 29647 Ohms */
2093  ,//    274,    /* ==   0.93 	deg C: Count == 144, R == 31379 Ohms */
1988  ,//    273,    /* ==  -0.12 	deg C: Count == 152, R == 33110 Ohms */
1888  ,//    272,    /* ==  -1.12 	deg C: Count == 160, R == 34841 Ohms */
1794  ,//    271,    /* ==  -2.06 	deg C: Count == 168, R == 36572 Ohms */
1705  ,//    270,    /* ==  -2.95 	deg C: Count == 176, R == 38304 Ohms */
1620  ,//    269,    /* ==  -3.8 		deg C: Count == 184, R == 40035 Ohms */
1540  ,//    268,    /* ==  -4.6 		deg C: Count == 192, R == 41766 Ohms */
1463  ,//    268,    /* ==  -5.37 	deg C: Count == 200, R == 43497 Ohms */
1389  ,//    267,    /* ==  -6.11 	deg C: Count == 208, R == 45229 Ohms */
1319  ,//    266,    /* ==  -6.81 	deg C: Count == 216, R == 46960 Ohms */
1251  ,//    266,    /* ==  -7.49 	deg C: Count == 224, R == 48691 Ohms */
1186  ,//    265,    /* ==  -8.14 	deg C: Count == 232, R == 50422 Ohms */
1123  ,//    264,    /* ==  -8.77 	deg C: Count == 240, R == 52154 Ohms */
1063  ,//    264,    /* ==  -9.37 	deg C: Count == 248, R == 53885 Ohms */
1005  ,//    263,    /* ==  -9.95 	deg C: Count == 256, R == 55616 Ohms */
948   ,//    262,    /* == -10.52 	deg C: Count == 264, R == 57347 Ohms */
894   ,//    262,    /* == -11.06 	deg C: Count == 272, R == 59078 Ohms */
841   ,//    261,    /* == -11.59 	deg C: Count == 280, R == 60810 Ohms */
790   ,//    261,    /* == -12.1 		deg C: Count == 288, R == 62541 Ohms */
741   ,//    260,    /* == -12.59 	deg C: Count == 296, R == 64272 Ohms */
693   ,//    260,    /* == -13.07 	deg C: Count == 304, R == 66003 Ohms */
647   ,//    259,    /* == -13.53 	deg C: Count == 312, R == 67735 Ohms */
601   ,//    259,    /* == -13.99 	deg C: Count == 320, R == 69466 Ohms */
557   ,//    259,    /* == -14.43 	deg C: Count == 328, R == 71197 Ohms */
514   ,//    258,    /* == -14.86 	deg C: Count == 336, R == 72928 Ohms */
473   ,//    258,    /* == -15.27 	deg C: Count == 344, R == 74660 Ohms */
432   ,//    257,    /* == -15.68 	deg C: Count == 352, R == 76391 Ohms */
392   ,//    257,    /* == -16.08 	deg C: Count == 360, R == 78122 Ohms */
354   ,//    257,    /* == -16.46 	deg C: Count == 368, R == 79853 Ohms */
316   ,//    256,    /* == -16.84 	deg C: Count == 376, R == 81585 Ohms */
279   ,//    256,    /* == -17.21 	deg C: Count == 384, R == 83316 Ohms */
243   ,//    255,    /* == -17.57 	deg C: Count == 392, R == 85047 Ohms */
208   ,//    255,    /* == -17.92 	deg C: Count == 400, R == 86778 Ohms */
174   ,//    255,    /* == -18.26 	deg C: Count == 408, R == 88510 Ohms */
140   ,//    254,    /* == -18.6 		deg C: Count == 416, R == 90241 Ohms */
107   ,//    254,    /* == -18.93 	deg C: Count == 424, R == 91972 Ohms */
75    ,//    254,    /* == -19.25 	deg C: Count == 432, R == 93703 Ohms */
43    ,//    253,    /* == -19.57 	deg C: Count == 440, R == 95434 Ohms */
12    //    253,     /* == -19.88 	deg C: Count == 448, R == 97166 Ohms */
};

#endif
