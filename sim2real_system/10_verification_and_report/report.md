# Held-out verification — concrete

2026-11-12 08:41 · fresh real trials: T1_straight ×3, T2_spin ×3, T3_speed_sweep ×3, T3b_top_speed ×3, T4_arc ×3, T5_stop ×3, T6_spin_sweep ×3, R1_straight_fb ×3, R2_spin_rates ×3, R3_arcs_slow ×3, R4_arcs_fast ×3, R5_nav_pattern ×3

Changed parameters: `wheel_distance` 0.5 → 0.3192, `max_linear_speed` 0.5 → 0.62, `max_angular_speed` 1.9 → 2.78, `max_acceleration` 5 → 2.562, `max_deceleration` 8 → 3.974, `max_angular_acceleration` 20 → 5.864, `corner_mu_dynamic` 1 → 1.132, `corner_mu_static` 1.2 → 1.17, `middle_mu_dynamic` 1 → 0.9325, `middle_mu_static` 1.2 → 1.112, `floor_mu_dynamic` 0.675 → 0.6757, `floor_mu_static` 0.8 → 0.8388, `drive_damping` 1e+04 → 4738, `drive_max_force` 50 → 8.853, `base_linear_damping` 0 → 3.6, `base_angular_damping` 0.05 → 3.645

| metric | baseline sim | calibrated sim |
|---|---|---|
| rms feature error (σ units) | 4.97 | 2.41 |
| T1_straight endpoint error | 11.3 cm | 10.9 cm |
| T1_straight mean path error | 6.7 cm | 6.4 cm |
| T1_straight final heading error | 12.9 ° | 12.9 ° |
| T2_spin endpoint error | 5.7 cm | 5.9 cm |
| T2_spin mean path error | 3.8 cm | 3.5 cm |
| T2_spin final heading error | 68.9 ° | 45.5 ° |
| T3_speed_sweep endpoint error | 8.9 cm | 9.1 cm |
| T3_speed_sweep mean path error | 4.3 cm | 4.4 cm |
| T3_speed_sweep final heading error | 8.6 ° | 8.6 ° |
| T3b_top_speed endpoint error | 6.7 cm | 6.7 cm |
| T3b_top_speed mean path error | 5.2 cm | 4.8 cm |
| T3b_top_speed final heading error | 5.0 ° | 3.2 ° |
| T4_arc endpoint error | 25.3 cm | 26.2 cm |
| T4_arc mean path error | 12.4 cm | 11.7 cm |
| T4_arc final heading error | 49.0 ° | 53.4 ° |
| T5_stop endpoint error | 6.1 cm | 6.3 cm |
| T5_stop mean path error | 4.1 cm | 4.2 cm |
| T5_stop final heading error | 8.1 ° | 8.1 ° |
| T6_spin_sweep endpoint error | 5.0 cm | 4.0 cm |
| T6_spin_sweep mean path error | 2.8 cm | 2.3 cm |
| T6_spin_sweep final heading error | 38.7 ° | 24.5 ° |
| R1_straight_fb endpoint error | 4.5 cm | 3.8 cm |
| R1_straight_fb mean path error | 3.1 cm | 3.5 cm |
| R1_straight_fb final heading error | 8.9 ° | 9.0 ° |
| R2_spin_rates endpoint error | 5.5 cm | 11.2 cm |
| R2_spin_rates mean path error | 2.3 cm | 3.2 cm |
| R2_spin_rates final heading error | 12.3 ° | 7.1 ° |
| R3_arcs_slow endpoint error | 7.6 cm | 7.4 cm |
| R3_arcs_slow mean path error | 12.4 cm | 6.6 cm |
| R3_arcs_slow final heading error | 28.9 ° | 32.6 ° |
| R4_arcs_fast endpoint error | 12.8 cm | 12.3 cm |
| R4_arcs_fast mean path error | 6.3 cm | 6.6 cm |
| R4_arcs_fast final heading error | 9.0 ° | 8.3 ° |
| R5_nav_pattern endpoint error | 8.6 cm | 8.0 cm |
| R5_nav_pattern mean path error | 16.5 cm | 7.2 cm |
| R5_nav_pattern final heading error | 18.1 ° | 15.2 ° |

## Per-feature (real mean vs sim)

| test.feature | real | baseline sim | calibrated sim | z baseline | z calibrated |
|---|---|---|---|---|---|
| T1_straight.tau_v | 0.0902 ± 0.0772 | 0.0291 | 0.0583 | -0.79 | -0.41 |
| T1_straight.v_ss | 0.1414 ± 0.0167 | 0.1492 | 0.1492 | 0.46 | 0.46 |
| T1_straight.dist | 0.5705 ± 0.0526 | 0.5805 | 0.5723 | 0.19 | 0.03 |
| T2_spin.dyaw | 2.0876 ± 0.0952 | 3.2713 | 1.2833 | 6.79 | -4.61 |
| T2_spin.turn_walk | 0.0531 ± 0.0275 | 0.0652 | 0.0868 | 0.3 | 0.84 |
| T2_spin.omega_mean | 0.2610 ± 0.0119 | 0.4089 | 0.1604 | 12.44 | -8.45 |
| T3_speed_sweep.v_ss_s3 | 0.1990 ± 0.0040 | 0.1983 | 0.1984 | -0.17 | -0.15 |
| T3_speed_sweep.v_ss_s2 | 0.1413 ± 0.0115 | 0.1488 | 0.1492 | 0.66 | 0.69 |
| T3_speed_sweep.v_ss_s1 | 0.0955 ± 0.0095 | 0.0994 | 0.0994 | 0.42 | 0.41 |
| T3_speed_sweep.v_ss_s0 | 0.0565 ± 0.0211 | 0.0498 | 0.0498 | -0.32 | -0.32 |
| T3b_top_speed.v_ss_max | 0.6332 ± 0.0127 | 0.4943 | 0.5919 | -10.97 | -3.26 |
| T5_stop.coast_time | 0.1228 ± 0.0167 | 0.0583 | 0.0583 | -3.87 | -3.87 |
| T5_stop.coast_dist | 0.0065 ± 0.0010 | 0.0039 | 0.0058 | -2.63 | -0.69 |
| T6_spin_sweep.omega_s3 | 0.7054 ± 0.0405 | 2.0666 | 1.0473 | 33.64 | 8.45 |
| T6_spin_sweep.omega_s2 | 0.4840 ± 0.0897 | 1.4980 | 0.3981 | 11.31 | -0.96 |
| T6_spin_sweep.omega_s1 | 0.4129 ± 0.1079 | 0.7898 | 0.3809 | 3.49 | -0.3 |
| T6_spin_sweep.omega_s0 | 0.1515 ± 0.0653 | 0.2312 | 0.0528 | 1.22 | -1.51 |
| R1_straight_fb.w_k11 | -0.0072 ± 0.0678 | -0.0000 | 0.0003 | 0.11 | 0.11 |
| R1_straight_fb.v_k5 | 0.1870 ± 0.0121 | 0.1985 | 0.1986 | 0.86 | 0.87 |
| R1_straight_fb.v_k1 | 0.0931 ± 0.0061 | 0.0994 | 0.0995 | 0.94 | 0.96 |
| R1_straight_fb.w_k5 | -0.0101 ± 0.0347 | -0.0005 | 0.0002 | 0.28 | 0.3 |
| R1_straight_fb.w_k7 | 0.0063 ± 0.0290 | 0.0003 | 0.0000 | -0.21 | -0.22 |
| R1_straight_fb.w_k9 | -0.0144 ± 0.0200 | -0.0001 | -0.0001 | 0.71 | 0.71 |
| R1_straight_fb.w_k1 | -0.0000 ± 0.0234 | 0.0001 | 0.0000 | 0.01 | 0.0 |
| R1_straight_fb.v_k9 | 0.2538 ± 0.0237 | 0.2577 | 0.2579 | 0.16 | 0.17 |
| R1_straight_fb.v_k11 | -0.2664 ± 0.0124 | -0.2581 | -0.2580 | 0.56 | 0.57 |
| R1_straight_fb.v_k3 | -0.0883 ± 0.0121 | -0.0994 | -0.0993 | -0.9 | -0.89 |
| R1_straight_fb.v_k7 | -0.1998 ± 0.0149 | -0.1975 | -0.1988 | 0.14 | 0.06 |
| R1_straight_fb.w_k3 | 0.0267 ± 0.0502 | 0.0000 | -0.0001 | -0.53 | -0.53 |
| R2_spin_rates.w_k11 | -0.3926 ± 0.0717 | -0.7617 | -0.2681 | -4.52 | 1.52 |
| R2_spin_rates.w_k5 | 0.1963 ± 0.0592 | 0.3154 | 0.1644 | 1.91 | -0.51 |
| R2_spin_rates.w_k7 | -0.2299 ± 0.0481 | -0.2201 | -0.1066 | 0.19 | 2.31 |
| R2_spin_rates.w_k9 | 0.3642 ± 0.0976 | 0.8606 | 0.3168 | 4.77 | -0.46 |
| R2_spin_rates.w_k13 | 0.4367 ± 0.0358 | 1.2077 | 0.4303 | 13.65 | -0.11 |
| R2_spin_rates.w_k1 | 0.1727 ± 0.0695 | 0.2887 | 0.0764 | 1.62 | -1.34 |
| R2_spin_rates.w_k15 | -0.4853 ± 0.1057 | -1.1949 | -0.4367 | -6.1 | 0.42 |
| R2_spin_rates.w_k3 | -0.1543 ± 0.0378 | -0.1909 | -0.0961 | -0.9 | 1.43 |
| R3_arcs_slow.w_k11 | -0.1768 ± 0.0911 | -0.0431 | -0.0133 | 1.44 | 1.76 |
| R3_arcs_slow.v_k23 | -0.1259 ± 0.0168 | -0.2154 | -0.1308 | -5.21 | -0.28 |
| R3_arcs_slow.w_k23 | 0.1181 ± 0.0953 | 0.0215 | 0.0368 | -1.01 | -0.85 |
| R3_arcs_slow.v_k5 | 0.1022 ± 0.0165 | 0.0878 | 0.1093 | -0.85 | 0.43 |
| R3_arcs_slow.v_k13 | 0.1170 ± 0.0050 | 0.1198 | 0.1214 | 0.45 | 0.73 |
| R3_arcs_slow.v_k1 | 0.1115 ± 0.0073 | 0.1192 | 0.1207 | 0.97 | 1.15 |
| R3_arcs_slow.w_k5 | 0.0467 ± 0.0280 | 0.0167 | 0.0071 | -1.06 | -1.4 |
| R3_arcs_slow.w_k7 | -0.1306 ± 0.0674 | -0.0175 | -0.0083 | 1.65 | 1.78 |
| R3_arcs_slow.w_k9 | 0.1940 ± 0.0506 | 0.0375 | 0.0172 | -2.89 | -3.26 |
| R3_arcs_slow.w_k13 | -0.0348 ± 0.0512 | -0.0050 | -0.0012 | 0.58 | 0.65 |
| R3_arcs_slow.w_k1 | 0.0613 ± 0.0668 | 0.0043 | 0.0008 | -0.85 | -0.9 |
| R3_arcs_slow.w_k15 | 0.0548 ± 0.0415 | 0.0045 | 0.0011 | -1.2 | -1.28 |
| R3_arcs_slow.w_k17 | -0.1308 ± 0.0216 | -0.0319 | -0.0103 | 3.92 | 4.77 |
| R3_arcs_slow.v_k21 | 0.1076 ± 0.0114 | 0.1881 | 0.1527 | 6.79 | 3.8 |
| R3_arcs_slow.v_k9 | 0.1238 ± 0.0056 | 0.2083 | 0.0824 | 12.62 | -6.18 |
| R3_arcs_slow.v_k11 | -0.1101 ± 0.0050 | -0.1441 | -0.0741 | -5.68 | 6.01 |
| R3_arcs_slow.v_k15 | -0.1071 ± 0.0160 | -0.1156 | -0.1198 | -0.52 | -0.78 |
| R3_arcs_slow.v_k3 | -0.1097 ± 0.0091 | -0.1149 | -0.1168 | -0.54 | -0.73 |
| R3_arcs_slow.v_k7 | -0.1140 ± 0.0136 | -0.0831 | -0.1055 | 2.2 | 0.61 |
| R3_arcs_slow.w_k3 | -0.0754 ± 0.0802 | -0.0043 | -0.0040 | 0.88 | 0.89 |
| R3_arcs_slow.w_k19 | 0.0785 ± 0.0200 | 0.0194 | 0.0113 | -2.75 | -3.12 |
| R3_arcs_slow.w_k21 | -0.2810 ± 0.0952 | -0.0371 | -0.0291 | 2.46 | 2.54 |
| R3_arcs_slow.v_k17 | 0.1184 ± 0.0146 | 0.1110 | 0.1319 | -0.49 | 0.9 |
| R3_arcs_slow.v_k19 | -0.1142 ± 0.0050 | -0.1004 | -0.1214 | 2.28 | -1.18 |
| R4_arcs_fast.w_k11 | -0.1646 ± 0.0378 | -0.0339 | -0.0162 | 3.17 | 3.6 |
| R4_arcs_fast.v_k23 | -0.1969 ± 0.0139 | -0.1907 | -0.2200 | 0.41 | -1.53 |
| R4_arcs_fast.w_k23 | 0.1604 ± 0.0373 | 0.0473 | 0.0335 | -2.79 | -3.13 |
| R4_arcs_fast.v_k5 | 0.1803 ± 0.0050 | 0.1847 | 0.1945 | 0.6 | 1.94 |
| R4_arcs_fast.v_k13 | 0.2359 ± 0.0050 | 0.2577 | 0.2577 | 2.52 | 2.52 |
| R4_arcs_fast.v_k1 | 0.2488 ± 0.0263 | 0.2556 | 0.2573 | 0.25 | 0.31 |
| R4_arcs_fast.w_k5 | 0.0927 ± 0.0231 | 0.0113 | 0.0059 | -3.27 | -3.49 |
| R4_arcs_fast.w_k7 | -0.0793 ± 0.0967 | -0.0103 | -0.0037 | 0.71 | 0.78 |
| R4_arcs_fast.w_k9 | 0.2211 ± 0.1046 | 0.0399 | 0.0178 | -1.7 | -1.9 |
| R4_arcs_fast.w_k13 | -0.0439 ± 0.0536 | -0.0027 | -0.0013 | 0.77 | 0.79 |
| R4_arcs_fast.w_k1 | 0.0397 ± 0.0212 | 0.0036 | 0.0014 | -1.68 | -1.77 |
| R4_arcs_fast.w_k15 | 0.0934 ± 0.0892 | 0.0036 | 0.0010 | -1.0 | -1.03 |
| R4_arcs_fast.w_k17 | -0.0967 ± 0.0200 | -0.0135 | -0.0038 | 3.75 | 4.18 |
| R4_arcs_fast.v_k21 | 0.1906 ± 0.0118 | 0.2255 | 0.2246 | 2.66 | 2.59 |
| R4_arcs_fast.v_k9 | 0.1595 ± 0.0193 | 0.1512 | 0.1584 | -0.42 | -0.06 |
| R4_arcs_fast.v_k11 | -0.1947 ± 0.0050 | -0.1690 | -0.1569 | 3.33 | 4.91 |
| R4_arcs_fast.v_k15 | -0.2422 ± 0.0280 | -0.2556 | -0.2576 | -0.46 | -0.53 |
| R4_arcs_fast.v_k3 | -0.2592 ± 0.0328 | -0.2559 | -0.2565 | 0.1 | 0.08 |
| R4_arcs_fast.v_k7 | -0.1824 ± 0.0071 | -0.1835 | -0.1955 | -0.12 | -1.45 |
| R4_arcs_fast.w_k3 | -0.0437 ± 0.0411 | -0.0029 | -0.0017 | 0.99 | 1.01 |
| R4_arcs_fast.w_k19 | 0.0664 ± 0.0540 | 0.0119 | 0.0059 | -1.0 | -1.11 |
| R4_arcs_fast.w_k21 | -0.2053 ± 0.0658 | -0.0545 | -0.0262 | 2.19 | 2.6 |
| R4_arcs_fast.v_k17 | 0.1937 ± 0.0050 | 0.1979 | 0.2069 | 0.55 | 1.72 |
| R4_arcs_fast.v_k19 | -0.1864 ± 0.0061 | -0.1978 | -0.2006 | -1.39 | -1.72 |
| R5_nav_pattern.w_k4 | -0.2114 ± 0.0558 | -0.0401 | -0.0234 | 2.87 | 3.15 |
| R5_nav_pattern.v_k6 | -0.1458 ± 0.0098 | -0.1179 | -0.1557 | 2.6 | -0.93 |
| R5_nav_pattern.v_k1 | 0.0058 ± 0.0085 | -0.0149 | -0.0158 | -2.43 | -2.54 |
| R5_nav_pattern.w_k7 | 0.0655 ± 0.0326 | -0.0000 | 0.0004 | -1.97 | -1.96 |
| R5_nav_pattern.w_k6 | 0.1098 ± 0.0382 | 0.0263 | 0.0177 | -2.1 | -2.32 |
| R5_nav_pattern.w_k9 | -0.3961 ± 0.0309 | -0.9545 | -0.2872 | -11.11 | 2.17 |
| R5_nav_pattern.w_k1 | 0.4403 ± 0.1250 | 0.9355 | 0.3929 | 3.74 | -0.36 |
| R5_nav_pattern.v_k9 | -0.0042 ± 0.0050 | 0.0139 | 0.0329 | 3.62 | 7.42 |
| R5_nav_pattern.v_k2 | 0.1928 ± 0.0114 | 0.1849 | 0.2013 | -0.61 | 0.66 |
| R5_nav_pattern.w_k8 | -0.1030 ± 0.0519 | -0.0212 | -0.0084 | 1.55 | 1.79 |
| R5_nav_pattern.v_k4 | 0.1312 ± 0.0256 | 0.1600 | 0.1613 | 1.11 | 1.16 |
| R5_nav_pattern.v_k3 | 0.2375 ± 0.0123 | 0.2579 | 0.2582 | 1.43 | 1.45 |
| R5_nav_pattern.v_k7 | -0.2568 ± 0.0112 | -0.2577 | -0.2575 | -0.06 | -0.05 |
| R5_nav_pattern.w_k3 | 0.0069 ± 0.0255 | 0.0004 | 0.0002 | -0.25 | -0.26 |
| R5_nav_pattern.v_k8 | -0.1969 ± 0.0127 | -0.1809 | -0.1901 | 1.14 | 0.49 |
| R5_nav_pattern.w_k2 | 0.1104 ± 0.0573 | 0.0196 | 0.0186 | -1.56 | -1.57 |

Path errors are start-aligned and averaged over the real repeats. `<test>/paths_map.png|mp4`: real (green) vs baseline sim (red) vs calibrated sim (cyan) on the overhead camera view.
