# Stage 2A: first bed reachability sample

This is kinematic reachability and a simplified collision check, not a safety classification.
Patient is absent; targets are bed-relative placeholders. Bed length 2.0 m is an assumption.
Collision geometry is temporary capsules/boxes, not the original collision meshes.

## Summary

- rows: 100
- reachable: 34
- collision-free in simplified model: 100
- reachable and collision-free: 34

### By approach

| approach | rows | reachable | collision-free |
|---|---:|---:|---:|
| frontal | 20 | 5 | 20 |
| lateral | 80 | 29 | 80 |

### By pose

| pose | rows | reachable | collision-free |
|---|---:|---:|---:|
| P4 | 38 | 22 | 38 |
| P5 | 38 | 0 | 38 |
| P6 | 8 | 8 | 8 |
| P7 | 8 | 2 | 8 |
| P9 | 8 | 2 | 8 |

### By mode

| mode | rows | reachable | collision-free |
|---|---:|---:|---:|
| 5d | 80 | 29 | 80 |
| 6d | 20 | 5 | 20 |

### By side

| side | rows | reachable | collision-free |
|---|---:|---:|---:|
| L | 50 | 23 | 50 |
| R | 50 | 11 | 50 |

## First rows

| approach | H | W | clearance | pose | side | mode | reachable | collision-free | pos err mm | ori err deg | min margin | cond | min clearance |
|---|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| lateral | 0.7 | 1.0 | 0.10 | P4 | L | 6d | no | yes | 55.682 | 5.857 | 0.0 | 4.81e+00 | 0.083 |
| lateral | 0.7 | 1.0 | 0.10 | P4 | L | 5d | yes | yes | 0.000 | 0.000 | 6.7 | 6.19e+00 | 0.055 |
| lateral | 0.7 | 1.0 | 0.10 | P5 | L | 6d | no | yes | 254.880 | 1.063 | 0.0 | 1.74e+03 | 0.108 |
| lateral | 0.7 | 1.0 | 0.10 | P5 | L | 5d | no | yes | 254.233 | 0.847 | 0.0 | 1.41e+03 | 0.108 |
| lateral | 0.7 | 1.0 | 0.10 | P6 | L | 6d | yes | yes | 0.000 | 0.000 | 31.2 | 7.92e+00 | 0.055 |
| lateral | 0.7 | 1.0 | 0.10 | P6 | L | 5d | yes | yes | 0.000 | 0.000 | 11.6 | 7.78e+00 | 0.055 |
| lateral | 0.7 | 1.0 | 0.10 | P7 | L | 6d | no | yes | 285.573 | 21.897 | 0.0 | 1.89e+01 | 0.088 |
| lateral | 0.7 | 1.0 | 0.10 | P7 | L | 5d | yes | yes | 0.004 | 0.000 | 1.5 | 4.33e+01 | 0.075 |
| lateral | 0.7 | 1.0 | 0.10 | P9 | L | 6d | yes | yes | 0.000 | 0.000 | 19.6 | 1.03e+01 | 0.118 |
| lateral | 0.7 | 1.0 | 0.10 | P9 | L | 5d | yes | yes | 0.000 | 0.000 | 33.8 | 8.87e+00 | 0.122 |
| lateral | 0.7 | 1.0 | 0.10 | P4 | R | 6d | no | yes | 112.891 | 4.549 | 0.0 | 5.01e+00 | 0.101 |
| lateral | 0.7 | 1.0 | 0.10 | P4 | R | 5d | no | yes | 61.078 | 2.098 | 0.0 | 4.41e+00 | 0.093 |
| lateral | 0.7 | 1.0 | 0.10 | P5 | R | 6d | no | yes | 220.125 | 0.528 | 0.7 | 3.47e+03 | 0.101 |
| lateral | 0.7 | 1.0 | 0.10 | P5 | R | 5d | no | yes | 219.995 | 0.480 | 0.7 | 4.13e+04 | 0.101 |
| lateral | 0.7 | 1.0 | 0.10 | P6 | R | 6d | yes | yes | 0.000 | 0.000 | 9.4 | 6.65e+00 | 0.048 |
| lateral | 0.7 | 1.0 | 0.10 | P6 | R | 5d | yes | yes | 0.000 | 0.000 | 10.6 | 5.16e+00 | 0.048 |
| lateral | 0.7 | 1.0 | 0.10 | P7 | R | 6d | no | yes | 129.395 | 18.458 | 0.0 | 6.20e+00 | 0.124 |
| lateral | 0.7 | 1.0 | 0.10 | P7 | R | 5d | yes | yes | 0.000 | 0.000 | 6.1 | 8.25e+00 | 0.068 |
| lateral | 0.7 | 1.0 | 0.10 | P9 | R | 6d | no | yes | 433.624 | 4.252 | 0.0 | 5.23e+00 | 0.114 |
| lateral | 0.7 | 1.0 | 0.10 | P9 | R | 5d | no | yes | 393.471 | 9.378 | 0.0 | 9.42e+00 | 0.047 |
| frontal | 0.7 | 1.0 | 0.10 | P4 | L | 6d | no | yes | 49.454 | 5.374 | 0.0 | 7.71e+00 | 0.076 |
| frontal | 0.7 | 1.0 | 0.10 | P4 | L | 5d | yes | yes | 0.000 | 0.000 | 7.2 | 6.09e+00 | 0.055 |
| frontal | 0.7 | 1.0 | 0.10 | P5 | L | 6d | no | yes | 1239.670 | 3.899 | 0.0 | 1.86e+02 | 0.108 |
| frontal | 0.7 | 1.0 | 0.10 | P5 | L | 5d | no | yes | 1239.118 | 1.691 | 0.6 | 3.07e+03 | 0.107 |
| frontal | 0.7 | 1.0 | 0.10 | P6 | L | 6d | yes | yes | 0.000 | 0.000 | 10.0 | 1.09e+01 | 0.055 |
| frontal | 0.7 | 1.0 | 0.10 | P6 | L | 5d | yes | yes | 0.000 | 0.000 | 10.0 | 7.72e+00 | 0.055 |
| frontal | 0.7 | 1.0 | 0.10 | P7 | L | 6d | no | yes | 175.840 | 0.636 | 0.0 | 1.02e+02 | 0.102 |
| frontal | 0.7 | 1.0 | 0.10 | P7 | L | 5d | no | yes | 176.204 | 1.197 | 0.0 | 1.34e+02 | 0.102 |
| frontal | 0.7 | 1.0 | 0.10 | P9 | L | 6d | no | yes | 1046.022 | 3.898 | 0.0 | 5.58e+03 | 0.105 |
| frontal | 0.7 | 1.0 | 0.10 | P9 | L | 5d | no | yes | 1045.986 | 3.771 | 0.0 | 6.09e+02 | 0.105 |
| frontal | 0.7 | 1.0 | 0.10 | P4 | R | 6d | no | yes | 101.798 | 8.445 | 0.0 | 4.85e+00 | 0.102 |
| frontal | 0.7 | 1.0 | 0.10 | P4 | R | 5d | no | yes | 62.994 | 2.012 | 0.0 | 4.30e+00 | 0.096 |
| frontal | 0.7 | 1.0 | 0.10 | P5 | R | 6d | no | yes | 1204.421 | 1.741 | 0.0 | 1.36e+03 | 0.116 |
| frontal | 0.7 | 1.0 | 0.10 | P5 | R | 5d | no | yes | 1204.193 | 0.688 | 0.1 | 2.23e+03 | 0.116 |
| frontal | 0.7 | 1.0 | 0.10 | P6 | R | 6d | yes | yes | 0.000 | 0.000 | 5.9 | 7.46e+00 | 0.048 |
| frontal | 0.7 | 1.0 | 0.10 | P6 | R | 5d | yes | yes | 0.000 | 0.000 | 8.0 | 6.19e+00 | 0.048 |
| frontal | 0.7 | 1.0 | 0.10 | P7 | R | 6d | no | yes | 149.515 | 2.685 | 0.0 | 5.56e+01 | 0.105 |
| frontal | 0.7 | 1.0 | 0.10 | P7 | R | 5d | no | yes | 145.316 | 2.148 | 0.0 | 4.33e+01 | 0.102 |
| frontal | 0.7 | 1.0 | 0.10 | P9 | R | 6d | no | yes | 1012.353 | 3.534 | 0.0 | 1.25e+02 | 0.117 |
| frontal | 0.7 | 1.0 | 0.10 | P9 | R | 5d | no | yes | 1011.377 | 3.064 | 0.0 | 1.21e+02 | 0.117 |

## Unreachable rows

Count: 66

## Reachable but simplified-collision rows

Count: 0
