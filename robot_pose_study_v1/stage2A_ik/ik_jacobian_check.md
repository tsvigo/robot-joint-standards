# IK Jacobian finite-difference check

Central difference, h = 1e-6 rad. Position rows: m/rad; orientation rows: rad/rad.
Conditioning uses the scaled Jacobian: position rows divided by L_norm = 0.5 m.

| side | config | mode | max abs error | relative Frobenius | pass |
|---|---|---|---:|---:|---|
| L | P0 | 6d | 4.212e-10 | 2.772e-10 | yes |
| L | P0 | 5d | 4.212e-10 | 3.093e-10 | yes |
| L | palmdown0 | 6d | 7.181e-10 | 4.698e-10 | yes |
| L | palmdown0 | 5d | 7.181e-10 | 5.506e-10 | yes |
| L | forward | 6d | 8.101e-10 | 3.983e-10 | yes |
| L | forward | 5d | 8.101e-10 | 4.683e-10 | yes |
| L | forward_bent | 6d | 1.073e-09 | 6.124e-10 | yes |
| L | forward_bent | 5d | 1.073e-09 | 6.798e-10 | yes |
| L | el90 | 6d | 5.914e-10 | 3.383e-10 | yes |
| L | el90 | 5d | 5.914e-10 | 3.845e-10 | yes |
| L | folded | 6d | 8.569e-10 | 6.142e-10 | yes |
| L | folded | 5d | 8.569e-10 | 7.699e-10 | yes |
| L | random | 6d | 1.152e-09 | 7.266e-10 | yes |
| L | random | 5d | 1.152e-09 | 8.587e-10 | yes |
| R | P0 | 6d | 7.670e-10 | 3.147e-10 | yes |
| R | P0 | 5d | 7.670e-10 | 3.570e-10 | yes |
| R | palmdown0 | 6d | 5.300e-10 | 3.832e-10 | yes |
| R | palmdown0 | 5d | 5.300e-10 | 4.440e-10 | yes |
| R | forward | 6d | 7.137e-10 | 4.433e-10 | yes |
| R | forward | 5d | 7.137e-10 | 5.062e-10 | yes |
| R | forward_bent | 6d | 6.723e-10 | 4.577e-10 | yes |
| R | forward_bent | 5d | 6.723e-10 | 5.126e-10 | yes |
| R | el90 | 6d | 1.895e-09 | 8.232e-10 | yes |
| R | el90 | 5d | 6.328e-10 | 5.295e-10 | yes |
| R | folded | 6d | 2.796e-09 | 2.104e-09 | yes |
| R | folded | 5d | 1.338e-09 | 8.987e-10 | yes |
| R | random | 6d | 2.151e-09 | 9.485e-10 | yes |
| R | random | 5d | 2.151e-09 | 1.131e-09 | yes |
