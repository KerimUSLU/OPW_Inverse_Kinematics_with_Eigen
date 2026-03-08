# OPW Inverse Kinematics with Eigen

A small C++ demo implementing the analytical OPW inverse kinematics solution (Brandstötter et al., 2014) for 6‑DOF serial manipulators with an orthogonal wrist.

This repository is created for **CSE 685 Robot Control Theory** homework and includes:

- ✅ **Forward kinematics** (FK) for OPW robots
- ✅ **Analytical inverse kinematics** (IK) returning up to 8 solutions
- ✅ **Round-trip validation (FK → IK → FK)** to prove correctness

### 🔍 Round‑trip validation (FK → IK → FK)

For each test case:

1. **Compute FK** from a known joint set `q_in` to get a target pose `(u, R0e)`.
2. **Solve IK** for that target pose, producing up to 8 candidate joint solutions.
3. **Recompute FK** for each IK candidate and compare the resulting position to the original target.

The demo prints the position error (`err=... m`) for each candidate, so you can verify that each IK solution correctly reconstructs the intended pose.

---

## 🧩 Build & Run

Dependencies:
- A C++17 compiler (e.g., `g++`)
- Eigen headers (included under `eigen-3.4.0/`)

```bash
make        # build (produces ./opw_ik)
make run    # build + execute
make clean  # remove binary
```

---

## 📐 OPW Parameterization (robot model)

The robot is modeled using the OPW parameter set (Table I in Brandstötter et al.):

- `a1` : shoulder offset (x direction)
- `a2` : elbow offset
- `b`  : wrist offset (y direction)
- `c1` : base height
- `c2` : shoulder-to-elbow length
- `c3` : elbow-to-wrist length
- `c4` : wrist-to-end-effector length

These parameters are stored in `OPWParameters` and drive *both* FK and IK.

---

## 🔁 Algorithm (step-by-step)

### 1) Forward Kinematics (FK)
Given joint angles \(q = [\theta_1, \dots, \theta_6]\):

1. Apply per-joint offsets and sign conventions.
2. Compute intermediate quantities (e.g., \(c_{x1}, c_{z1}\)) using OPW equations (Equations 2–7 in the paper).
3. Compute the base-to-wrist (C) rotation matrix \(R_{0c}\) and the wrist rotation \(R_{ce}\).
4. Combine them to get the end-effector rotation \(R_{0e} = R_{0c} R_{ce}\).
5. Compute end-effector position: \(u = C + c_4 \, R_{0e}[:,2]\).

This is implemented in `forward_kinematics(const OPWParameters&, const std::array<double,6>&)`.

### 2) Inverse Kinematics (IK)
Given a target pose \((u, R_{0e})\):

1. **Wrist center (C) computation**
   - \( C = u - c_4 \, R_{0e}[:,2] \)
2. **Compute \(|c_{x1}|\)**
   - \( |c_{x1}| = \sqrt{c_{x0}^2 + c_{y0}^2 - b^2} \)
   - If the term under the sqrt is negative → pose unreachable.
3. **Solve for the first three joints (position solution)**
   - There are **two shoulder configurations** (front/back): \(c_{x1} = \\pm |c_{x1}|\).
   - For each shoulder configuration, there are **two elbow configurations** (elbow up/down).
   - This yields **4 position solutions**.
4. **Wrist orientation (last three joints)**
   - For each position solution, compute \(R_{0c}\) and then \(R_{ce} = R_{0c}^T R_{0e}\).
   - Extract wrist angles from \(R_{ce}\) using stable `atan2` formulas (Equations 17+).
   - Each position solution produces **2 wrist solutions** (due to ± nature of \(\theta_5\)).
5. **Total solutions**
   - Max: **4 position solutions × 2 wrist solutions = 8 IK solutions**.

This is implemented in `inverse_kinematics(const OPWParameters&, const Pose&)`.

> ⚠️ The solver handles wrist singularities (\(\theta_5 ≈ 0\)) by generating both wrist branches and still producing valid configurations.

---

## ✅ Demo output (sample)
The demo runs a series of test robot models and prints every IK solution along with the FK reconstruction error.

Example output (truncated):

```
╔══════════════════════════════════════════════════════════╗
║     OPW Inverse Kinematics — Brandstötter et al. 2014   ║
╚══════════════════════════════════════════════════════════╝

════════════════════════════════════════════
  KUKA KR6 R700 sixx
════════════════════════════════════════════
  Girdi q [°]: [30.0, -45.0, 60.0, 15.0, -30.0, 90.0]

  Hedef Pose (FK)
  Rotasyon:
-0.7 -0.7 -0.1
 0.7 -0.7 -0.2
 0.1 -0.3  1.0
  Pozisyon: [-0.1, -0.1, 1.1]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]  -150.00,  -20.88,   64.08,  164.96,   29.91,  116.18  | err=1.8e-10 m
  [2]  -150.00,  -20.88,   64.08,  -15.04,  -29.91,  -63.82  | err=1.8e-10 m
  [3]  -150.00,   42.59,  -53.12,   17.36,   25.70,  -92.67  | err=1.8e-10 m
  [4]  -150.00,   42.59,  -53.12, -162.64,  -25.70,   87.33  | err=1.8e-10 m
  [5]    30.00,  -45.00,   60.00, -165.00,   30.00,  -90.00  | err=1.8e-10 m
  [6]    30.00,  -45.00,   60.00,   15.00,  -30.00,   90.00  | err=1.8e-10 m
  [7]    30.00,   14.00,  -49.05,  -20.09,   22.13,  121.78  | err=1.8e-10 m
  [8]    30.00,   14.00,  -49.05,  159.91,  -22.13,  -58.22  | err=1.8e-10 m
  ---------------------------------------------------------------------------
  Geçerli: 8/8  |  En iyi pos hatası: 1.8e-10 m

════════════════════════════════════════════
  ABB IRB 2400/10 — Home pozisyonu
════════════════════════════════════════════
  Girdi q [°]: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  Hedef Pose (FK)
  Rotasyon:
1.0 0.0 0.0
0.0 1.0 0.0
0.0 0.0 1.0
  Pozisyon: [-0.0, 0.0, 2.2]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]   180.00,   -9.70,   23.86,  180.00,   14.16,    0.00  | err=2.8e-10 m
  [2]   180.00,   -9.70,   23.86,    0.00,  -14.16, -180.00  | err=2.8e-10 m
  [3]   180.00,    4.60,   -3.59,  180.00,    1.02,    0.00  | err=2.8e-10 m
  [4]   180.00,    4.60,   -3.59,    0.00,   -1.02, -180.00  | err=2.8e-10 m
  [5]     0.00,  -10.57,   20.28,  180.00,    9.71,  180.00  | err=3.7e-10 m
  [6]     0.00,  -10.57,   20.28,    0.00,   -9.71,    0.00  | err=3.7e-10 m
  [7]     0.00,    0.00,   -0.00,    0.00,    0.00,   -0.00  | err=3.7e-10 m
  [8]     0.00,    0.00,   -0.00,  180.00,   -0.00, -180.00  | err=3.7e-10 m
  ---------------------------------------------------------------------------
  Geçerli: 8/8  |  En iyi pos hatası: 2.8e-10 m

════════════════════════════════════════════
  Stäubli TX40 (yanal ofset b=35mm)
════════════════════════════════════════════
  Girdi q [°]: [-20.0, 30.0, 10.0, 45.0, -60.0, 15.0]

  Hedef Pose (FK)
  Rotasyon:
 0.8 -0.5 -0.3
 0.3  0.8 -0.5
 0.5  0.3  0.8
  Pozisyon: [0.2, -0.1, 0.7]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]   -20.00,   30.00,   10.00, -135.00,   60.00, -165.00  | err=1.3e-09 m
  [2]   -20.00,   30.00,   10.00,   45.00,  -60.00,   15.00  | err=1.3e-09 m
  [3]   -20.00,   40.00,  -10.00, -130.13,   53.22, -173.82  | err=1.3e-09 m
  [4]   -20.00,   40.00,  -10.00,   49.87,  -53.22,    6.18  | err=1.3e-09 m
  [5]   175.50,  -40.00,   10.00,   40.12,   58.70,  179.58  | err=1.3e-09 m
  [6]   175.50,  -40.00,   10.00, -139.88,  -58.70,   -0.42  | err=1.3e-09 m
  [7]   175.50,  -30.00,  -10.00,   36.89,   66.53, -173.41  | err=1.3e-09 m
  [8]   175.50,  -30.00,  -10.00, -143.11,  -66.53,    6.59  | err=1.3e-09 m
  ---------------------------------------------------------------------------
  Geçerli: 8/8  |  En iyi pos hatası: 1.3e-09 m

════════════════════════════════════════════
  Fanuc R-2000iB/200R
════════════════════════════════════════════
  Girdi q [°]: [45.0, -30.0, 15.0, -90.0, 60.0, 0.0]

  Hedef Pose (FK)
  Rotasyon:
 0.5  0.7  0.5
-0.2  0.7 -0.7
-0.8  0.3  0.5
  Pozisyon: [-0.1, -0.4, 2.8]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]  -135.00,  -38.04,   61.74,   94.99,   60.38,  -10.02  | err=5.4e-11 m
  [2]  -135.00,  -38.04,   61.74,  -85.01,  -60.38,  169.98  | err=5.4e-11 m
  [3]  -135.00,   18.99,  -41.80,   70.51,   66.73,   41.86  | err=5.4e-11 m
  [4]  -135.00,   18.99,  -41.80, -109.49,  -66.73, -138.14  | err=5.4e-11 m
  [5]    45.00,  -30.00,   15.00,  -90.00,   60.00,   -0.00  | err=4.4e-10 m
  [6]    45.00,  -30.00,   15.00,   90.00,  -60.00,  180.00  | err=4.4e-10 m
  [7]    45.00,  -24.49,    4.94,  -87.38,   60.10,   -5.26  | err=4.4e-10 m
  [8]    45.00,  -24.49,    4.94,   92.62,  -60.10,  174.74  | err=4.4e-10 m
  ---------------------------------------------------------------------------
  Geçerli: 8/8  |  En iyi pos hatası: 5.4e-11 m

════════════════════════════════════════════
  Unimation PUMA 560
════════════════════════════════════════════
  Girdi q [°]: [0.0, 45.0, -90.0, 0.0, 45.0, 0.0]

  Hedef Pose (FK)
  Rotasyon:
 1.0 -0.0  0.0
 0.0  1.0  0.0
 0.0  0.0  1.0
  Pozisyon: [-0.0, 0.1, 1.3]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]    11.69,  -45.00,   95.37,  180.00,   50.37,  168.31  | err=1.1e-10 m
  [2]    11.69,  -45.00,   95.37,    0.00,  -50.37,  -11.69  | err=1.1e-10 m
  [3]    11.69,   47.93,  -90.00,    0.00,   42.07,  -11.69  | err=1.1e-10 m
  [4]    11.69,   47.93,  -90.00,  180.00,  -42.07,  168.31  | err=1.1e-10 m
  [5]     0.00,  -47.93,   95.37,  180.00,   47.44,  180.00  | err=1.1e-10 m
  [6]     0.00,  -47.93,   95.37,    0.00,  -47.44,    0.00  | err=1.1e-10 m
  [7]     0.00,   45.00,  -90.00,    0.00,   45.00,   -0.00  | err=1.1e-10 m
  [8]     0.00,   45.00,  -90.00,  180.00,  -45.00, -180.00  | err=1.1e-10 m
  ---------------------------------------------------------------------------
  Geçerli: 8/8  |  En iyi pos hatası: 1.1e-10 m

════════════════════════════════════════════
  KUKA KR6 — Bilek singülaritesi (θ5≈0)
════════════════════════════════════════════
  Girdi q [°]: [45.0, -30.0, 20.0, 10.0, 0.0, 5.0]

  Hedef Pose (FK)
  Rotasyon:
 0.5 -0.9 -0.1
 0.9  0.5 -0.1
 0.2 -0.0  1.0
  Pozisyon: [-0.2, -0.2, 1.1]

  IK Çözümleri (° cinsinden):
  ---------------------------------------------------------------------------
  [1]  -135.00,    2.14,   35.21, -180.00,   27.35,   15.00  | err=2.9e-10 m
  [2]  -135.00,    2.14,   35.21,    0.00,  -27.35, -165.00  | err=2.9e-10 m
  [3]  -135.00,   34.18,  -24.26,   -0.13,    0.08, -164.87  | err=2.9e-10 m
  [4]  -135.00,   34.18,  -24.26,  179.87,   -0.08,   15.13  | err=2.9e-10 m
  [5]    45.00,  -30.00,   20.00,   10.00,    0.00,    5.00  | err=5.5e-10 m
  [6]    45.00,  -30.00,   20.00, -170.00,   -0.00, -175.00  | err=5.5e-10 m
  [7]    45.00,  -14.37,   -9.05,    0.00,   13.42,   15.00  | err=5.5e-10 m
  [8]    45.00,  -14.37,   -9.05, -180.00,  -13.42, -165.00  | err=5.5e-10 m
  ---------------------------------------------------------------------------
```

Each section prints the target pose computed by FK, then all reachable IK solutions, and finally the best position error when feeding each IK solution back through FK.

---

## 📌 Notes for CSE 685

- This implementation is **analytic** (no numerical root finding).
- It demonstrates the classic OPW closed-form IK method and its ability to produce multiple solutions.
- The sample output shows that all 8 solutions are valid for typical reachable poses, and the FK re-projection error is on the order of 1e-10 meters.
