#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import numpy as np


def default_kinematics_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(package_root))
    from my_ur_control.ur_kinematics import find_kinematics_file
    return find_kinematics_file("ur3")


def import_model():
    package_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(package_root))
    from my_ur_control.ur_kinematics import URKinematicsModel
    return URKinematicsModel


def vee(skew: np.ndarray) -> np.ndarray:
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)


def numerical_jacobian(model, q: np.ndarray, eps: float) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    T0 = model.forward_kinematics(q)
    R0 = T0[:3, :3]
    J = np.zeros((6, 6), dtype=float)

    for i in range(6):
        qp, qm = q.copy(), q.copy()
        qp[i] += eps
        qm[i] -= eps
        Tp = model.forward_kinematics(qp)
        Tm = model.forward_kinematics(qm)

        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)

        Rdot = (Tp[:3, :3] - Tm[:3, :3]) / (2 * eps)
        omega_hat = Rdot @ R0.T
        omega_hat = 0.5 * (omega_hat - omega_hat.T)
        J[3:, i] = vee(omega_hat)

    return J


def fixed_cases():
    return [
        [0.0, -1.57, 0.65, -1.57, -1.57, 0.0],
        [0.0, -2.08, 0.65, -1.57, -1.57, 0.0],
        [0.613139, -0.367991, -0.717982, -1.226637, -0.735982, 0.188496],
        [0.245327, -1.471964, -0.418823, -1.226637, 0.0, 0.0],
        [0.98, -0.367, -0.598, -1.47, 0.98, 0.25],
        [-1.1, -1.22, -1.07, -1.47, 0.98, 0.25],
    ]


def main():
    ap = argparse.ArgumentParser(description="Validate UR3 analytical Jacobian against numerical FK derivative.")
    ap.add_argument("--kinematics", type=Path, default=default_kinematics_path())
    ap.add_argument("--eps", type=float, default=1e-6)
    ap.add_argument("--eps-sweep", action="store_true")
    ap.add_argument("--random", type=int, default=10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--abs-tolerance", type=float, default=1e-7)
    ap.add_argument("--show-matrix-on-failure", action="store_true")
    args = ap.parse_args()

    if args.random < 0 or args.eps <= 0 or args.abs_tolerance <= 0:
        ap.error("random >= 0, eps > 0 and abs-tolerance > 0 are required")
    if not args.kinematics.is_file():
        print(f"ERROR: kinematics file not found: {args.kinematics}")
        return 2

    URKinematicsModel = import_model()
    model = URKinematicsModel.from_yaml(args.kinematics)
    cases = [np.asarray(q, dtype=float) for q in fixed_cases()]
    rng = np.random.default_rng(args.seed)
    cases += [rng.uniform(-2.0, 2.0, 6) for _ in range(args.random)]

    eps_values = [args.eps] if not args.eps_sweep else [1e-4, 1e-5, 1e-6, 1e-7]
    overall = True

    print("=" * 72)
    print("UR3 ANALYTICAL VS NUMERICAL JACOBIAN")
    print(f"Calibration: {args.kinematics}")
    print(f"Hash: {model.calibration_hash}")
    print("=" * 72)

    for eps in eps_values:
        print(f"\nEPS={eps:.1e}")
        worst = (-1.0, None)
        for i, q in enumerate(cases):
            Ja = model.compute_jacobian(q)
            Jn = numerical_jacobian(model, q, eps)
            E = Ja - Jn
            max_abs = float(np.max(np.abs(E)))
            fro = float(np.linalg.norm(E, ord="fro"))
            rel = fro / max(float(np.linalg.norm(Ja, ord="fro")), 1e-15)
            lin = float(np.max(np.abs(E[:3])))
            ang = float(np.max(np.abs(E[3:])))
            ok = max_abs <= args.abs_tolerance
            overall &= ok
            if max_abs > worst[0]:
                worst = (max_abs, (i, q.copy(), rel))
            print(f"case {i:02d}: max_abs={max_abs:.3e}, rel_fro={rel:.3e}, linear={lin:.3e}, angular={ang:.3e} -> {'PASS' if ok else 'FAIL'}")
            if not ok and args.show_matrix_on_failure:
                print("Analytical:\n", Ja)
                print("Numerical:\n", Jn)
                print("Error:\n", E)
        print(f"Worst case: {worst[1][0]:02d}, max_abs={worst[0]:.3e}, rel_fro={worst[1][2]:.3e}")

    print("\n" + "=" * 72)
    print("RESULT:", "PASS" if overall else "FAIL")
    print("This validates J against the current FK implementation; it does not prove YAML/URDF/TF equivalence.")
    print("=" * 72)
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
