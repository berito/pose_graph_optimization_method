#!/usr/bin/env python3
# Correlated (grouped) outlier generator for robust PGO.
# Forked from the Vertigo generator (ipc/scripts/generateDataset.py).
# Injects mutually-consistent outlier groups (DC-GM, Lajoie 2019, sec. V):
# a run of consecutive source poses spuriously matched to a run of consecutive
# target poses, all sharing ONE world-frame misalignment D so the group is
# internally consistent but globally wrong. Emits a .groups oracle sidecar and
# a .gt_labels file. mode=random reproduces classic Vertigo singletons (control).
# 2D / SE(2) only.

import argparse
import math
import random


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(p, q):
    c, s = math.cos(p[2]), math.sin(p[2])
    return (p[0] + c * q[0] - s * q[1],
            p[1] + s * q[0] + c * q[1],
            wrap(p[2] + q[2]))


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return (-(c * p[0] + s * p[1]),
            (s * p[0] - c * p[1]),
            -p[2])


def read_g2o(path):
    vertices, edges = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("VERTEX_SE2"):
                vertices.append(line)
            elif line.startswith("EDGE_SE2"):
                edges.append(line)
    return vertices, edges


def read_gt(path):
    poses = []
    with open(path) as f:
        for line in f:
            t = line.split()
            if len(t) >= 3:
                poses.append((float(t[0]), float(t[1]), float(t[2])))
    return poses


def count_loops(edges):
    n = 0
    for e in edges:
        t = e.split()
        if int(t[2]) != int(t[1]) + 1:
            n += 1
    return n


def edge_str(a, b, z, info):
    return ("EDGE_SE2 %d %d %.6f %.6f %.6f %s\n"
            % (a, b, z[0], z[1], z[2], info))


def pick_anchors(n_poses, L, min_sep, max_sep, rng, tries=400):
    for _ in range(tries):
        i0 = rng.randint(0, n_poses - L)
        j0 = rng.randint(0, n_poses - L)
        d = abs(i0 - j0)
        if d < min_sep or d > max_sep:
            continue
        # disjoint runs
        if i0 + L <= j0 or j0 + L <= i0:
            return i0, j0
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="infile", required=True, help="clean graph.g2o")
    ap.add_argument("--gt", required=True, help="GT.txt (one pose per line: x y theta)")
    ap.add_argument("--out", required=True, help="output spoiled .g2o")
    ap.add_argument("--mode", choices=["grouped", "random"], default="grouped")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--num-outliers", type=int, help="number of outlier loop edges")
    g.add_argument("--outlier-frac", type=float,
                   help="outlier fraction of the final loop set, e.g. 0.5")
    ap.add_argument("--group-len", type=int, default=5, help="edges per group (grouped mode)")
    ap.add_argument("--min-sep", type=int, default=50,
                    help="min |i0-j0| anchor separation (a real false place)")
    ap.add_argument("--max-sep", type=int, default=200,
                    help="max |i0-j0| anchor separation (bounds subgraph span / runtime)")
    ap.add_argument("--sigma-t", type=float, default=0.1, help="inlier translation noise [m]")
    ap.add_argument("--sigma-r", type=float, default=0.01, help="inlier rotation noise [rad]")
    ap.add_argument("--disp-trans", type=float, default=5.0,
                    help="sigma of group misalignment D translation [m]")
    ap.add_argument("--disp-rot", type=float, default=30.0,
                    help="sigma of group misalignment D rotation [deg]")
    ap.add_argument("--information", default=None,
                    help="override info matrix 'I11,I12,I13,I22,I23,I33' (default from sigma)")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed (deterministic spoiled data)")
    args = ap.parse_args()

    rng = random.Random(args.seed)

    vertices, edges = read_g2o(args.infile)
    gt = read_gt(args.gt)
    n_poses = len(vertices)
    if len(gt) < n_poses:
        raise SystemExit("GT has fewer poses (%d) than graph (%d)" % (len(gt), n_poses))

    n_true_loops = count_loops(edges)
    if args.outlier_frac is not None:
        f = args.outlier_frac
        n_out = int(round(f / (1.0 - f) * n_true_loops))
    else:
        n_out = args.num_outliers

    if args.information:
        info = args.information.replace(",", " ")
    else:
        it = 1.0 / (args.sigma_t ** 2)
        ir = 1.0 / (args.sigma_r ** 2)
        info = "%.6f 0 0 %.6f 0 %.6f" % (it, it, ir)

    disp_r = math.radians(args.disp_rot)

    injected = []          # (a, b, z)
    groups = []            # list of [(a,b), ...]

    if args.mode == "random":
        # Classic Vertigo: each outlier is an independent random false loop (singleton).
        made = 0
        while made < n_out:
            a = rng.randint(0, n_poses - 1)
            b = rng.randint(0, n_poses - 1)
            d = abs(a - b)
            if a == b or d < args.min_sep or d > args.max_sep:
                continue
            if a > b:
                a, b = b, a
            z = (rng.gauss(0, 0.3), rng.gauss(0, 0.3), rng.gauss(0, math.radians(10)))
            injected.append((a, b, z))
            groups.append([(a, b)])
            made += 1
    else:
        L = args.group_len
        made = 0
        while made < n_out:
            length = min(L, n_out - made)
            anc = pick_anchors(n_poses, length, args.min_sep, args.max_sep, rng)
            if anc is None:
                raise SystemExit("could not place a group; lower --min-sep or --group-len")
            i0, j0 = anc
            # one shared world-frame misalignment for the whole group
            D = (rng.gauss(0, args.disp_trans),
                 rng.gauss(0, args.disp_trans),
                 wrap(rng.gauss(0, disp_r)))
            grp = []
            for k in range(length):
                a, b = i0 + k, j0 + k
                # z = x_a^-1 . D . x_b  (+ inlier noise)
                z = compose(inverse(gt[a]), compose(D, gt[b]))
                z = (z[0] + rng.gauss(0, args.sigma_t),
                     z[1] + rng.gauss(0, args.sigma_t),
                     wrap(z[2] + rng.gauss(0, args.sigma_r)))
                lo, hi = (a, b) if a < b else (b, a)
                injected.append((lo, hi, z))
                grp.append((lo, hi))
            groups.append(grp)
            made += length

    with open(args.out, "w") as f:
        for v in vertices:
            f.write(v)
        for e in edges:
            f.write(e)
        for a, b, z in injected:
            f.write(edge_str(a, b, z, info))

    base = args.out.rsplit(".", 1)[0]
    with open(base + ".groups", "w") as f:
        for grp in groups:
            f.write(" ".join("%d %d" % (a, b) for a, b in grp) + "\n")

    with open(base + ".gt_labels", "w") as f:
        f.write("# a b is_outlier\n")
        for a, b, _ in injected:
            f.write("%d %d 0\n" % (a, b))

    print("poses=%d true_loops=%d injected=%d groups=%d -> %s"
          % (n_poses, n_true_loops, len(injected), len(groups), args.out))
    print("sidecars: %s.groups  %s.gt_labels" % (base, base))


if __name__ == "__main__":
    main()
