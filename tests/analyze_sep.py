#!/usr/bin/env python3
"""Fourier analysis of the 16/15 cross-residual produced by encoder_sim sep.

Question: can a reference-free harmonic/INL table be built from fine_delta alone?
That needs track-16 and track-15 error content to land on disjoint mechanical
orders. This script measures where each isolated error source actually lands.

Usage:  encoder_sim.exe sep > sep.csv && python analyze_sep.py sep.csv
"""

import sys
import numpy as np

REVS = 8  # encoder_sim sep emits exactly 8 uniformly-sampled revolutions


def spectrum(theta_deg, values, revs):
    """Amplitude spectrum by mechanical order. Assumes samples arrive in theta order
    and span exactly `revs` whole revolutions -- both guaranteed by encoder_sim."""
    # Samples are uniform in theta across `revs` revolutions; fold to one rev so
    # order k of the folded series is mechanical order k.
    n = len(values) // revs * revs
    folded = values[:n].reshape(revs, -1).mean(axis=0)
    spec = np.fft.rfft(folded) / len(folded)
    amp = np.abs(spec) * 2.0
    amp[0] = np.abs(spec[0])
    return amp, folded


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "sep.csv"
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    configs = ["clean", "asym_only", "track1_only", "track2_only", "ecc_only", "all"]

    print(f"{'config':<14}{'top mechanical orders in fine_delta (order:amplitude_deg)'}")
    print("-" * 90)
    tables = {}
    for cfg in configs:
        sel = raw["config"] == cfg
        if not sel.any():
            continue
        theta = raw["theta_deg"][sel]
        fd = raw["fine_delta_deg"][sel]
        amp, folded = spectrum(theta, fd, REVS)
        tables[cfg] = amp
        top = np.argsort(amp[1:])[::-1][:6] + 1
        top = sorted(top)
        cells = "  ".join(f"{k}:{amp[k]:.3f}" for k in top if amp[k] > 1e-3)
        print(f"{cfg:<14}{cells}")

    print()
    print("energy split by order class (amplitude sum, deg):")
    print(f"{'config':<14}{'ord 1 (ecc)':>13}{'mult of 16':>13}{'mult of 15':>13}{'other':>10}")
    print("-" * 90)
    for cfg, amp in tables.items():
        n = len(amp)
        idx = np.arange(1, n)
        e_ecc = amp[1] if n > 1 else 0.0
        e16 = amp[[k for k in idx if k % 16 == 0]].sum()
        e15 = amp[[k for k in idx if k % 15 == 0]].sum()
        other = amp[[k for k in idx if k != 1 and k % 16 and k % 15]].sum()
        print(f"{cfg:<14}{e_ecc:>13.4f}{e16:>13.4f}{e15:>13.4f}{other:>10.4f}")

    print()
    if "all" in tables and "track1_only" in tables and "track2_only" in tables:
        a, t1, t2 = tables["all"], tables["track1_only"], tables["track2_only"]
        n = min(len(a), len(t1), len(t2))
        k16 = [k for k in range(1, n) if k % 16 == 0]
        k15 = [k for k in range(1, n) if k % 15 == 0]
        # Superposition check: does the combined case equal the sum of isolated ones?
        err16 = np.abs(a[k16] - t1[k16]).max() if k16 else 0.0
        err15 = np.abs(a[k15] - t2[k15]).max() if k15 else 0.0
        print("separability check (combined case vs isolated cases):")
        print(f"  track-16 orders {k16}: max amplitude discrepancy {err16:.5f} deg")
        print(f"  track-15 orders {k15}: max amplitude discrepancy {err15:.5f} deg")
        print(f"  -> orders overlap? {sorted(set(k16) & set(k15))}")


if __name__ == "__main__":
    main()
