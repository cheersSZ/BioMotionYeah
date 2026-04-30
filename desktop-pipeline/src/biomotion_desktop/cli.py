"""argparse-based CLI: `python -m biomotion_desktop.cli {run,compare,plot}`.

Three subcommands map 1:1 to the three pipeline stages users care about:

  run      Trial config       -> 5 desktop ground-truth files
  compare  desktop + iOS dirs -> compare.json (three-tier metrics)
  plot     desktop + iOS dirs -> per-channel PNG overlays
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .trial import load_trial
from .pipeline import run_pipeline
from .compare import asymmetry_report_for_export, compare_folders, write_report
from .ios_export import autodetect_stem


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="biomotion-desktop",
        description=(
            "OpenSim desktop ground-truth pipeline + iOS-export comparator "
            "for the BioMotion app."
        ),
    )
    sub = p.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="Run IK -> ID -> SO on a trial config.")
    run.add_argument("--config", required=True, type=Path,
                     help="Path to trial YAML.")
    run.add_argument("--output", required=True, type=Path,
                     help="Output directory (will be created).")
    run.add_argument("--skip-so", action="store_true",
                     help="Skip Static Optimization (IK + ID only).")

    cmp_ = sub.add_parser("compare",
                          help="Diff a desktop output folder against an iOS export.")
    cmp_.add_argument("--desktop", required=True, type=Path,
                      help="Desktop pipeline output folder.")
    cmp_.add_argument("--ios", required=True, type=Path,
                      help="iOS export folder (OfflineExport-*).")
    cmp_.add_argument("--stem", default=None,
                      help="Desktop-side filename stem; autodetected if omitted.")
    cmp_.add_argument("--ios-stem", default=None,
                      help="iOS-side filename stem (defaults to --stem); "
                           "the iOS app uses a per-run UUID, not the trial name.")
    cmp_.add_argument("--report", type=Path, default=None,
                      help="Output JSON path (default: <desktop>/compare.json).")
    cmp_.add_argument("--tiers", default="1,2,3",
                      help="Comma-separated tiers to compute (default: 1,2,3).")
    # Bilateral-asymmetry add-on. Off by default so existing CI/diff
    # consumers see byte-identical stdout. When on, appends a per-pair L/R
    # peak / RMS / asymmetry-index table for the iOS activations and exits
    # non-zero when any pair exceeds the threshold (CI gate).
    cmp_.add_argument("--asymmetry-report", action="store_true",
                      help="Append a bilateral L/R muscle asymmetry table "
                           "for the iOS activations file.")
    cmp_.add_argument("--asymmetry-threshold-pct", type=float, default=10.0,
                      help="Pair asymmetry index threshold in percent (default: 10.0). "
                           "Only used when --asymmetry-report is set.")

    plt_ = sub.add_parser("plot",
                          help="Render per-channel comparison PNGs.")
    plt_.add_argument("--desktop", required=True, type=Path)
    plt_.add_argument("--ios", required=True, type=Path)
    plt_.add_argument("--stem", default=None)
    plt_.add_argument("--ios-stem", default=None,
                      help="iOS-side filename stem (defaults to --stem).")
    plt_.add_argument("--out", type=Path, default=None,
                      help="Output directory (default: <desktop>/plots).")
    plt_.add_argument("--max-per-file", type=int, default=None,
                      help="Limit plots per file (handy when there are 80 muscles).")

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "run":
        return _cmd_run(args)
    if args.command == "compare":
        return _cmd_compare(args)
    if args.command == "plot":
        return _cmd_plot(args)
    return 2  # unreachable; argparse enforces required=True


def _cmd_run(args) -> int:
    trial = load_trial(args.config)
    print(f"[run] trial '{trial.name}' from {args.config}")
    print(f"      osim: {trial.osim}")
    print(f"      trc:  {trial.trc}")
    print(f"      out:  {args.output.resolve()}")
    result = run_pipeline(trial, args.output, skip_so=args.skip_so)
    print("[run] timings (s):", json.dumps(result.timings_s, indent=2))
    print("[run] outputs:")
    for p in (
        result.kinematics_mot,
        result.inverse_dynamics_sto,
        result.activations_sto,
        result.muscle_forces_sto,
        result.summary_json,
    ):
        if p is not None:
            print(f"      {p}")
    return 0


def _cmd_compare(args) -> int:
    stem = args.stem or autodetect_stem(args.desktop)
    ios_stem = args.ios_stem or autodetect_stem(args.ios)
    tiers = tuple(int(x) for x in args.tiers.split(",") if x.strip())
    report = compare_folders(
        args.desktop, args.ios, stem, ios_stem=ios_stem, tiers=tiers
    )
    out_path = args.report or (Path(args.desktop) / "compare.json")
    write_report(report, out_path)
    print(f"[compare] desktop_stem='{stem}'  ios_stem='{ios_stem}'  tiers={tiers}")
    print(f"[compare] {len(report.files)} file(s) compared")
    for key, fc in report.files.items():
        ov = fc.overall
        print(
            f"          {key:<28} channels={ov.get('channel_count', 0)}  "
            f"rmse_mean={ov.get('rmse_mean', float('nan')):.4f}  "
            f"r2_mean={ov.get('r2_mean', float('nan')):.4f}"
        )
    if report.files_missing.get("desktop_only") or report.files_missing.get("ios_only"):
        print(f"[compare] missing files: {report.files_missing}")
    print(f"[compare] report: {out_path.resolve()}")

    # Opt-in bilateral-asymmetry diagnostic. Appended after the existing
    # tier output so the legacy stdout prefix stays byte-identical when the
    # flag is absent. Returns non-zero only when --asymmetry-report is set
    # AND a pair exceeds the threshold — pure-diagnostic runs never fail
    # the build accidentally.
    if args.asymmetry_report:
        text, passed = asymmetry_report_for_export(
            args.ios, ios_stem, threshold_pct=args.asymmetry_threshold_pct
        )
        print()
        print(text)
        return 0 if passed else 1
    return 0


def _cmd_plot(args) -> int:
    from .plot import plot_comparison

    stem = args.stem or autodetect_stem(args.desktop)
    ios_stem = args.ios_stem or autodetect_stem(args.ios)
    out_dir = args.out or (Path(args.desktop) / "plots")
    written = plot_comparison(
        args.desktop, args.ios, stem, out_dir,
        ios_stem=ios_stem, max_per_file=args.max_per_file,
    )
    print(
        f"[plot] desktop_stem='{stem}'  ios_stem='{ios_stem}'  "
        f"wrote {len(written)} file(s) to {out_dir.resolve()}"
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
