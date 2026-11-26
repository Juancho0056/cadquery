"""Run at project root: python scripts/export_experimental_ring.py"""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from modular_arm.exporters.ring import export_ring
from modular_arm.validation.ring_checks import check_experimental_ring


def git_sha() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


if __name__ == "__main__":
    report = check_experimental_ring()
    report["git_sha"] = git_sha()
    out = ROOT / "artifacts"
    out.mkdir(exist_ok=True)
    (out / "experimental_ring_v0_validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if report["status"] != "PASS":
        raise SystemExit("Geometry validation failed, refusing export")
    print("Exported:", *(str(path) for path in export_ring(out)))
    print("H01/H02:", report["status"], " | H03-H10: NOT_RUN")
