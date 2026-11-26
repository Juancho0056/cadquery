"""Generate a local JSON inventory without extracting or modifying reference files."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from modular_arm.validation.three_mf import inventory

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    print(json.dumps(inventory(args.source), indent=2, ensure_ascii=False))
