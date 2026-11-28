"""Export standalone unloaded-fit adapter for the V4.5 female head."""
import argparse
from modular_arm.v45.clamp_adapter import export

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="out/v45_clamp_adapter")
    args = p.parse_args()
    part = export(args.output)
    print("CAD valid:", part.isValid())
    print("Unloaded fit prototype only. Clamp retention unverified.")
