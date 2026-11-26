from pathlib import Path
import cadquery as cq
from modular_arm.components.experimental_ring import make_experimental_ring
from modular_arm.config.ring import ExperimentalRingConfig


def export_ring(directory: str | Path, config: ExperimentalRingConfig | None = None) -> tuple[Path, Path]:
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    solid = make_experimental_ring(config)
    step = destination / "experimental_ring_v0.step"
    stl = destination / "experimental_ring_v0.stl"
    cq.exporters.export(solid, str(step), exportType="STEP")
    cq.exporters.export(solid, str(stl), exportType="STL")
    return step, stl
