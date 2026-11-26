"""Open in CQ-editor. Project root must be the current folder."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / "src"))
from modular_arm.components.experimental_ring import make_experimental_ring
from modular_arm.config.ring import ExperimentalRingConfig

ring = make_experimental_ring(ExperimentalRingConfig())
show_object(ring, name="experimental_ring_24t_not_functional")
