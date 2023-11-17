import eagerx
import numpy as np
from eagerx.core.specs import EngineStateSpec
from typing import Any


class DummyState(eagerx.EngineState):
    @classmethod
    def make(cls):
        return cls.get_specification()

    def initialize(self, spec: EngineStateSpec, simulator: Any):
        pass

    def reset(self, state: np.ndarray):
        pass
