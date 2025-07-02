from abc import ABC, abstractmethod
import numpy as np

class ILeg(ABC):
    @abstractmethod
    def fk(self, angles: np.ndarray) -> np.ndarray:
        pass
    
    @abstractmethod
    def ik(self, target: np.ndarray) -> np.ndarray:
        pass