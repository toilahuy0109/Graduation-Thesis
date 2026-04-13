import cv2
import numpy as np
import yaml
from typing import Optional, Tuple

class CameraManager:
    def __init__(self, config_path: str = 'configs/camera_config.yaml'):
        self.config = self.load_config(config_path)
        self.cap = None
        self.camera_matrix = None
        self.dist_coeffs = None

    
    def load_config(self, config_path):
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    
    