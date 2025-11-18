#!/usr/bin/env python3
"""
YOLOv8 Model Download Utility

Downloads and verifies YOLOv8 model files using ultralytics API.
Models are automatically downloaded on first use.
"""

import os
import sys
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YOLOv8ModelDownloader:
    """Handles downloading and verification of YOLOv8 model files."""

    def __init__(self, logger=None):
        """
        Initialize the downloader.

        Args:
            logger: ROS logger instance (optional, uses print if None)
        """
        self.logger = logger
        self.base_path = os.path.join(get_package_share_directory('arm_perception'), 'config')

        # Available YOLOv8 models (nano, small, medium, large, xlarge)
        self.models = {
            'yolov8n': 'yolov8n.pt',  # Fastest, least accurate (6.3MB)
            'yolov8s': 'yolov8s.pt',  # Small, balanced (22MB)
            'yolov8m': 'yolov8m.pt',  # Medium (52MB)
            'yolov8l': 'yolov8l.pt',  # Large (87MB)
            'yolov8x': 'yolov8x.pt',  # Extra large, most accurate (136MB)
        }

    def log_info(self, msg: str):
        """Log info message."""
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def log_error(self, msg: str):
        """Log error message."""
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}", file=sys.stderr)

    def download_model(self, model_name: str = 'yolov8n') -> bool:
        """
        Download YOLOv8 model.

        Args:
            model_name: Model variant (yolov8n, yolov8s, yolov8m, yolov8l, yolov8x)

        Returns:
            True if successful, False otherwise
        """
        if model_name not in self.models:
            self.log_error(f"Unknown model: {model_name}")
            self.log_error(f"Available models: {', '.join(self.models.keys())}")
            return False

        model_file = self.models[model_name]
        model_path = os.path.join(self.base_path, model_file)

        try:
            # Check if model already exists
            if os.path.exists(model_path):
                file_size = os.path.getsize(model_path)
                self.log_info(f"✓ {model_name} exists ({file_size:,} bytes)")
                return True

            self.log_info(f"Downloading {model_name}...")
            self.log_info(f"  Destination: {model_path}")

            # Download using ultralytics API (auto-downloads to cache)
            model = YOLO(model_file)

            # Find the downloaded model in ultralytics cache
            import torch
            from pathlib import Path

            # Ultralytics caches models in ~/.cache/ultralytics or torch hub
            cache_dir = Path.home() / '.cache' / 'ultralytics'
            cached_model = None

            if cache_dir.exists():
                for cached_file in cache_dir.rglob(model_file):
                    cached_model = cached_file
                    break

            # Also check torch hub
            if not cached_model:
                torch_hub = Path(torch.hub.get_dir())
                for cached_file in torch_hub.rglob(model_file):
                    cached_model = cached_file
                    break

            # Copy to config directory for easy access
            if cached_model and cached_model.exists():
                import shutil
                shutil.copy(cached_model, model_path)
                file_size = os.path.getsize(model_path)
                self.log_info(f"✓ {model_name} downloaded ({file_size:,} bytes)")
                return True
            else:
                self.log_info(f"✓ {model_name} cached by ultralytics (ready to use)")
                return True

        except Exception as e:
            self.log_error(f"✗ Failed to download {model_name}: {e}")
            return False

    def ensure_model(self, model_name: str = 'yolov8n') -> bool:
        """
        Ensure YOLOv8 model is available.

        Args:
            model_name: Model variant to download

        Returns:
            True if model is available
        """
        self.log_info("=" * 60)
        self.log_info("YOLOv8 Model Verification and Download")
        self.log_info("=" * 60)

        success = self.download_model(model_name)

        self.log_info("=" * 60)

        if success:
            self.log_info(f"✓ {model_name} ready!")
        else:
            self.log_error(f"✗ Failed to prepare {model_name}")

        return success


def main():
    """Standalone execution - download YOLOv8 model."""
    print("YOLOv8 Model Download Utility")
    print("=" * 60)

    # Default to yolov8n (fastest, good for testing)
    model_name = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n'

    try:
        downloader = YOLOv8ModelDownloader()
        success = downloader.ensure_model(model_name)

        if success:
            print(f"\n✓ {model_name} ready!")
            return 0
        else:
            print(f"\n✗ Failed to download {model_name}")
            return 1

    except Exception as e:
        print(f"\n✗ Error: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
