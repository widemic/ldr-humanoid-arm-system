#!/usr/bin/env python3
"""
YOLO Model Download Utility

Downloads and verifies YOLOv4, YOLOv4-tiny, and YOLOv8 model files.
Can be run standalone or imported as a module.
"""

import os
import sys
import yaml
import urllib.request
from typing import Dict, Tuple
from ament_index_python.packages import get_package_share_directory


class YOLOModelDownloader:
    """Handles downloading and verification of YOLO model files (v4 and v8)."""

    def __init__(self, config_path: str = None, logger=None):
        """
        Initialize the downloader.

        Args:
            config_path: Path to configuration file (optional, auto-detected if None)
            logger: ROS logger instance (optional, uses print if None)
        """
        self.logger = logger

        if config_path is None:
            base_path = os.path.join(get_package_share_directory('arm_perception'), 'config')
            config_path = os.path.join(base_path, 'object_recognition.yaml')

        self.config_path = config_path
        self.base_path = os.path.dirname(config_path)
        self.config = self.load_config()

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

    def load_config(self) -> Dict:
        """Load configuration from YAML file."""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f'Configuration file not found: {self.config_path}')

        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.log_info(f'Configuration loaded from {self.config_path}')
            return config
        except Exception as e:
            self.log_error(f'Failed to load config: {e}')
            raise

    def download_file(self, url: str, dest_path: str, description: str = "file") -> bool:
        """
        Download a file from URL to destination path.

        Args:
            url: Source URL
            dest_path: Destination file path
            description: Human-readable description for logging

        Returns:
            True if successful, False otherwise
        """
        try:
            self.log_info(f"Downloading {description}...")
            self.log_info(f"  URL: {url}")
            self.log_info(f"  Destination: {dest_path}")

            urllib.request.urlretrieve(url, dest_path)

            if os.path.exists(dest_path):
                file_size = os.path.getsize(dest_path)
                self.log_info(f"✓ {description} downloaded ({file_size:,} bytes)")
                return True
            else:
                self.log_error(f"✗ {description} download failed - file not created")
                return False

        except Exception as e:
            self.log_error(f"✗ Failed to download {description}: {e}")
            return False

    def check_and_download_file(self, file_key: str, url: str, description: str) -> bool:
        """
        Check if file exists, download if missing.

        Args:
            file_key: Filename from config
            url: Download URL
            description: Human-readable description

        Returns:
            True if file exists or was successfully downloaded
        """
        file_path = os.path.join(self.base_path, file_key)

        if os.path.exists(file_path):
            file_size = os.path.getsize(file_path)
            self.log_info(f"✓ {description} exists ({file_size:,} bytes)")
            return True
        else:
            self.log_info(f"✗ {description} not found, downloading...")
            return self.download_file(url, file_path, description)

    def ensure_yolov4_files(self) -> Tuple[bool, bool, bool]:
        """
        Check and download YOLOv4 files (weights, config, names).

        Returns:
            Tuple of (weights_ok, config_ok, names_ok)
        """
        cfg = self.config['yolov4']

        self.log_info("Checking YOLOv4 model files...")

        weights_ok = self.check_and_download_file(
            cfg['files']['weights'],
            cfg['download_urls']['weights'],
            f"YOLOv4 weights ({cfg.get('file_sizes', {}).get('weights', 'unknown size')})"
        )

        config_ok = self.check_and_download_file(
            cfg['files']['config'],
            cfg['download_urls']['config'],
            f"YOLOv4 config ({cfg.get('file_sizes', {}).get('config', 'unknown size')})"
        )

        names_ok = self.check_and_download_file(
            cfg['files']['names'],
            cfg['download_urls']['names'],
            f"COCO names ({cfg.get('file_sizes', {}).get('names', 'unknown size')})"
        )

        return weights_ok, config_ok, names_ok

    def ensure_yolov4_tiny_files(self) -> Tuple[bool, bool, bool]:
        """
        Check and download YOLOv4-tiny files (weights, config, names).

        Returns:
            Tuple of (weights_ok, config_ok, names_ok)
        """
        cfg = self.config['yolov4_tiny']

        self.log_info("Checking YOLOv4-tiny model files...")

        weights_ok = self.check_and_download_file(
            cfg['files']['weights'],
            cfg['download_urls']['weights'],
            f"YOLOv4-tiny weights ({cfg.get('file_sizes', {}).get('weights', 'unknown size')})"
        )

        config_ok = self.check_and_download_file(
            cfg['files']['config'],
            cfg['download_urls']['config'],
            f"YOLOv4-tiny config ({cfg.get('file_sizes', {}).get('config', 'unknown size')})"
        )

        names_ok = self.check_and_download_file(
            cfg['files']['names'],
            cfg['download_urls']['names'],
            f"COCO names ({cfg.get('file_sizes', {}).get('names', 'unknown size')})"
        )

        return weights_ok, config_ok, names_ok

    def ensure_yolov8_model(self, model_name: str = 'yolov8n') -> bool:
        """
        Check and download YOLOv8 model.

        Args:
            model_name: Model variant (yolov8n, yolov8s, yolov8m, yolov8l, yolov8x)

        Returns:
            True if model is available
        """
        model_file = f"{model_name}.pt"
        model_path = os.path.join(self.base_path, model_file)

        if os.path.exists(model_path):
            file_size = os.path.getsize(model_path)
            self.log_info(f"✓ {model_name} exists ({file_size:,} bytes)")
            return True

        # YOLOv8 will auto-download when loaded by ultralytics
        self.log_info(f"✗ {model_name} not found")
        self.log_info(f"  Will be auto-downloaded to {model_path} on first use")
        return False

    def ensure_all_files(self, include_tiny: bool = True, include_yolov8: bool = True) -> bool:
        """
        Ensure all required YOLO model files are present.

        Args:
            include_tiny: Also check/download YOLOv4-tiny files
            include_yolov8: Also check YOLOv8 model

        Returns:
            True if all files are available
        """
        self.log_info("=" * 60)
        self.log_info("YOLO Model File Verification and Download")
        self.log_info("=" * 60)

        # Check YOLOv4
        if self.config.get('yolov4', {}).get('enabled', True):
            weights_ok, config_ok, names_ok = self.ensure_yolov4_files()
            yolov4_ready = weights_ok and config_ok and names_ok

            if yolov4_ready:
                self.log_info("✓ YOLOv4 model files complete")
            else:
                self.log_error("✗ YOLOv4 model files incomplete")
        else:
            self.log_info("YOLOv4 disabled in config")
            yolov4_ready = False

        # Check YOLOv4-tiny
        if include_tiny and self.config.get('yolov4_tiny', {}).get('enabled', True):
            weights_ok, config_ok, names_ok = self.ensure_yolov4_tiny_files()
            tiny_ready = weights_ok and config_ok and names_ok

            if tiny_ready:
                self.log_info("✓ YOLOv4-tiny model files complete")
            else:
                self.log_error("✗ YOLOv4-tiny model files incomplete")
        else:
            self.log_info("YOLOv4-tiny disabled in config")
            tiny_ready = False

        # Check YOLOv8
        yolov8_ready = False
        if include_yolov8 and self.config.get('yolov8', {}).get('enabled', True):
            model_name = self.config.get('yolov8', {}).get('model_name', 'yolov8n')
            yolov8_ready = self.ensure_yolov8_model(model_name)

            if yolov8_ready:
                self.log_info(f"✓ {model_name} model ready")
        else:
            if not include_yolov8:
                self.log_info("YOLOv8 check skipped")

        self.log_info("=" * 60)

        # Return True if at least one model is ready
        return yolov4_ready or tiny_ready or yolov8_ready


def main():
    """Standalone execution - download all YOLO models."""
    print("YOLO Model Download Utility")
    print("=" * 60)

    try:
        downloader = YOLOModelDownloader()
        success = downloader.ensure_all_files(include_tiny=True)

        if success:
            print("\n✓ Model files ready!")
            return 0
        else:
            print("\n✗ Failed to download all required files")
            return 1

    except Exception as e:
        print(f"\n✗ Error: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
