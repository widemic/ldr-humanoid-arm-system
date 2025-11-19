#!/usr/bin/env python3
"""
Configuration Loader for Object Recognition

Handles loading and parsing of YAML configuration files,
returning structured configuration objects ready to use.
"""

import os
import yaml
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class DeviceConfig:
    """Device configuration (GPU/CPU)."""
    use_gpu: bool
    device_id: int = 0


@dataclass
class TopicsConfig:
    """ROS topics configuration."""
    camera_input: str
    processed_output: str


@dataclass
class DetectionConfig:
    """Detection parameters."""
    confidence_threshold: float
    nms_threshold: float
    min_box_size: int
    input_size: int


@dataclass
class ModelFilesConfig:
    """Model file paths and URLs."""
    enabled: bool
    weights: str
    config: str
    names: str
    weights_url: str
    config_url: str
    names_url: str


@dataclass
class ModelSelectionConfig:
    """Model selection configuration."""
    type: str = "yolov8"


@dataclass
class YOLOv8Config:
    """YOLOv8 model configuration."""
    enabled: bool
    model_name: str
    model_file: str
    imgsz: int
    conf: float
    iou: float
    max_det: int
    half: bool
    verbose: bool


@dataclass
class PerformanceConfig:
    """Performance settings."""
    fps_update_interval: float


@dataclass
class WindowConfig:
    """Window display settings."""
    show: bool
    title: str


@dataclass
class DrawingConfig:
    """OpenCV drawing parameters."""
    # Bounding box
    box_thickness: int

    # Label
    label_font: int
    label_font_scale: float
    label_font_thickness: int
    label_text_color: Tuple[int, int, int]
    label_padding: int

    # Center point
    center_point_radius: int
    center_point_filled: int  # -1 for filled, positive for outline

    # Coordinates
    coords_font_scale: float
    coords_font_thickness: int
    coords_offset_y: int

    # Info overlay
    info_font: int

    info_fps_scale: float
    info_fps_thickness: int
    info_fps_color: Tuple[int, int, int]
    info_fps_position: Tuple[int, int]

    info_count_scale: float
    info_count_thickness: int
    info_count_color: Tuple[int, int, int]
    info_count_position: Tuple[int, int]

    info_device_scale: float
    info_device_thickness: int
    info_device_color: Tuple[int, int, int]
    info_device_position: Tuple[int, int]

    info_quit_scale: float
    info_quit_thickness: int
    info_quit_color: Tuple[int, int, int]
    info_quit_offset_bottom: int


@dataclass
class VisualizationConfig:
    """Complete visualization configuration."""
    window: WindowConfig
    show_center_point: bool
    show_coordinates: bool
    show_fps: bool
    show_device_info: bool
    show_object_count: bool
    show_quit_message: bool
    drawing: DrawingConfig
    colors: List[Tuple[int, int, int]]


@dataclass
class ObjectRecognitionConfig:
    """Complete object recognition configuration."""
    device: DeviceConfig
    topics: TopicsConfig
    model: ModelSelectionConfig
    detection: DetectionConfig
    yolov8: YOLOv8Config
    yolov4: ModelFilesConfig
    yolov4_tiny: ModelFilesConfig
    performance: PerformanceConfig
    visualization: VisualizationConfig


class ConfigLoader:
    """Loads and parses configuration files."""

    def __init__(self, config_path: str, logger=None):
        """
        Initialize configuration loader.

        Args:
            config_path: Base path to config directory
            logger: Optional ROS logger for messages
        """
        self.config_path = config_path
        self.logger = logger

    def _log_info(self, msg: str):
        """Log info message."""
        if self.logger:
            self.logger.info(msg)

    def _log_error(self, msg: str):
        """Log error message."""
        if self.logger:
            self.logger.error(msg)

    def _load_yaml(self, filename: str) -> dict:
        """Load a YAML file."""
        filepath = os.path.join(self.config_path, filename)

        if not os.path.exists(filepath):
            self._log_error(f'Configuration file not found: {filepath}')
            raise FileNotFoundError(f'Configuration file not found: {filepath}')

        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            self._log_info(f'Loaded {filename}')
            return data
        except Exception as e:
            self._log_error(f'Failed to load {filename}: {e}')
            raise

    def _parse_device(self, data: dict) -> DeviceConfig:
        """Parse device configuration."""
        device = data.get('device', {})
        return DeviceConfig(
            use_gpu=device.get('use_gpu', False),
            device_id=device.get('device_id', 0)
        )

    def _parse_model_selection(self, data: dict) -> ModelSelectionConfig:
        """Parse model selection configuration."""
        model = data.get('model', {})
        return ModelSelectionConfig(
            type=model.get('type', 'yolov8')
        )

    def _parse_yolov8(self, data: dict) -> YOLOv8Config:
        """Parse YOLOv8 configuration."""
        yolov8 = data.get('yolov8', {})
        return YOLOv8Config(
            enabled=yolov8.get('enabled', True),
            model_name=yolov8.get('model_name', 'yolov8n'),
            model_file=yolov8.get('model_file', 'yolov8n.pt'),
            imgsz=yolov8.get('imgsz', 640),
            conf=yolov8.get('conf', 0.3),
            iou=yolov8.get('iou', 0.4),
            max_det=yolov8.get('max_det', 300),
            half=yolov8.get('half', True),
            verbose=yolov8.get('verbose', False)
        )

    def _parse_topics(self, data: dict) -> TopicsConfig:
        """Parse topics configuration."""
        topics = data.get('topics', {})
        return TopicsConfig(
            camera_input=topics.get('camera_input', '/camera/color/image_raw'),
            processed_output=topics.get('processed_output', '/camera/color/processed_image')
        )

    def _parse_detection(self, data: dict) -> DetectionConfig:
        """Parse detection configuration."""
        detection = data.get('detection', {})
        return DetectionConfig(
            confidence_threshold=detection.get('confidence_threshold', 0.3),
            nms_threshold=detection.get('nms_threshold', 0.4),
            min_box_size=detection.get('min_box_size', 10),
            input_size=detection.get('input_size', 416)
        )

    def _parse_model_files(self, data: dict, model_key: str) -> ModelFilesConfig:
        """Parse model files configuration."""
        model = data.get(model_key, {})
        files = model.get('files', {})
        urls = model.get('download_urls', {})

        return ModelFilesConfig(
            enabled=model.get('enabled', True),
            weights=files.get('weights', ''),
            config=files.get('config', ''),
            names=files.get('names', ''),
            weights_url=urls.get('weights', ''),
            config_url=urls.get('config', ''),
            names_url=urls.get('names', '')
        )

    def _parse_performance(self, data: dict) -> PerformanceConfig:
        """Parse performance configuration."""
        perf = data.get('performance', {})
        return PerformanceConfig(
            fps_update_interval=perf.get('fps_update_interval', 1.0)
        )

    def _parse_window(self, data: dict) -> WindowConfig:
        """Parse window configuration."""
        window = data.get('window', {})
        return WindowConfig(
            show=window.get('show', True),
            title=window.get('title', 'Object Recognition')
        )

    def _parse_drawing(self, data: dict) -> DrawingConfig:
        """Parse drawing configuration."""
        drawing = data.get('drawing', {})

        return DrawingConfig(
            box_thickness=drawing.get('box_thickness', 2),

            label_font=drawing.get('label_font', cv2.FONT_HERSHEY_SIMPLEX),
            label_font_scale=drawing.get('label_font_scale', 0.6),
            label_font_thickness=drawing.get('label_font_thickness', 2),
            label_text_color=tuple(drawing.get('label_text_color', [255, 255, 255])),
            label_padding=drawing.get('label_padding', 10),

            center_point_radius=drawing.get('center_point_radius', 4),
            center_point_filled=-1 if drawing.get('center_point_filled', True) else 1,

            coords_font_scale=drawing.get('coords_font_scale', 0.4),
            coords_font_thickness=drawing.get('coords_font_thickness', 1),
            coords_offset_y=drawing.get('coords_offset_y', 15),

            info_font=drawing.get('info_font', cv2.FONT_HERSHEY_SIMPLEX),

            info_fps_scale=drawing.get('info_fps_scale', 0.7),
            info_fps_thickness=drawing.get('info_fps_thickness', 2),
            info_fps_color=tuple(drawing.get('info_fps_color', [0, 255, 0])),
            info_fps_position=tuple(drawing.get('info_fps_position', [10, 30])),

            info_count_scale=drawing.get('info_count_scale', 0.7),
            info_count_thickness=drawing.get('info_count_thickness', 2),
            info_count_color=tuple(drawing.get('info_count_color', [0, 255, 0])),
            info_count_position=tuple(drawing.get('info_count_position', [10, 60])),

            info_device_scale=drawing.get('info_device_scale', 0.5),
            info_device_thickness=drawing.get('info_device_thickness', 1),
            info_device_color=tuple(drawing.get('info_device_color', [255, 255, 255])),
            info_device_position=tuple(drawing.get('info_device_position', [10, 90])),

            info_quit_scale=drawing.get('info_quit_scale', 0.6),
            info_quit_thickness=drawing.get('info_quit_thickness', 2),
            info_quit_color=tuple(drawing.get('info_quit_color', [255, 255, 255])),
            info_quit_offset_bottom=drawing.get('info_quit_offset_bottom', 10)
        )

    def _parse_visualization(self, data: dict) -> VisualizationConfig:
        """Parse visualization configuration."""
        return VisualizationConfig(
            window=self._parse_window(data),
            show_center_point=data.get('show_center_point', True),
            show_coordinates=data.get('show_coordinates', True),
            show_fps=data.get('show_fps', True),
            show_device_info=data.get('show_device_info', True),
            show_object_count=data.get('show_object_count', True),
            show_quit_message=data.get('show_quit_message', True),
            drawing=self._parse_drawing(data),
            colors=[tuple(c) for c in data.get('colors', [
                [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0],
                [255, 0, 255], [0, 255, 255], [255, 165, 0], [128, 0, 128],
                [255, 192, 203], [0, 128, 128], [128, 128, 0], [128, 0, 0],
                [0, 0, 128], [0, 128, 0], [200, 200, 200]
            ])]
        )

    def load(self) -> ObjectRecognitionConfig:
        """
        Load complete configuration from YAML files.

        Returns:
            ObjectRecognitionConfig with all settings
        """
        # Load main config
        main_data = self._load_yaml('object_recognition.yaml')

        # Load visualization config
        viz_data = self._load_yaml('visualization.yaml')

        # Parse and return
        return ObjectRecognitionConfig(
            device=self._parse_device(main_data),
            topics=self._parse_topics(main_data),
            model=self._parse_model_selection(main_data),
            detection=self._parse_detection(main_data),
            yolov8=self._parse_yolov8(main_data),
            yolov4=self._parse_model_files(main_data, 'yolov4'),
            yolov4_tiny=self._parse_model_files(main_data, 'yolov4_tiny'),
            performance=self._parse_performance(main_data),
            visualization=self._parse_visualization(viz_data)
        )


def load_config(config_path: str, logger=None) -> ObjectRecognitionConfig:
    """
    Convenience function to load configuration.

    Args:
        config_path: Path to config directory
        logger: Optional ROS logger

    Returns:
        ObjectRecognitionConfig object
    """
    loader = ConfigLoader(config_path, logger)
    return loader.load()
