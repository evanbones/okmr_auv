# okmr_object_detection

The okmr_object_detection package provides real-time object segmentation for the AUV using ONNX Runtime inference.

## Purpose

This package serves as the perception layer in the AUV's autonomy stack, processing camera feeds to identify and segment underwater objects. It supports multiple segmentation models and can dynamically switch between them based on mission requirements.

## How to Use

### Prerequisites
- Synchronized RGB and depth camera feeds on `/rgb` and `/depth` topics
- Pre-trained ONNX segmentation models in the `models/` directory  
- Parameter files for model-specific configurations

### Running
```bash
ros2 run okmr_object_detection onnx_segmentation_detector
ros2 launch okmr_object_detection full_object_detection_system.launch.py
```

### Services
- `change_model`: Switch between ONNX models at runtime
- `set_inference_camera`: Control inference mode (disabled/front/bottom camera)

## How It Works

The `onnx_segmentation_detector.py` implements the core functionality:

1. **Input Processing**: Synchronizes RGB/depth feeds, applies depth masking for distant objects, and preprocesses images (resize/pad/normalize)

2. **ONNX Inference**: Supports two model architectures (3-output and 2-output) with GPU acceleration via CUDA/TensorRT providers

3. **Post-processing**: Filters detections by confidence, applies top-k selection, decodes segmentation masks using prototype masks, and combines multiple objects into a unified output

4. **Output**: Publishes segmentation masks to `/mask` topic with class IDs per pixel (0=background, 1+=objects)

The `detector.py` base class handles camera synchronization and ROS2 integration patterns for all detector implementations.

## Dependencies

- `onnxruntime`: Model inference engine
- `okmr_msgs`: Custom service definitions (`ChangeModel`, `SetInferenceCamera`)
- Standard ROS2/OpenCV packages for image processing and messaging

**Data Flow**:
1. **okmr_hardware_interface** → Camera feeds → **okmr_object_detection**
2. **okmr_object_detection** → Segmentation masks → **okmr_navigation**, **okmr_mapping** 
3. **okmr_automated_planner** → Model switching commands → **okmr_object_detection**
