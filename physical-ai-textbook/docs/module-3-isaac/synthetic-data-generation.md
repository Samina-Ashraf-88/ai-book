---
sidebar_position: 3
title: Synthetic Data Generation with Isaac Sim
---

# Synthetic Data Generation with Isaac Sim

## Overview

Synthetic data generation is a crucial component of modern robotics development, allowing for the creation of large datasets for training AI models without the need for physical hardware or manual data collection. Isaac Sim provides powerful tools for generating realistic synthetic data.

## Learning Objectives

- Understand the importance of synthetic data in robotics
- Learn to configure Isaac Sim for data generation
- Create diverse and realistic synthetic datasets
- Implement domain randomization techniques
- Transfer models from synthetic to real data (Sim-to-Real)

## Why Synthetic Data?

Synthetic data generation addresses several challenges in robotics:

- **Data scarcity**: Physical data collection is time-consuming and expensive
- **Safety**: Training in simulation avoids risks to robots and humans
- **Variety**: Easy to generate diverse scenarios and conditions
- **Annotations**: Perfect ground truth annotations are available
- **Repeatability**: Same scenarios can be reproduced exactly

## Isaac Sim Data Generation Capabilities

Isaac Sim provides several tools for synthetic data generation:

- **Photorealistic rendering**: Using RTX technology for realistic visuals
- **Sensor simulation**: Accurate simulation of cameras, LiDAR, IMU, etc.
- **Physics simulation**: Realistic object interactions and dynamics
- **Domain randomization**: Techniques to improve sim-to-real transfer
- **Large-scale environments**: Support for complex scenes with many objects

## Setting Up Isaac Sim for Data Generation

### Basic Scene Configuration

```python
# Example Isaac Sim data generation setup
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np

class SyntheticDataGenerator:
    def __init__(self):
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)

        # Initialize synthetic data helper
        self.sd_helper = SyntheticDataHelper()

        # Load robot and environment
        self.setup_scene()

    def setup_scene(self):
        # Add ground plane
        self.world.scene.add_ground_plane("/World/defaultGroundPlane")

        # Add robot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/Robot",
                name="my_robot",
                usd_path="/path/to/robot.usd"
            )
        )

        # Add objects for data generation
        self.add_objects_for_data()

    def add_objects_for_data(self):
        # Add various objects with random poses for data generation
        for i in range(10):
            object_path = f"/World/Object_{i}"
            # Add random object from asset library
            # This would be specific to your use case

    def generate_dataset(self, num_samples=1000):
        # Generate synthetic dataset
        for i in range(num_samples):
            # Randomize scene
            self.randomize_scene()

            # Step simulation
            self.world.step(render=True)

            # Capture data
            data = self.capture_data()

            # Save data
            self.save_data(data, i)

    def randomize_scene(self):
        # Randomize object poses, lighting, textures, etc.
        pass

    def capture_data(self):
        # Capture camera images, LiDAR data, etc.
        camera_data = self.sd_helper.get_rgb_camera_data()
        depth_data = self.sd_helper.get_depth_data()
        segmentation_data = self.sd_helper.get_segmentation_data()
        return {
            'rgb': camera_data,
            'depth': depth_data,
            'segmentation': segmentation_data
        }

    def save_data(self, data, index):
        # Save data to disk with proper annotations
        import cv2
        cv2.imwrite(f'data/rgb_{index:06d}.png', data['rgb'])
        cv2.imwrite(f'data/depth_{index:06d}.png', data['depth'])
        cv2.imwrite(f'data/seg_{index:06d}.png', data['segmentation'])
```

## Domain Randomization

Domain randomization is a technique to improve sim-to-real transfer by randomizing various aspects of the simulation:

### Visual Domain Randomization

```python
# Example domain randomization implementation
import random
from pxr import Gf, UsdLux, UsdGeom

class DomainRandomizer:
    def __init__(self, stage):
        self.stage = stage
        self.light_prim = self.stage.GetPrimAtPath("/World/Light")
        self.material_prims = []

    def randomize_lighting(self):
        # Randomize light intensity, color, and position
        light_intensity = random.uniform(500, 1500)
        light_color = Gf.Vec3f(
            random.uniform(0.8, 1.0),
            random.uniform(0.8, 1.0),
            random.uniform(0.8, 1.0)
        )

        # Apply to light prim
        if self.light_prim.IsValid():
            light_api = UsdLux.DistantLightAPI(self.light_prim)
            light_api.GetIntensityAttr().Set(light_intensity)
            light_api.GetColorAttr().Set(light_color)

    def randomize_materials(self):
        # Randomize material properties like color, roughness, etc.
        for prim in self.material_prims:
            # Randomize diffuse color
            diffuse_color = (random.random(), random.random(), random.random())
            # Apply to material

    def randomize_textures(self):
        # Randomize textures and surface properties
        # This would involve changing USD material properties
        pass

    def randomize_camera_noise(self):
        # Simulate camera noise and artifacts
        noise_params = {
            'gaussian_noise': random.uniform(0.0, 0.05),
            'poisson_noise': random.uniform(0.0, 0.02),
            'motion_blur': random.uniform(0.0, 0.1)
        }
        return noise_params

    def apply_randomization(self):
        self.randomize_lighting()
        self.randomize_materials()
        self.randomize_textures()
```

### Physical Domain Randomization

```python
# Physical domain randomization
class PhysicalRandomizer:
    def __init__(self, world):
        self.world = world

    def randomize_physics(self):
        # Randomize physical properties
        physics_params = {
            'gravity': random.uniform(-9.9, -9.7),
            'friction': random.uniform(0.3, 0.9),
            'restitution': random.uniform(0.0, 0.5)
        }

        # Apply to physics scene
        # This would be specific to the physics engine used

    def randomize_robot_dynamics(self):
        # Randomize robot dynamics parameters
        # Mass, inertia, joint friction, etc.
        robot_params = {
            'mass_variance': random.uniform(0.95, 1.05),
            'friction_variance': random.uniform(0.8, 1.2),
            'actuator_noise': random.uniform(0.0, 0.02)
        }

        return robot_params

    def randomize_sensor_noise(self):
        # Randomize sensor noise parameters
        sensor_params = {
            'camera_noise': random.uniform(0.0, 0.05),
            'imu_noise': {
                'acceleration': random.uniform(0.01, 0.1),
                'gyro': random.uniform(0.001, 0.01)
            },
            'lidar_noise': random.uniform(0.01, 0.05)
        }

        return sensor_params
```

## Synthetic Data Types

### RGB Images

Generating realistic RGB images:

```python
# RGB image generation with annotations
def generate_rgb_dataset(self, num_samples=1000):
    dataset = []
    for i in range(num_samples):
        # Randomize scene
        self.domain_randomizer.apply_randomization()

        # Step simulation to settle objects
        for _ in range(10):
            self.world.step(render=True)

        # Capture RGB image
        rgb_image = self.sd_helper.get_rgb_camera_data()

        # Get segmentation for object annotations
        seg_data = self.sd_helper.get_segmentation_data()

        # Generate bounding boxes from segmentation
        bboxes = self.generate_bounding_boxes(seg_data)

        # Save image and annotations
        sample = {
            'image': rgb_image,
            'annotations': bboxes,
            'metadata': {
                'scene_id': i,
                'domain_params': self.domain_randomizer.get_current_params()
            }
        }

        dataset.append(sample)
        self.save_sample(sample, i)

    return dataset
```

### Depth Data

Generating depth maps:

```python
# Depth data generation
def generate_depth_dataset(self, num_samples=1000):
    dataset = []
    for i in range(num_samples):
        # Randomize scene
        self.domain_randomizer.apply_randomization()

        # Step simulation
        self.world.step(render=True)

        # Capture depth data
        depth_data = self.sd_helper.get_depth_data()

        # Convert to point cloud if needed
        point_cloud = self.depth_to_pointcloud(depth_data)

        # Save depth data
        sample = {
            'depth': depth_data,
            'pointcloud': point_cloud,
            'metadata': {
                'scene_id': i,
                'camera_pose': self.get_camera_pose()
            }
        }

        dataset.append(sample)
        self.save_depth_sample(sample, i)

    return dataset

def depth_to_pointcloud(self, depth_data):
    # Convert depth image to point cloud
    height, width = depth_data.shape
    # Calculate point cloud using camera intrinsics
    # This would use the camera's intrinsic parameters
    return point_cloud
```

### Segmentation Data

Generating semantic and instance segmentation:

```python
# Segmentation data generation
def generate_segmentation_dataset(self, num_samples=1000):
    dataset = []
    for i in range(num_samples):
        # Randomize scene
        self.domain_randomizer.apply_randomization()

        # Step simulation
        self.world.step(render=True)

        # Capture segmentation data
        seg_data = self.sd_helper.get_segmentation_data()

        # Process segmentation into semantic and instance masks
        semantic_mask, instance_mask = self.process_segmentation(seg_data)

        # Save segmentation data
        sample = {
            'semantic_mask': semantic_mask,
            'instance_mask': instance_mask,
            'metadata': {
                'scene_id': i,
                'object_ids': self.get_object_ids()
            }
        }

        dataset.append(sample)
        self.save_segmentation_sample(sample, i)

    return dataset

def process_segmentation(self, raw_seg_data):
    # Process raw segmentation data into semantic and instance masks
    # Raw data might contain object IDs, class IDs, etc.
    semantic_mask = self.map_to_semantic_classes(raw_seg_data)
    instance_mask = self.map_to_instance_ids(raw_seg_data)

    return semantic_mask, instance_mask
```

## Advanced Data Generation Techniques

### Multi-Sensor Fusion

Generating data from multiple sensors simultaneously:

```python
# Multi-sensor data generation
def generate_multisensor_data(self, num_samples=1000):
    dataset = []
    for i in range(num_samples):
        # Randomize scene
        self.domain_randomizer.apply_randomization()

        # Step simulation
        self.world.step(render=True)

        # Capture data from all sensors
        sensor_data = {
            'camera_rgb': self.sd_helper.get_rgb_camera_data(),
            'camera_depth': self.sd_helper.get_depth_data(),
            'lidar': self.get_lidar_data(),
            'imu': self.get_imu_data(),
            'force_torque': self.get_force_torque_data()
        }

        # Synchronize data timestamps
        synced_data = self.synchronize_sensors(sensor_data)

        # Save multi-sensor sample
        sample = {
            'sensors': synced_data,
            'metadata': {
                'scene_id': i,
                'timestamp': self.world.current_time
            }
        }

        dataset.append(sample)
        self.save_multisensor_sample(sample, i)

    return dataset
```

### Temporal Data Generation

Generating temporal sequences for video or time-series analysis:

```python
# Temporal data generation
def generate_temporal_data(self, sequence_length=30, num_sequences=100):
    dataset = []
    for seq_id in range(num_sequences):
        # Randomize initial scene
        self.domain_randomizer.apply_randomization()

        sequence = []
        for frame in range(sequence_length):
            # Step simulation
            self.world.step(render=True)

            # Capture frame data
            frame_data = {
                'rgb': self.sd_helper.get_rgb_camera_data(),
                'depth': self.sd_helper.get_depth_data(),
                'timestamp': self.world.current_time
            }

            sequence.append(frame_data)

            # Apply small randomization for natural variation
            if frame > 0:  # Don't randomize the first frame
                self.apply_subtle_randomization()

        # Save sequence
        sequence_sample = {
            'frames': sequence,
            'metadata': {
                'sequence_id': seq_id,
                'length': sequence_length
            }
        }

        dataset.append(sequence_sample)
        self.save_temporal_sequence(sequence_sample, seq_id)

    return dataset

def apply_subtle_randomization(self):
    # Apply small randomization to simulate natural variation
    # without completely changing the scene
    pass
```

## Data Quality and Validation

### Quality Metrics

```python
# Data quality validation
class DataQualityValidator:
    def __init__(self):
        self.metrics = {}

    def validate_rgb_data(self, rgb_image):
        # Check for common issues in RGB images
        if rgb_image is None:
            return False, "Image is None"

        if rgb_image.shape[0] < 100 or rgb_image.shape[1] < 100:
            return False, "Image too small"

        # Check for overexposure
        overexposed_pixels = np.sum(rgb_image > 250) / rgb_image.size
        if overexposed_pixels > 0.1:  # More than 10% overexposed
            return False, f"Too many overexposed pixels: {overexposed_pixels:.2%}"

        # Check for underexposure
        underexposed_pixels = np.sum(rgb_image < 10) / rgb_image.size
        if underexposed_pixels > 0.5:  # More than 50% underexposed
            return False, f"Too many underexposed pixels: {underexposed_pixels:.2%}"

        return True, "Valid"

    def validate_depth_data(self, depth_data):
        # Check depth data validity
        valid_pixels = np.isfinite(depth_data)
        valid_ratio = np.sum(valid_pixels) / depth_data.size

        if valid_ratio < 0.5:  # Less than 50% valid pixels
            return False, f"Too many invalid depth values: {1-valid_ratio:.2%}"

        # Check depth range
        min_depth = np.min(depth_data[valid_pixels])
        max_depth = np.max(depth_data[valid_pixels])

        if min_depth < 0.01 or max_depth > 100.0:  # Unreasonable range
            return False, f"Depth range unreasonable: {min_depth:.2f} to {max_depth:.2f}"

        return True, "Valid"

    def validate_annotations(self, annotations, image_shape):
        # Validate bounding box annotations
        for bbox in annotations:
            x, y, w, h = bbox
            if x < 0 or y < 0 or x+w > image_shape[1] or y+h > image_shape[0]:
                return False, f"Bounding box out of bounds: {bbox}"

        return True, "Valid"
```

## Sim-to-Real Transfer

### Domain Adaptation Techniques

```python
# Techniques for improving sim-to-real transfer
class DomainAdaptation:
    def __init__(self):
        self.sim_model = None
        self.real_model = None

    def train_with_domain_randomization(self, model, num_epochs=100):
        # Train model with domain randomization
        for epoch in range(num_epochs):
            # Generate batch of randomized synthetic data
            synthetic_batch = self.generate_randomized_batch(batch_size=32)

            # Train on synthetic data
            model.train_on_batch(synthetic_batch)

            if epoch % 10 == 0:
                # Validate on small real dataset if available
                real_batch = self.get_real_batch(batch_size=8)
                validation_score = model.validate_on_batch(real_batch)
                print(f"Epoch {epoch}, Real validation: {validation_score}")

    def test_time_adaptation(self, model, real_image):
        # Adapt model at test time using real data statistics
        real_stats = self.compute_image_statistics(real_image)

        # Adjust model based on real statistics
        adapted_model = self.adapt_model_to_domain(model, real_stats)

        return adapted_model

    def compute_image_statistics(self, image):
        # Compute statistics like mean, std, color distribution
        stats = {
            'mean': np.mean(image, axis=(0, 1)),
            'std': np.std(image, axis=(0, 1)),
            'color_dist': self.compute_color_distribution(image)
        }
        return stats

    def adapt_model_to_domain(self, model, target_stats):
        # Adapt model using techniques like batch normalization adaptation
        adapted_model = model
        # Apply domain adaptation techniques
        return adapted_model
```

## Exercise

Create a synthetic data generation pipeline that:

1. Sets up a simple environment in Isaac Sim
2. Implements domain randomization for lighting and textures
3. Generates RGB and depth data with annotations
4. Validates the quality of generated data
5. Demonstrates sim-to-real transfer techniques

### Implementation Outline

1. Create a basic scene with objects
2. Implement domain randomization
3. Generate a dataset of images with annotations
4. Validate data quality
5. Train a simple model on synthetic data
6. Test on real data (simulated) to evaluate transfer

This comprehensive approach to synthetic data generation with Isaac Sim enables the creation of large, diverse datasets for training AI models for robotics applications.