# RayTracer

A GPU-accelerated path tracer built with OpenGL, featuring physically-based rendering and acceleration structures for real-time ray tracing.

This project is an upgraded version of MATH 1B research project [raytracing](https://github.com/Joecheong2006/raytracing), developed in collaboration with professor Doli Bambhania.

## Features

### Rendering Capabilities
- **Physically-based materials**: Support for diffuse, metallic, and transmissive materials
- **Global illumination**: Path tracing with multiple light bounces
- **BVH acceleration**: Static Bounding Volume Hierarchy for fast ray-triangle intersection
- **Complex geometry**: Support for high-poly models (dragon, beetle, etc.)

### Technical Improvements
- **Broad OpenGL compatibility**: Works with OpenGL 3.3+ 
  - Uses fragment shaders instead of compute shaders
  - Uses Texture Buffer Objects (TBO) instead of Shader Storage Buffer Objects (SSBO)
  - Previous version required OpenGL 4.3+
- **Cross-platform build system**: CMake support for Windows, macOS, and Linux
- **Static BVH**: Spatial acceleration structure for efficient ray tracing

## Getting Started

### Prerequisites
- C++ compiler with C++11 support
- CMake 3.10 or higher
- OpenGL 3.3+ compatible graphics driver

### Clone
```bash
git clone --recursive https://github.com/Joecheong2006/RayTracer.git
cd RayTracer
```

### Build & Run
```bash
mkdir build
cd build
cmake ..
make
./raytracer
```

For Windows (Visual Studio):
```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
.\Release\raytracer.exe
```

## Gallery

## Prism Test1
![Prism Test1](screenshots/prism_test1.png)

## Roadmap

- [x] Static BVH acceleration structure
- [x] Transmissive materials (glass, water)
- [x] Texture mapping and UV coordinates
- [ ] Next Event Estimation (NEE) for direct light sampling
- [ ] Denoising filters
- [ ] Dynamic BVH for animated scenes
- [ ] Importance sampling for complex materials

## Technical Details

The ray tracer uses:
- **Fragment shaders** for parallel ray generation and intersection (OpenGL 3.3+ compatible)
- **Texture Buffer Objects (TBO)** for storing geometry and BVH data (alternative to SSBO)
- **BVH traversal** for O(log n) intersection tests
- **Monte Carlo integration** for global illumination
- **Progressive rendering** with accumulation buffer

## Acknowledgments

This project was developed as a MATH 1B research project under the guidance of professor Doli Bambhania, exploring the intersection of mathematics and computer graphics through real-time ray tracing techniques.
