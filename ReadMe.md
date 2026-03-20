# 🦾 IK Solver Library

Welcome to the comprehensive technical guide for the enhanced IK (Inverse Kinematics) Solver library! This is a C++17 library designed for solving 2D and 3D inverse kinematics problems with multiple algorithms, constraint systems, and mathematical utilities.

## 🏗️ **Project Overview**

This library provides robust inverse kinematics solving capabilities with:
- **Multiple Solver Algorithms**: CCD, FABRIK, and Jacobian-based methods
- **2D & 3D Support**: Full kinematic chain support for both 2D and 3D scenarios
- **Joint Constraints**: Realistic joint limitations and angle constraints
- **Enhanced Vector Math**: Comprehensive 2D/3D vector operations
- **Mathematical Utilities**: Safe math operations and numerical stability
- **SDL2 Visualization**: Real-time IK solving demonstrations

## 📂 **Project Structure**

```
IKSolver/
├── include/iksolver/           # Public API headers
│   ├── Vector2.h               # Enhanced 2D vector mathematics
│   ├── Vector3.h               # Enhanced 3D vector mathematics
│   ├── MathUtils.h             # Mathematical constants & utilities
│   ├── JointConstraints.h      # Joint limitation system
│   ├── IKChain2D.h             # 2D kinematic chain management
│   ├── IKChain3D.h             # 3D kinematic chain management
│   ├── CCDSolver.h             # 2D CCD solver
│   ├── FABRIKSolver.h          # 2D FABRIK solver
│   ├── JacobianSolver.h        # Jacobian-based solver
│   └── Solvers3D.h             # 3D CCD & FABRIK solvers
├── src/                        # Implementation files
├── examples/                   # Usage examples
├── tests/                      # Comprehensive test suite
├── docs/                       # Documentation
└── Makefile                    # Build system
```

## 🚀 **Quick Start Guide**

### Building the Library

```bash
# Build everything (library + demos + tests)
make

# Build just the library
make libiksolver.a

# Build and run tests
make test_solver.exe
./test_solver.exe

# Build SDL2 demo
make demo.exe
./demo.exe
```

### Basic 2D IK Example

```cpp
#include "iksolver/IKChain2D.h"
#include "iksolver/CCDSolver.h"

using namespace ik;

// Create a 4-joint arm
std::vector<Vector2> joints = {
    {0, 0},    // Base
    {50, 0},   // Joint 1
    {100, 0},  // Joint 2
    {150, 0}   // End effector
};

IKChain2D chain(joints);

// Solve to reach target position
Vector2 target(75, 75);
bool success = CCDSolver::Solve(chain, target, 1e-3f, 50);

if (success) {
    Vector2 endPos = chain.getEndEffector();
    std::cout << "Reached target! End effector at: " 
              << endPos.x << ", " << endPos.y << std::endl;
}
```

## 🔧 **Core Components Deep Dive**

### Vector Mathematics

The enhanced vector classes provide comprehensive mathematical operations:

```cpp
// Vector2 Advanced Operations
Vector2 v1(3, 4);
Vector2 v2(1, 0);

double length = v1.length();                    // 5.0
double angle = v1.angle();                      // atan2(4, 3)
double angleBetween = v1.angleTo(v2);          // Angle between vectors
Vector2 perp = v1.perpendicular();             // (-4, 3)
Vector2 lerped = v1.lerp(v2, 0.5);            // Linear interpolation
Vector2 reflected = v1.reflect(v2);            // Reflection across v2
bool equal = v1.isApproximatelyEqual(v2);      // Fuzzy comparison

// Vector3 Operations
Vector3 a(1, 0, 0);
Vector3 b(0, 1, 0);
Vector3 cross = a.cross(b);                     // (0, 0, 1)
double dot = a.dot(b);                          // 0.0
Vector3 projected = a.projectOnto(b);           // Project a onto b
```

### Joint Constraints

Realistic joint limitations for natural movement:

```cpp
// 2D Joint Constraints
JointConstraint2D shoulderConstraint(-PI/2, PI/2);  // ±90 degrees
chain.setJointConstraint(0, shoulderConstraint);

// 3D Joint Constraints with rotation axis
Vector3 hingeAxis(0, 0, 1);  // Z-axis rotation
JointConstraint3D kneeConstraint(hingeAxis, 0, PI/2);  // 0-90 degrees
chain3D.setJointConstraint(1, kneeConstraint);

// Constraint validation
double angle = PI;
double constrained = shoulderConstraint.applyConstraint(angle);  // PI/2
bool valid = shoulderConstraint.isWithinConstraints(angle);      // false
```

### IK Chain Management

Flexible chain construction and manipulation:

```cpp
// Construction from positions
std::vector<Vector2> positions = {{0,0}, {10,0}, {20,0}, {30,0}};
IKChain2D chain1(positions);

// Construction from base + lengths
Vector2 base(100, 100);
std::vector<double> lengths = {15, 12, 8};
IKChain2D chain2(base, lengths);

// Chain manipulation
chain1.setJointAngle(1, PI/4);                 // Set joint angle
chain1.updateForwardKinematics();              // Update positions
double totalLength = chain1.getTotalLength();   // Get reach
bool reachable = chain1.isTargetReachable(target);
```

## 🎯 **Solver Algorithms**

### 1. CCD (Cyclic Coordinate Descent)

Fast iterative method working from end effector backward:

```cpp
#include "iksolver/CCDSolver.h"

// 2D CCD
bool success = CCDSolver::Solve(chain2D, target, 1e-3f, 50);

// 3D CCD with settings
CCDSolver3D::SolverSettings settings;
settings.tolerance = 1e-4f;
settings.maxIterations = 100;
settings.maxAngleChange = 0.1f;  // Limit rotation per iteration
settings.respectConstraints = true;

bool success3D = CCDSolver3D::Solve(chain3D, target3D, settings);
```

### 2. FABRIK (Forward And Backward Reaching IK)

Efficient algorithm with forward and backward passes:

```cpp
#include "iksolver/FABRIKSolver.h"

// 2D FABRIK
bool success = FABRIKSolver::Solve(chain2D, target, 1e-3f, 30);

// 3D FABRIK
FABRIKSolver3D::SolverSettings fabrikSettings;
fabrikSettings.tolerance = 1e-3f;
fabrikSettings.respectConstraints = true;

bool success3D = FABRIKSolver3D::Solve(chain3D, target3D, fabrikSettings);
```

### 3. Jacobian-Based Solver

Matrix-based approach with damped least squares:

```cpp
#include "iksolver/JacobianSolver.h"

JacobianSolver::SolverSettings jacobianSettings;
jacobianSettings.tolerance = 1e-4f;
jacobianSettings.maxIterations = 100;
jacobianSettings.dampingFactor = 0.1f;
jacobianSettings.stepSize = 0.1f;
jacobianSettings.useDampedLeastSquares = true;

bool success = JacobianSolver::Solve(chain2D, target, jacobianSettings);
```

## 🎮 **Interactive SDL2 Demo**

The library includes a real-time visualization demo:

```bash
make demo.exe
./demo.exe
```

**Controls:**
- **Left Click**: Set target position
- **Space**: Pause/resume solving
- **R**: Randomize joint angles
- **+/-**: Adjust iteration speed
- **Esc**: Exit

The demo shows:
- Real-time IK solving
- Visual feedback of joint positions
- Error distance calculations
- Algorithm performance comparison

## 🧪 **Advanced Usage Examples**

### Multi-Chain Robot Arm

```cpp
// Create shoulder-elbow-wrist chain
std::vector<Vector2> armJoints = {
    {0, 0},      // Shoulder
    {20, 0},     // Elbow  
    {35, 0},     // Wrist
    {45, 0}      // End effector
};

IKChain2D robotArm(armJoints);

// Add realistic joint constraints
robotArm.setJointConstraint(0, JointConstraint2D(-PI, PI));        // Full rotation shoulder
robotArm.setJointConstraint(1, JointConstraint2D(0, PI));          // Elbow: 0-180°
robotArm.setJointConstraint(2, JointConstraint2D(-PI/2, PI/2));     // Wrist: ±90°

// Solve with constraints
Vector2 targetPos(30, 25);
bool reached = CCDSolver::Solve(robotArm, targetPos);

// Check final configuration
for (size_t i = 0; i < robotArm.size(); ++i) {
    Vector2 jointPos = robotArm.getJointPosition(i);
    double jointAngle = robotArm.getJointAngle(i);
    std::cout << "Joint " << i << ": pos(" << jointPos.x << "," << jointPos.y 
              << ") angle(" << jointAngle * RAD_TO_DEG << "°)" << std::endl;
}
```

### 3D Robotic Manipulator

```cpp
// Create 6-DOF robotic arm
std::vector<Vector3> arm3D = {
    {0, 0, 0},     // Base
    {0, 0, 10},    // Joint 1
    {0, 15, 10},   // Joint 2  
    {0, 25, 10},   // Joint 3
    {0, 35, 10},   // Joint 4
    {0, 45, 10},   // Joint 5
    {0, 50, 10}    // End effector
};

IKChain3D robotArm3D(arm3D);

// Set rotation axes for each joint
robotArm3D.setJointRotationAxis(0, Vector3(0, 0, 1));  // Base rotation (Z)
robotArm3D.setJointRotationAxis(1, Vector3(1, 0, 0));  // Shoulder (X)
robotArm3D.setJointRotationAxis(2, Vector3(1, 0, 0));  // Elbow (X)
robotArm3D.setJointRotationAxis(3, Vector3(0, 1, 0));  // Wrist twist (Y)
robotArm3D.setJointRotationAxis(4, Vector3(1, 0, 0));  // Wrist bend (X)
robotArm3D.setJointRotationAxis(5, Vector3(0, 0, 1));  // End twist (Z)

// Add constraints
robotArm3D.setJointConstraint(1, JointConstraint3D(Vector3(1,0,0), -PI/2, PI/2));
robotArm3D.setJointConstraint(2, JointConstraint3D(Vector3(1,0,0), 0, PI));

// Solve 3D IK
Vector3 target3D(10, 30, 15);
CCDSolver3D::SolverSettings settings3D;
settings3D.respectConstraints = true;
bool success = CCDSolver3D::Solve(robotArm3D, target3D, settings3D);
```

### Performance Comparison

```cpp
#include <chrono>

// Test different algorithms
auto testAlgorithm = [](const std::string& name, auto solverFunc) {
    auto start = std::chrono::high_resolution_clock::now();
    
    bool success = solverFunc();
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << name << ": " << (success ? "SUCCESS" : "FAILED") 
              << " in " << duration.count() << "μs" << std::endl;
};

Vector2 target(50, 50);
IKChain2D testChain({{0,0}, {20,0}, {40,0}, {60,0}});

testAlgorithm("CCD", [&]() { 
    return CCDSolver::Solve(testChain, target, 1e-3f, 50); 
});

testAlgorithm("FABRIK", [&]() { 
    return FABRIKSolver::Solve(testChain, target, 1e-3f, 50); 
});

testAlgorithm("Jacobian", [&]() { 
    JacobianSolver::SolverSettings settings;
    return JacobianSolver::Solve(testChain, target, settings); 
});
```

## 🔍 **Mathematical Utilities**

Safe mathematical operations for numerical stability:

```cpp
#include "iksolver/MathUtils.h"

// Angle utilities
double angle1 = 3.5;
double angle2 = -2.8;
double normalized = normalizeAngle(angle1);          // [-π, π]
double difference = angleDifference(angle1, angle2); // Shortest path
double shortest = shortestAngularDistance(angle1, angle2);

// Safe operations
double safeSqrt = safeSqrt(-1.0);     // Returns 0.0 instead of NaN
double safeAcos = safeAcos(1.5);      // Clamps to valid range
double clamped = clamp(15.0, 0.0, 10.0);  // Returns 10.0

// Floating point comparison
bool isEqual = isApproximatelyEqual(0.1 + 0.2, 0.3, EPSILON);
bool isZero = isApproximatelyZero(1e-10, EPSILON);

// Interpolation and utilities
double lerped = lerp(10.0, 20.0, 0.5);   // 15.0
double squared = square(5.0);             // 25.0
double random = randomInRange(0.0, 1.0);  // Random [0,1]
```

## 🧪 **Testing Framework**

Comprehensive test suite validating all functionality:

```bash
# Run all tests
make test_solver.exe
./test_solver.exe
```

The test suite covers:
- ✅ Vector2/Vector3 operations
- ✅ Mathematical utilities
- ✅ Joint constraints
- ✅ IK chain management
- ✅ All solver algorithms
- ✅ Edge cases and error conditions

## 🔧 **Build Configuration**

### Compiler Requirements
- **C++17** compatible compiler (GCC 7+, MSVC 2017+, Clang 5+)
- **MinGW** on Windows for the provided Makefile
- **SDL2** for visualization demos (via vcpkg recommended)

### Dependencies
```bash
# Install SDL2 via vcpkg (Windows)
vcpkg install sdl2:x64-mingw-dynamic

# Or system package manager (Linux)
sudo apt-get install libsdl2-dev
```

### Custom Build Flags
```makefile
# Optimization levels
CXXFLAGS += -O2        # Standard optimization
CXXFLAGS += -O3        # Maximum optimization
CXXFLAGS += -g         # Debug symbols

# Additional warnings
CXXFLAGS += -Wall -Wextra -Wpedantic

# Mathematical definitions
CXXFLAGS += -D_USE_MATH_DEFINES
```

## 🎯 **Algorithm Comparison**

| Algorithm | Speed | Accuracy | Constraints | Best For |
|-----------|-------|----------|-------------|----------|
| **CCD** | ⚡⚡⚡ | ⭐⭐ | ⭐⭐⭐ | Real-time, games |
| **FABRIK** | ⚡⚡ | ⭐⭐⭐ | ⭐⭐ | Quality results |
| **Jacobian** | ⚡ | ⭐⭐⭐⭐ | ⭐⭐⭐ | Research, precision |

### When to Use Each:

**CCD**: Fast convergence, handles constraints well, perfect for games and real-time applications.

**FABRIK**: Better accuracy than CCD, good for animation and robotics where quality matters.

**Jacobian**: Most accurate, best for scientific applications, handles complex constraints, but computationally intensive.

## 🚨 **Common Pitfalls & Solutions**

### 1. Target Unreachable
```cpp
// Always check reachability first
if (!chain.isTargetReachable(target)) {
    std::cout << "Target out of reach! Max distance: " 
              << chain.getTotalLength() << std::endl;
    // Handle gracefully - stretch toward target or show error
}
```

### 2. Numerical Instability
```cpp
// Use safe math operations
#include "iksolver/MathUtils.h"

double angle = safeAcos(cosValue);  // Instead of std::acos
double distance = safeSqrt(lengthSq);  // Instead of std::sqrt
```

### 3. Constraint Violations
```cpp
// Validate constraints are properly set
if (!constraint.isWithinConstraints(angle)) {
    double corrected = constraint.applyConstraint(angle);
    chain.setJointAngle(i, corrected);
}
```

### 4. Chain Validation
```cpp
// Always validate chain integrity
if (!chain.isValid()) {
    std::cerr << "Chain validation failed!" << std::endl;
    // Fix or recreate chain
}
```

## 🔮 **Future Enhancements**

Potential improvements and extensions:

1. **Additional Solvers**: Implement BFGS, Newton-Raphson methods
2. **Collision Detection**: Add obstacle avoidance capabilities  
3. **Dynamic Constraints**: Time-varying constraints and limits
4. **Parallel Processing**: Multi-threaded solving for complex chains
5. **Optimization**: SIMD vectorization for vector operations
6. **Serialization**: Save/load chain configurations
7. **Animation Support**: Keyframe interpolation and smoothing

## 📚 **References & Further Reading**

- [Inverse Kinematics - Game Programming Gems](https://en.wikipedia.org/wiki/Inverse_kinematics)
- [FABRIK Algorithm Paper](https://www.andreasaristidou.com/FABRIK.html)
- [CCD Implementation Guide](https://sites.google.com/site/auraliusblack/inverse-kinematics-for-dummies)
- [Jacobian Methods in Robotics](https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant)

## 💡 **Contributing**

This library is designed to be extended and improved. Key areas for contribution:
- Additional solver algorithms
- Performance optimizations
- Better constraint systems
- More comprehensive testing
- Documentation improvements

---

**Happy IK Solving!** 🦾✨

This library provides you with professional-grade inverse kinematics capabilities with multiple solving strategies, realistic constraints, and comprehensive mathematical utilities. Whether you're building games, robotics simulations, or research applications, you have the tools needed for robust IK solving.
