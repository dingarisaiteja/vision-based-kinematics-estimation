# UAV Vision-Based Kinematics Estimation Framework

This repository hosts the software architecture developed for autonomous quadcopter landing operations within GPS-denied environments. By deploying low-latency image processing pipelines paired with empirical scaling models, the framework accurately estimates multi-axis positions and linear velocities using a single downward-facing monocular camera.

## 📐 Mathematical & Empirical Models

### 1. Empirical Position Regressions
To counteract real-world lens aberrations, high-altitude pixel degradation, and resolution variations, empirical regression parameters were modeled via continuous curve-fitting. The non-linear equations convert the detected target pixel radius ($R$) and center variations ($\Delta x, \Delta y$) into spatial dimensions ($X, Y, Z$):

$$Z = 854.02 \cdot \left(\frac{ND}{0.869}\right) \cdot \left(\frac{VR}{4608}\right) \cdot R^{-1.03}$$

$$X = 0.00253 \cdot \left(\frac{854.02 \cdot R^{-1.03}}{4.49}\right) \cdot \left(\frac{ND}{0.869}\right) \cdot \left(\frac{4608}{VR}\right) \cdot \Delta x$$

$$Y = -0.0046 \cdot \left(\frac{854.02 \cdot R^{-1.03}}{7.98}\right) \cdot \left(\frac{ND}{0.869}\right) \cdot \left(\frac{4608}{VR}\right) \cdot \Delta y$$

*Where configured baseline constraints are initialized as: Target Diameter ($ND = 0.46\text{m}$) and Sensor Virtual Resolution ($VR = 3840\text{px}$).*

### 2. Kinematic Velocity Differentiation
Translational velocity components and net spatial velocity vectors are derived across discrete video tracking time-steps ($\Delta t = 1 / \text{FPS}$):

$$V_x = \frac{X_{t} - X_{t-1}}{\Delta t}, \quad V_y = \frac{Y_{t} - Y_{t-1}}{\Delta t}, \quad V_z = \frac{Z_{t} - Z_{t-1}}{\Delta t}$$

$$V = \sqrt{V_x^2 + V_y^2 + V_z^2}$$

---

## 📁 Code Architecture

The computational system is organized into decoupled validation pipelines:

* **`static_testing/image_processor.m`**: Standard verification pipeline designed to ingest individual frame matrices, isolate target properties through adaptive Hough parameters (`imfindcircles`), and validate enclosed object topologies.
* **`dynamic_tracking/video_pipeline.m`**: Dynamic tracking pipeline that streams data sequences, processes fast pixel arrays via vectorized masking matrices, runs real-time time-marching kinematic integrations, generates text display overlays, and outputs high-fidelity compiled tracking media.

---

## 📊 Performance & Analytical Results
* **Precision Constraints**: Achieved localized coordinate estimations with limited error boundaries ($\le 1\%$ at near distances), verifying performance parity against perfectly calibrated pinhole camera configurations.
* **Dynamic Range Tracking**: Maintained robust state estimation parameters with cross-axis tracking stability under $6\%$ variations across rapid descent profiles spanning $0.20\text{m/s}$ to $0.70\text{m/s}$.
