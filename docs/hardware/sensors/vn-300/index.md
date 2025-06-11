# VectorNav vn-300

The VectorNav VN-300 is a collection of sensors that provide heading, position, and attitude information.
It contains an IMU with an Accelerometer, Gyroscope, Magnetometer, and Barometer as well as a GNSS receiver.
It has inbuilt sensor fusion and error correction algorithms to provide accurate and reliable data.

For our purposes, the IMU data is most important. GNSS data is too slow and not accurate enough.

## IMU General Information
IMU stands for Inertial Measurement Unit. It provides information about the orientation, velocity and acceleration.
Its main drawback for usage in navigation is that it drifts over time because noise and bias accumulate.

## IMU Data
|                        | Accelerometer | Gyroscope               | Magnetometer  |
| ---------------------- | ------------- | ----------------------- | ------------- |
| Range                  | ±16g          | ±2000°/s                | ±2.5Gauss     |
| In-Run Bias Stability  | < 0.04mg RMS  | < 10°/hr (5-7°/hr typ.) | -             |
| Noise Density          | 0.14mg/√Hz    | 0.0035°/s/√Hz           | 0.2μGauss/√Hz |
| Bandwidth              | 260Hz         | 256Hz                   | 200Hz         |
| Cross-Axis Sensitivity | ±0.05°        | < 0.05°                 | ± 0.05°       |

## Heading
- Magnetic: 2.0° RMS (suitable magnetic environment & calibration)
- INS: 0.2°, 1σ (with sufficient motion)
- GNSS:
  - 0.5m Baseline: 0.3° to 0.6° RMS
  - 1m Baseline: 0.15° to 0.3° RMS
  - 2m Baseline: 0.08° to 0.15° RMS

## Pitch / Roll
- Static: 0.5° RMS
- INS: 0.03°, 1σ (with sufficient motion)

## Position / Velocity
- Horizontal Position Accuracy: 1.0m RMS
- Vertical Position Accuracy: 1.5m RMS
- Free Inertial Position Drift: 3.0 cm/s²
- Velocity Accuracy: < 0.05m/s 

## Additional Features
- Global Update Rate: up to 400Hz
- Has the option to dynamically switch between different modes
- Uses a Kalman Filter for sensor fusion
- Has in-built error correction and noise reduction

## Datasheets
- [High-level overview (2 pages)](assets/vn-300-datasheet-rev2.pdf)
- [In depth datasheet (175 pages)](assets/vn-300-user-manual-rev_2-46.pdf)