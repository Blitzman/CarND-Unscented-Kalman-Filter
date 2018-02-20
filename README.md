# Unscented Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
Self-Driving Car Engineer Nanodegree Program

[dataset1]: ./img/dataset1_screenshot.png
[dataset2]: ./img/dataset2_screenshot.png

## Requirements

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Udacity Simulator [Download link](https://github.com/udacity/self-driving-car-sim/releases)


Tips for setting up the environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Compilation and Execution

1. Clone this repo
2. cd to repo root dir.
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
7. `./UnscentedKF`
8. Run simulator

## Rubric

### Compiling

#### Your code should compile

Code compiles without errors using `cmake` and `make`. [CMakeLists.txt]( https://github.com/Blitzman/CarND-Unscented-Kalman-Filter/blob/master/CMakeLists.txt ) was unchanged from the original version provided in the starter code.

### Accuracy

#### px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, 0.40, 0.30] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1"

After running the algorithm against Dataset 1 and Dataset 2 in the simulator and comparing the estimated positions with the ground truth, the RMSE values are:

* Dataset 1 : [0.0692, 0.0829, 0.3333, 0.2344]
* Dataset 2 : [0.0685, 0.0693, 0.5846, 0.2473]

![dataset1][dataset1]
![dataset2][dataset2]

### Follows the Correct Algorithm

#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The implementation of the Kalman filter can be found in [ukf.cpp]( https://github.com/Blitzman/CarND-Unscented-Kalman-Filter/blob/master/src/ukf.cpp ). It contains the needed functions as taught in the lessons: `Prediction()` for predicting sigma points, the state, and the state covariance matrix; `UpdateLidar()` to perform an update of the state and the covariance matrix using a laser measurement  and `UpdateRadar()` for updating the state and the state covariance matrix using a radar measurement. The unscented Kalman filter prediction and update functions are used in the general flow of the sensor fusion algorithm which is coded in `ProcessMeasurement()`.

#### Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurements are handled in [ukf.cpp](https://github.com/Blitzman/CarND-Unscented-Kalman-Filter/blob/master/src/ukf.cpp) in `ProcessMeasurement()` lines 114 to 154. First, an empty measurement vector for the CTRV model is created which will be initialized depending on the sensor type. If it is a RADAR measurement, we convert the polar coordinates rho, phi, and rho_dot to cartesian and initialize the vector with the corresponding x, y, and v values. If the measurement comes from a LIDAR sensor we just take x and y and assume zero velocity. In both cases, we assume zero psi and psi_dot. After that, we update the timestamp to compute the elapsed time for the next measurement. The initial covariance matrix P is already initialized in the constructor to 1.0 diagonal. The measurement noise covariance matrices are also initialized accordingly in the constructor using laser and radar standard deviation measurement noise values.

#### Your Kalman Filter algorithm first predicts then updates.

The algorithm predicts first in line 160 [ukf.cpp](https://github.com/Blitzman/CarND-Unscented-Kalman-Filter/blob/master/src/ukf.cpp). That prediction step makes use of some subroutines (`GenerateSigmaPoints()` and `PredictSigmaPoints()` to generate and predict sigma points and then predict the state mean and the covariance matrix). Once prediction is done, update is performed between lines 165 and 169.

#### Your Kalman Filter can handle radar and lidar measurements.

In [ukf.cpp](https://github.com/Blitzman/CarND-Unscented-Kalman-Filter/blob/master/src/ukf.cpp) the update step is performed differently depending on the type of sensor that provides the measurement. If it comes from a RADAR sensor, we call the `UpdateRadar()` function. If the measurement comes from a LIDAR sensor we just update using `UpdateLidar()`.

### Code Efficiency

#### Your algorithm should avoid unnecessary calculations.

Our implementation has been refined to avoid common coding and efficiency mistakes.

* We have refactored the sigma point generation and prediction steps into individual functions `GenerateSigmaPoints()` and `PredictSigmaPoints()` to avoid excessive clutter in the UKF prediction function.
* The angle normalization step has been refactored into `NormalizeAngle()` to avoid unnecessary code repetition.
* Precomputations have been applied to those values which are reused such as `cos_psi`, `sin_psi`, and `delta_t_2` in `PredictSigmaPoints()` for instance.
