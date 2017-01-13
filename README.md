# PanCam 360 degree picture

## Overview

This package automatically takes 360 degree pictures with the PanCam. When the component is started it instructs the PanCam to a predefined position, waits until it reaches this position and saves the next incoming image frames to make sure the images are not blurred because of the PTU movement. Then in proceeds to the next position until all pictures are taken. Upon completion of the set the component stops itself (state set to `STOPPED`).

The tilt angle, pan angle separation and number of images are defined via the parameters.

For logging the package has a special structure that contains the timestamped left and right pictures with the PTU angles as well as a counter indicating to which set the images belong to.

**Authors: Karl Kangur  
Contact: Martin Azkarate  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**


## Installation

### Dependencies

This package depends on the following packages:

* [drivers/controldev](https://github.com/rock-drivers/drivers-controldev)
* [drivers/orogen/controldev](https://github.com/rock-drivers/drivers-orogen-controldev)

### Building

In order to install, clone the latest version from this repository into your workspace under `control/orogen/motion_translator`, add the following line to `autoproj/manifest` under `layout:`

    - control/orogen/pancam_panorama

Execute the following to build the package:

    $ autoproj build


## Basic Usage

### motion_translator

#### Inputs

* **`pan_angle_in`** (/double)

Feedback from the PTU pan angle, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

* **`tilt_angle_in`** (/double)

Feedback from the PTU tilt angle, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

* **`left_frame_in`** (base::samples::frame::Frame)

Input of the left camera of the PTU.

* **`right_frame_in`** (base::samples::frame::Frame)

Input of the right camera of the PTU.

#### Outputs

* **`pan_angle_out`** (/double)

Pan motion command directed to a pan-tilt unit, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

* **`tilt_angle_out`** (/double)

Tilt motion command directed to a pan-tilt unit, for example for the [ptu_directedperception](https://github.com/rock-drivers/drivers-orogen-ptu_directedperception) package.

* **`left_frame_out`** (base::samples::frame::Frame)

Output of the left camera of the PTU.

* **`right_frame_out`** (base::samples::frame::Frame)

Output of the right camera of the PTU.

* **`frame`** (/pancam_panorama/PanCamTimestampedFrame)

Output of a structure containing both images as well as the PTU pan and tilt angles with a timestamp, useful for logging.

#### Parameters

* **`positionTilt`** (/double)

PanCam tilt position for all positions in degrees.

* **`panoramaPictures`** (/int)

How many pictures to take for a 360 panorama picture, must be an odd number.

* **`positionSeparation`** (/double)

Angle separation between pictures, in degrees.

* **`positionErrorMargin`** (/double)

PanCam pan position error margin in degrees.

* **`frameDelayTimeMs`** (/double)

Minimum delay in milliseconds before saving a frame after the PanCam reaches its goal position.

* **`resolutionPerPosition`** (/double)

Axis resolution per position moved (in seconds/arc), used to transform degrees to motor steps, see the datasheet for the value.

