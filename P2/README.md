# Rescue People 

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P2/resources/figures/1.png" alt="explode"></a> 
</div>

<h3 align="center"> Rescue People </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-developing-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Task description](#Task-description)
- [Robot API](#Robot-API)
- [convert latitude and longitude coordinates to x and y](#convert-latitude-and-longitude-coordinates-to-x-and-y)
- [Create an imaginary area around the survivors](#Create-an-imaginary-area-around-the-survivors)
- [Zigzag Sweep for Area Coverage](#Zigzag-Sweep-for-Area-Coverage)
- [Person detectection and estimate position](#Person-detectection-and-estimate-position)
- [Video demo](#Video-demo)


## Task description
The goal of this exercise is to implement the logic that allows a quadrotor to recognize the faces of lost people and save their locations in order to perform a subsequent rescue maneuver.

## Robot API
#### Sensors and drone state
- HAL.get_position() - Returns the actual position of the drone as a numpy array [x, y, z], in m.
- HAL.get_velocity() - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s
- HAL.get_yaw_rate() - Returns the actual yaw rate of the drone, in rad/s.
- HAL.get_orientation() - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad.
- HAL.get_roll() - Returns the roll angle of the drone, in rad
- HAL.get_pitch() - Returns the pitch angle of the drone, in rad.
- HAL.get_yaw() - Returns the yaw angle of the drone, in rad.
- HAL.get_landed_state() - Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown.

#### Actuators and drone control
The three following drone control functions are non-blocking, i.e. each time you send a new command to the aircraft it immediately discards the previous control command.

**1. Position control**
- HAL.set_cmd_pos(x, y, z, az) - Commands the position (x,y,z) of the drone, in m and the yaw angle (az) (in rad) taking as reference the first takeoff point (map frame)
  
**2. Velocity control**
- HAL.set_cmd_vel(vx, vy, vz, az) - Commands the linear velocity of the drone in the x, y and z directions (in m/s) and the yaw rate (az) (rad/s) in its body fixed frame

**3. Mixed control**
- HAL.set_cmd_mix(vx, vy, z, az) - Commands the linear velocity of the drone in the x, y directions (in m/s), the height (z) related to the takeoff point and the yaw rate (az) (in rad/s)

#### Drone takeoff and land
Besides using the buttons at the drone teleoperator GUI, taking off and landing can also be controlled from the following commands in your code:

- HAL.takeoff(height) - Takeoff at the current location, to the given height (in m)
- HAL.land() - Land at the current location.

#### Drone cameras
- HAL.get_frontal_image() - Returns the latest image from the frontal camera as a OpenCV cv2_image
- HAL.get_ventral_image() - Returns the latest image from the ventral camera as a OpenCV cv2_image

#### GUI
- GUI.showImage(cv2_image) - Shows a image of the camera in the GUI
- GUI.showLeftImage(cv2_image) - Shows another image of the camera in the GUI


## convert latitude and longitude coordinates to x and y

The following functions transform geographic coordinates in degrees, minutes, and seconds (DMS) to a cartesian coordinate system (x, y) in meters.

```
       GPS_COORDINATES (DMS)
           /         \
     LATITUDE       LONGITUDE
        |               |
   parse_dms()      parse_dms()
        |               |
   dms_to_decimal()  dms_to_decimal()
        |               |
    Decimal Lat      Decimal Lon
           \         /
        calculate_dms_coordinates()
                 |
            (x, y) Coordinates

```

### 1. dms_to_decimal(deg, min, sec, direction)
This function converts coordinates in DMS format (degrees, minutes, seconds) to decimal degrees, which are easier to handle mathematicall and to work in the simulator 
that is the gazebo world.

**Parameters:**

- deg: (int/float) The degrees of the coordinate (e.g., 40º). 
- min: (int/float) The minutes of the coordinate (e.g., 16'). 
- sec: (float) The seconds of the coordinate (e.g., 48.2"). 
- direction: (str) The direction of the coordinate. It can be 'N' (north), 'S' (south), 'E' (east), or 'W' (west). This indicates the hemisphere or direction.

**Return:**

A decimal number representing the coordinate in decimal degrees. If the direction is 'S' or 'W', the number is converted to negative, as coordinates to 
the south or west are represented with negative values.

**Usage:**
```py
dms_to_decimal(40, 16, 48.2, 'N')  # Returns 40.28005555555555
```

### 2. parse_dms(dms_str)
This function takes a string with coordinates in DMS format and converts them to decimal degrees using the dms_to_decimal function. The function uses
regular expressions to extract degrees, minutes, seconds, and direction.

**Parameters:**

- dms_str: (str) A string containing a coordinate in DMS format. Example format: "40º16’48.2”N".

**Return:**

A value in decimal degrees calculated from the DMS string.

**Usage:**
```py
parse_dms("40º16’48.2”N")  # Returns 40.28005555555555
```

### 3. calculate_dms_coordinates(gps_lat, gps_lon)
This function calculates the x and y coordinates in meters for a given geographic location in DMS format (degrees, minutes, seconds). The x and y coordinates represent 
the distance in meters from that location relative to a reference point at latitude and longitude (0,0 in a local reference system). 
*calculate_dms_coordinates* uses *parse_dms* to convert latitude and longitude coordinates from DMS format to decimal degrees and then calculates the differences in meters 
relative to a reference point.

**Parameters:**

- gps_lat: (str) String with the latitude in DMS format (e.g., "40º16’47.23”N").
- gps_lon: (str) String with the longitude in DMS format (e.g., "3º49’01.78”W").

**Return:**

Two values: the x and y coordinates in meters relative to the reference point (0, 0). x corresponds to longitude and y to latitude.

**Usage:**
```py
calculate_dms_coordinates("40º16’47.23”N", "3º49’01.78”W")  # Returns something like (40.58, -29.99)
```


## Create an imaginary area around the survivors

Once we have the point provided in DMS (degrees, minutes, seconds) format converted to decimal, we can see that the survivors' area is in the fourth quadrant, as the ship is located at (0,0) and the given point is at (40, -30). Now, we need to create an imaginary area and scan the zone to search for survivors. 

We select the point given in the wiki as the upper-right corner of the rectangle that we will use as the search area (northeast corner).

```py
# Define the corners of a rectangle area at the point where we are told that the accident occurred
area_limits = [(x_accident, y_accident), # NORTH EAST
               (x_accident - AXIS_X_OFFSET, y_accident), # NORTH WEST
               (x_accident - AXIS_X_OFFSET, y_accident - AXIS_Y_OFFSET), # SOUTH WEST
               (x_accident, y_accident - AXIS_Y_OFFSET)] # SOUTH EAST
```

With the offset constants, we move along the 2D space.

Below, you can see an illustration of this explanation.

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P2/resources/figures/2.png" alt="explode"></a> 
</div>


## Zigzag Sweep for Area Coverage
The goal of this code is to generate a set of waypoints that a navigation system of the drone can follow to cover a rectangular area. The trajectory follows a zigzag pattern, alternating between 
east-to-west and west-to-east movements, and descending one unit along the Y-axis (north to south) after each pass.

*generate_waypoints()* Function

**Parameters**

The system requires the following input parameters to define the area:

- x_accident: X-coordinate of the northeast corner of the area to cover.
- y_accident: Y-coordinate of the northeast corner of the area to cover.
- AXIS_X_OFFSET: Horizontal displacement along the X-axis (width of the area).
- AXIS_Y_OFFSET: Vertical displacement along the Y-axis (height of the area).

**Return**

This function is responsible for generating the waypoints (coordinates) that the navigation system will follow to cover the area. List of tuples (x, y) with waypoints.

**Usage**

```py
  waypoints = generate_waypoints(x_accident, y_accident, AXIS_X_OFFSET, AXIS_Y_OFFSET)
```

You can see the trajectory bellow

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Service-Robots_25-26/blob/main/P2/resources/figures/3.png" alt="explode"></a> 
</div>

## Person detectection and estimate position

#### detect_person_by_color(image)

Detects a person whose body color is light green in the provided image.
This function is responsible for detecting areas in the image that match the light green color range, which is assumed to be the color of the person’s body.

- *Convert Image to HSV*: The image is first converted from BGR to HSV color space, which is more effective for color detection.
- *Color Range Definition*: A light green color range is defined using lower and upper HSV values.
- *lower_green* = np.array([30, 40, 40])
- *upper_green* = np.array([90, 255, 255])
- *Mask Creation*: A mask is generated that highlights regions of the image within the light green color range.
- *Morphological Operations*: A closing operation is applied to clean up the mask and remove small artifacts.
- *Contour Detection*: Contours are found in the mask to identify the potential areas where the person might be.
- *Contour Area Filtering*: Only contours with an area larger than a pre-defined threshold (CONTOUR_AREA) are considered valid detections. I do it
to differentiate contours that can be the size of a body, the height of the drone influences and that is why it is parameterized, to adjust it to the application and the context.
- *Centroid Calculation*: The centroid (center of mass) of the valid contour is calculated, which serves as the detected position of the person in image coordinates.

*The function returns*:

- [True, (cx, cy)]: If a person is detected (with the centroid of the contour in image coordinates).
- [False, (0, 0)]: If no person is detected.

I decided to do it this way because although the initial idea was to recognize faces, it was getting complicated and asking colleagues,
I took ideas and decided to recognize the person by the body.

#### estimate_person_position(dron_position, contour_center)

Converts the detected person's position in the image to a real-world position based on the drone's coordinates. with round_pose(pose), rounds the coordinates of
the detected person to one decimal place.

The person is assumed to be directly below the drone. Thus, the person's position in the x-y plane is the same as the drone's x-y position.
This simplifies the conversion from image to real-world coordinates.

*return*: The estimated real-world coordinates of the person (person_x, person_y).

#### is_new_person_position(new_pose, saved_poses)

Checks if the newly detected person is far enough from previous detections to be considered a new position.
The function compares the new position to each saved position. If the absolute difference in x or y coordinates is less than a pre-defined THRESHOLD,
the function returns False, indicating the position is too close to an existing detection.


## Video demo

I have to say that the simulation does not work the same when recording the screen, in the simulation it detects four bodies and in reality it detects all (six).

[video demo at x4 speed 2025](https://urjc-my.sharepoint.com/:v:/g/personal/g_alcocer_2020_alumnos_urjc_es/IQB6Xfl03le-QY_l2qW5fftyAfKa5RJZlt9FCEtGI40Qk7g?e=ZuiUxk)
[video demo at x4 speed 2024](https://urjc-my.sharepoint.com/personal/g_alcocer_2020_alumnos_urjc_es/_layouts/15/stream.aspx?id=%2Fpersonal%2Fg%5Falcocer%5F2020%5Falumnos%5Furjc%5Fes%2FDocuments%2FRescuePeople%2Emp4&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2E9eec89b8%2De8cc%2D4779%2Da82f%2D6a587ef9327e)




