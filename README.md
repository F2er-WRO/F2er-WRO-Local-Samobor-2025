# WRO FUTURE ENGINEERS 2025: Team F2er


## Pictures - Team and vehicle

### Team
Team information:

•	Name: F2er

•	Members: Ana Petrović, Bruna Vlatković, Patrik Vranješ

•	Mentor: Ana Sović Kržić

•	Organization: Faculty of Electrical Engineering and Computing

•	Country: Zagreb, Croatia

<p align="center">
  <img src="images/team.jpg" alt="Team" width="300"/>
</p>


### Vehicle 

The vehicle is built using the `fischertechink STEM Coding Competition Robot Kit`. 

<p align="center">
  <img src="images/vehicle1.jpg" alt="Vehicle 1" width="200"/>
  <img src="images/vehicle2.jpg" alt="Vehicle 2" width="200"/>
  <img src="images/vehicle5.jpg" alt="Vehicle 5" width="200"/>
  <br/>
  <img src="images/vehicle3.jpg" alt="Vehicle 3" width="294"/>
  <img src="images/vehicle4.jpg" alt="Vehicle 4" width="250"/>
  <img src="images/vehicle6.jpg" alt="Vehicle 6" width="157"/>
</p>








## Performance Videos

The videos are public and posted on YouTube.

1) Open Challenge: https://youtube.com/shorts/FNWYoWpSo5U?feature=share

2) Obstacle Challenge: https://youtube.com/shorts/vl-P6Tf0zsA?feature=share



## GitHub utilization
Our repository is on the following link: https://github.com/F2er-WRO/Project

It includes the codes for the robot setup such as camera, controller and display. Two codes for the upload and run process, as well as the `open_challenge.py` and `obstacle_challenge.py`. 

Lastly, the repository includes the README.md file suitable with the following rules of the WRO Organization. 


## Build, Compile and Upload Process

The project was developed using **Python** as the programming language. All source code was written and edited using `Visual Studio Code (VS Code)`.

To connect and control the robot remotely, we used the `paramiko` package, which allowed us to access the TXT 4.0 controller via SSH. 

We connect the TXT 4.0 controller and the laptop to the same local Wi-Fi and keep the controller updated before use. We pair by entering the controller’s IP address (shown on the device under Info → Wi-Fi) and the API key (Settings → API key). Keeping both devices on one local network avoids router restrictions that block cross-network access. This was necessary because we had to include **the controller’s IP address** in the code.

If the controller does not power on, we first recheck wiring and battery charge, then proceed with software troubleshooting.

During the early stages of development, we tested our code by running scripts directly through the command line (using *python scripts/upload_and_run.py*). 

To start the robot in a way the rules of the competition provide, we followed these steps:

1. In VS Code, after writing and saving the Python files, connect to the robot using the **"Upload and Run File"** option to transfer the Python script to the controller and start execution.
2. The robot will automatically begin executing the code once the upload completes and by clicking the run button on its interface.

### Additional Python Libraries Used:
- `opencv-python` – for image processing and color detection
- `numpy` – for numerical operations and array handling
- `fischertechnik` – to communicate with motors, servos, and sensors

## Mobility Management 
We decided to use the official `fischertechnik STEM Coding Competition kit`, which was specifically developed for this "Future Engineers" competition category of the World Robot Olympiad. The base was assembled strictly following the official step-by-step instructions provided in the manual, ensuring stable construction, proper wheel alignment, and optimal weight distribution.

The image displays **the specifications of the components** used from the specified set.
<p align="center">
  <img src="images/fischertechnik_set_parts.jpg" alt="Set parts" width="300"/>
</p>  

The drive motor is initialized using the encoder motor factory. Additionally, a motor step counter is created and linked to the encoder motor to track the number of steps taken during movement. 

The steering mechanism is controlled by a separate servomotor **Micro Servo 4.8/6V**, which allows the robot to adjust its direction in real time based on obstacle detection and navigation logic.

We began assembling the robot by constructing the lower base frame using red and black structural blocks, followed by adding the rear wheels with axle holders and spacers for stability. Next, we installed the encoder motor to complete the drive system. 

The front steering mechanism is supported by the Micro Servo. The front wheels were then attached to the steering links, completing the basic mobility structure. 

Afterwards, we positioned the **TXT 4.0 controller** on top of the base frame and connected all cables from the sensors, motor, camera, and servo to the corresponding ports.

At the start of the program, we initialize all the hardware components using the official `fischertechnik` Python library. These include the controller, sensors, motors, camera, and counters.

```python
txt_factory.init()
txt_factory.init_input_factory()
txt_factory.init_motor_factory()
txt_factory.init_servomotor_factory()
txt_factory.init_counter_factory()
txt_factory.init_usb_factory()
txt_factory.init_camera_factory()
```
### Convention
We define forward as the robot moving with left/right wheels spinning in opposite directions (mirrored). Positive steering angles turn the front wheels to the left; positive heading change is a left turn. This convention keeps logs, diagrams, and odometry consistent across modules. We use time-based 90° turns as a baseline and re-check them whenever tire friction, weight, or surface change. We treat right/left turns as separate calibrations and keep a margin to avoid over-rotation in tight spaces.


## Power and Sense Management

The robot is powered by an **8.4V 1800mAh NiMH battery pack** with short circuit protection, recharged using the standard charger from the robot kit introduced earlier.

The robot has **three ultrasonic sensors**, each connected to a different port: ***I1 (front), I5 (right), and I3 (left)***. 

Each sensor consists of a dual transducer that sends and receives ultrasonic pulses, enabling accurate distance calculations in real time. The sensors were securely connected using the standard plug-and-cable system provided in the kit, following the official `fischertechnik` assembly guidelines. 

Their placement was chosen to provide full coverage on three sides of the robot, ensuring it can orient itself within confined environments and react to dynamic obstacles. For wall-following we side-mount the ultrasonic sensor to look along the wall rather than straight ahead. This stabilizes the distance signal and reduces spurious spikes from corners and cross-traffic. 

Here is a simplified version of the **electrical scheme** of our robot:
<p align="center">
  <img src="images/el_scheme.png" alt="Electrical scheme" width="300"/>
</p>  

## Troubleshooting
**Robot pulls left/right**: re-center steering and re-fit counts-per-meter on the current surface.

**Late reactions**: check FPS stability and CPU load; keep frame size fixed.

**No network**: verify same LAN, IP, credentials, and that the controller UI shows Wi-Fi “ready”.

**Noisy distance readings**: re-aim the ultrasonic sideways for wall-follow and check for soft mounts or loose brackets.


## Obstacle Detection
### Step-Based Turning Logic

For the **Open Challenge**, our initial idea was to let the robot check the distances from the left and right sensors, compare them, and choose **the smaller one which means the robot is closer to the inner wall on that side**. Then it adjusts its path to go around that side. 

The logic behind the programming approach is shown in the flow diagram.
<p align="center">
<img src="images/flow_dia.png" alt="Flow diagram" width="300"/>
</p>  


We spent time testing and adjusting values like driving speed, turning speed, and how sharp the turns are, to make everything run smoothly. 

We implemented step-based turning logic using **a counter variable** to track the number of completed turns. The robot continuously evaluates distances using its left and right ultrasonic sensors. 

When it detects a corner, it triggers a turning function `turn_left()` or `turn_right()` that uses the servomotor for steering and the encoder motor for movement. This modular approach allows the robot to navigate around the central box by following walls and turning only when necessary.

```python
def turn_left():
    print("Left turn for 90")
    print(TXT_M_I3_ultrasonic_distance_meter.get_distance())
    TXT_M_M1_encodermotor.set_speed(150, TXT_M_M1_encodermotor.CCW)
    TXT_M_S1_servomotor.set_position(350)  # sharp left
    time.sleep(2)
    
    TXT_M_S1_servomotor.set_position(210)  # wheels in neutral
    time.sleep(0.2)
    
    while(TXT_M_I3_ultrasonic_distance_meter.get_distance()>100):
       TXT_M_M1_encodermotor.set_speed(260, TXT_M_M1_encodermotor.CCW)
```

We than used the counter to measure when **three full laps** have been completed. When it reaches 12, it stops at the starting point.

```python
while (counter < 12):
    if left == 0:
        if TXT_M_I5_ultrasonic_distance_meter.get_distance() > 50:
            turn_right()
            counter=counter + 1
    if left == 1:
        if TXT_M_I3_ultrasonic_distance_meter.get_distance() > 50:
            turn_left()
            counter=counter + 1
```

### Camera and Color Detection

In this project, we used the `OpenCV library` for computer vision to detect obstacles of specific colors and to measure their height within the camera frame.

We initialize the camera via the TXT controller, disable rotation, fix the resolution to 320×240 and 15 fps, and start the stream. We use the `TXT_M_USB1_1_camera` object as the device interface; once we call `start()`, frames are available to the CV pipeline.
Fixing resolution and FPS stabilizes latency and aspect ratio, which makes contour detection and centroid tracking consistent across runs.

```python
TXT_M_USB1_1_camera.set_rotate(False)
TXT_M_USB1_1_camera.set_height(240)
TXT_M_USB1_1_camera.set_width(320)
TXT_M_USB1_1_camera.set_fps(15)
TXT_M_USB1_1_camera.start()
```

Before detecting the color, we convert each frame from the default **BGR (Blue, Green, Red)** format to the **HSV (Hue, Saturation, Value)** color space. This improves color detection accuracy under different lighting conditions by separating color (hue) from brightness (value). This way we made sure that the color detection does not depend on the brightness and lighting in the room.

After developing the **color recognition logic based on pixel counting**, we concluded that we could try implementing this logic for turns and direction decisions in the **Open Challenge**. The color of the line at the turn that the robot detects first we called the main color. Based on this color, the robot turns in the predefined direction. If the main color is **blue**, it turns left; if the main color is **orange**, it turns right. 

```python
orange_lower_hsv = np.array([5, 100, 100])  
orange_upper_hsv = np.array([25, 255, 255])

blue_lower_hsv = np.array([100, 150, 50])  
blue_upper_hsv = np.array([130, 255, 255])

def zavoj_boja(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_orange = cv2.inRange(hsv_frame, orange_lower_hsv, orange_upper_hsv)
    mask_blue = cv2.inRange(hsv_frame, blue_lower_hsv, blue_upper_hsv)
    #we count the pixels in the masks
    blue_pixels = cv2.countNonZero(mask_blue)
    orange_pixels = cv2.countNonZero(mask_orange)
    print('plavi pikseli ', blue_pixels, 'narancasti pikseli ', orange_pixels)
    #compare the number of pixels and return the color with the higher pixel count
    if orange_pixels > blue_pixels:
        return "orange"
    else:
        return "blue"
```

While navigating around the inner wall, we wanted the robot to maintain a consistent distance from the inner wall. To enable this, we created a variable called `udaljenost_prva` (translated as initial_distance), in which we stored the sensor reading at the starting point from either the left or right side, depending on the main color. This helped us improve the accuracy of returning to the starting position after completing three laps.


For the **Obstacle Challenge**, we also need to define specific HSV value ranges for detecting **red** and **green** regions. Red is split into two ranges due to its position at both ends of the hue spectrum:

```python
RED_LOWER1 = np.array([0, 70, 50])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 70, 50])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([40, 70, 50])
GREEN_UPPER = np.array([80, 255, 255])
```

After, we split the frame the camera sees into three parts, left, right and center.

The `max_height()` function calculates the height of the tallest detected colored object by finding external contours in the mask. It returns `h` which is the maximum bounding rectangle height in that area. Next, we extract **contours** to describe object boundaries. A contour is an ordered set of points that traces the outline of a blob—the border between foreground and background. We use contours to compute the object centroid for tracking, to estimate size and shape (e.g., perimeter, area, circularity), and to validate detections by filtering out noise and irregular blobs before downstream actions.

Combining the HSV value of red and green and the height of the shape the camera sees, we counted **the number of pixels**. We used this logic to determine which obstacle is the closest to the robot, meaning the bigger number of pixels of a certain color point to the traffic sign in front of the camera. 

```python
def avoid_obstacle():
    for _ in range(5):  
        video_capture.read()
    ret, frame = video_capture.read()
    boje = detect_color(frame)
    hr=height_red(frame)
    hg=height_green(frame)
    print("red and green height ", hr, hg)
    color_near=close_color(frame)
    print("The closest obstacle:", color_near)
```

### Post-Competition Observations & Control Refactor
After the **national competition** we observed that ultrasonic readings **can be unstable due to echoes** and other interference, which occasionally produces spurious large values. Therefore, we apply exponential smoothing to reduce the impact of noise.

```python
if distance_left < 1000:
    filtered_distance_left  = alpha * filtered_distance_left  + (1 - alpha) * distance_left
if distance_right < 1000:
    filtered_distance_right = alpha * filtered_distance_right + (1 - alpha) * distance_right
```
We use exponential smoothing to blend the previous filtered value with the new measurement:
filteredₜ = α · filteredₜ₋₁ + (1 − α) · measurementₜ.
The α parameter sets the trade-off: a higher α yields slower response but stronger noise suppression; a lower α reacts faster but filters less.

To improve wall-following we are migrating to a **PID controller** fed by left/right ultrasonic distances.
