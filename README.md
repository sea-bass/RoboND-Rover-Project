## Project: Search and Sample Return
### By Sebastian Castro - 2018

---

[//]: # (Image References)
[autonav]: ./misc/autonomous_nav.PNG
[notebook1]: ./misc/notebook_output_1.png
[notebook2]: ./misc/notebook_output_2.png
[notebookvid]: ./misc/notebook_video.PNG

### Notebook Analysis

Below are some snapshots of the `Rover_Project_Test_Notebook.ipynb` notebook. The perception approach is discussed in the **perception.py** section below.

Snapshot of notebook with provided calibration image
![Notebook snapshot 1][notebook1]

Snapshot of notebook with provided calibration image
![Notebook snapshot 2][notebook2]

Snapshot of generated video. Refer to the generated **test_mapping.mp4** file for the full video.
![Notebook video snapshot][notebookvid]

### Autonomous Navigation and Mapping

#### perception.py
The color thresholding algorithm was modified with additional input and logic to accept upper and lower thresholds. This was essential to detecting rocks.

The mapping image was built using the following logic:

* **Free terrain** was found by thresholding the image based on the lower threshold of **(160,160,160)**
* **Occupied terrain** was the complement of the free terrain (or **1-freeTerrain**), except we ignored fully black pixels in the warped image. These correspond to areas outside the field of view of the rover, as the field of view is a cone but the image is a rectangle.
* **Rocks** were found by thresholding the image with a lower threshold of **(120,100,0)** and an upper threshold of **(200,200,50)**
* Pixels identified as rocks were also identified as free terrain

Finally, logic was added to ignore a measurement if the rover roll and pitch exceeded measured thresholds. This was useful because the perception algorithm assumes that pitch and roll are zero, so values far away can lead to loss of map fidelity. 

#### decision.py

The "normal" navigation code was modified so the throttle is not a constant. Instead, it is scaled based on the number of nearby navigable pixels. This is done in the `proportional_throttle()` function, which:

* Ignores free space pixels further than a maximum distance "max_dist" (I used 100 pixels)
* Linearly scales the throttle between 0 and "Rover.throttle_set" using the provided thresholds for stopping ("Rover.stop_forward") and going forward ("Rover.go_forward")

In addition, the steering logic was moved into a separate `steer_rover()` function. This function:

* Throttles the rover with the pre-computed throttle if the steering is within limits (15 degrees)
* Else, the throttle is set to zero (coast) and the rover is turned at its maximum rate

Besides "normal" navigation, a `follow_rock()` function was added. This is used to navigate if the rover detects any pixels related to a rock observation. This function:

* Steers the rover towards the rock by setting the steer angle to the mean of the angle observations
* Slows down the rover significantly such that it approaches the rock at a slow enough speed to pick it up
* The speed is scaled proportionally to the mean distance of all the rock pixels, so the closer it is the slower it is
* To prevent oscillations in throttle if the rock fades in and out of sight, the rover is set to "stop" mode if a rock if observed. Therefore, it takes multiple false rock readings to start moving at full speed again

Finally, logic was added at the end of the function to back up in a straight line if the pitch and roll exceed measured thresholds. This was useful for the rover to recover from driving into the sides of the cliffs or over smaller boulders.

![Autonomous Navigation and Mapping Snapshot][autonav]

---

