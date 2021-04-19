# AnR2021G02

<p align="center">
  <img width="500px" src="https://spectrum.ieee.org/image/MjI0MDU1NA.jpeg">
</p>

# Run

to run this project you have to get the [ROSNoeticDocker](https://github.com/PXLAIRobotics/ROSNoeticDocker.git)
clone this repo into the `/Projects/catkin_ws_src` directory

our AI package uses more dependencys so we also have to install this with pip. This is not installed so

```bash
sudo apt update && sudo apt install -y python3-pip
```

afterwards install the dependencys with

```
pip3 install -r requirements.txt
```

once inside run the following command to build the solution:

```bash
cd ~/Projects/catkin_ws && catkin_make --pkg referee && source devel/setup.bash
```

## Rommel

**RECOMMENDED** run the file `rommel.sh`

```bash
sh ~/Projects/catkin_ws/src/AnR2021G02/rommel.sh
```

alternative:

```bash
[terminal 1] $ rommel
[terminal 2] $ rosrun stern4most_pilot_AI2 stern4most_pilot_AI2.py
[terminal 3] $ rosrun stern4most_dashboard_AI2 stern4most_dashboard_AI2.py
[terminal 4] $ rosrun stern4most_lidar_AI2 stern4most_lidar_AI2.py
[terminal 5] $ rosrun stern4most_vision_AI2 stern4most_vision_AI2.py
[terminal 6] $ rosrun referee start_publisher.py
[terminal 7] $ rosrun referee referee_service.py
[terminal 8] $ rosrun stern4most_communication_AI2 stern4most_communication_AI2.py
```

## Backwards

**RECOMMENDED** run the file `backwards.sh`

```bash
sh ~/Projects/catkin_ws/src/AnR2021G02/backwards.sh
```

alternative:

```bash
[terminal 1] $ sternformost
[terminal 2] $ rosrun stern4most_pilot_AI2 stern4most_pilot_AI2.py
[terminal 3] $ rosrun stern4most_dashboard_AI2 stern4most_dashboard_AI2.py
[terminal 4] $ rosrun stern4most_vision_AI2 stern4most_vision_AI2.py
[terminal 5] $ rosrun referee start_publisher.py
[terminal 6] $ rosrun referee referee_service.py
[terminal 7] $ rosrun stern4most_communication_AI2 stern4most_communication_AI2.py
```

# Architecture

Our systems layout:

![Layout](img/Architecture.png)

all systems work independently of eachother. When all systems are engagde the LIDAR is the master of the vision.
It will first avoid an obstacle and the vision will correct afterwards to stay on track.

### Dashboard

![Layout](img/Dashboard.png)

# Known issues

- 15% chance the car will avoid a object by going off track
- 30% chance on getting stuck on tree

# Extras

## Mahcine learning

in the last 2 days we tried to get our hands dirty with machine learning. This because it didnt really looked that difficult
and wanted to try something different, [george hotz](https://youtu.be/ZtpWTJ7Jsh8) was a big insiration in the motivation.

Our machine learning was in 3 steps:

### Data collection

With data collection we created a module that collects our data. This data is the image seen by our turtlebot and the steering angle it currently has.
We can use this data to train. Our first dataset was made by driving from the start to the 6th sector of the track. We drove back and forward on this track to get 6000 images / datapoints

this was done in `stern4most_AI_AI2/stern4most_AI_Record_AI2.py`

### Training

With the dataset we created. we started to train our moddel with the help of some online references. We used: scikit-learn and tensorflow for this training.
At the end of our training we had a loss function of 0.03. in our first attempt.

![Layout](img/loss_funtion.png)

this was done in `training/training.py`

### Implimenting

With our trained moddel we implimented this almost the same way as our vision module, only we used our model to predict the steering angles. Our moddel was trained well enough on the small piece of the track
to drive the rest of the track flawlessly

---

### Things we are proud of

- Implimented LIDAR system with no online resources
- Impliment Machine learning
- Smoothness on cornering when driving backwards
- Smoothness on obstacle avoidance with LIDAR
- Decently documented code
- Mostly structured code

### Things we could have done better

- Better vision system
- LIDAR and vision working in symbiosis

---

### Full runs

#### Forwards rommel

<a href="https://youtu.be/CVHOaJUBfbA
" target="_blank"><img src="https://www.iconpacks.net/icons/2/free-youtube-logo-icon-2431-thumb.png"
alt="Full" width="150"/></a>

#### Backwards empty

<a href="https://youtu.be/0KUafmbeS6w
" target="_blank"><img src="https://www.iconpacks.net/icons/2/free-youtube-logo-icon-2431-thumb.png"
alt="Full" width="150"/></a>

#### Backwards rommel in under 5 min

<a href="https://youtu.be/dQw4w9WgXcQ
" target="_blank"><img src="https://www.iconpacks.net/icons/2/free-youtube-logo-icon-2431-thumb.png"
alt="Full" width="150"/></a>
