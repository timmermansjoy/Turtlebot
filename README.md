# AnR2021G02

<p align="center">
  <img width="500px" src="https://s3-eu-west-1.amazonaws.com/robosavvy/prodImages/turtlebot3/turtlebotPi.jpg">
</p>

# Run
to run this project you have to get the [ROSNoeticDocker](https://github.com/PXLAIRobotics/ROSNoeticDocker.git)
clone this repo into the `/Projects/catkin_ws_src` directory

once inside run the following command in different terminals:

For the normal track:
```bash
[terminal 1] $ sternformost
[terminal 2] $ rosrun stern4most_pilot_AI2 stern4most_pilot_AI2.py
[terminal 3] $ rosrun stern4most_dashboard_AI2 stern4most_dashboard_AI2.py
[terminal 4] $ rosrun stern4most_vision_AI2 stern4most_vision_AI2.py
[terminal 5] $ rosrun referee start_publisher.py
[terminal 6] $ rosrun referee referee_service.py
[terminal 7] $ rosrun stern4most_communication_AI2 stern4most_communication_AI2.py
```

For the rommel track:
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
