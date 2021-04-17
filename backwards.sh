#!/bin/sh

# Set Session Name
SESSION="stern4most"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
    # Start New Session with our name
    tmux new-session -d -s $SESSION

    # start gazebo empty track
    tmux rename-window -t 0 'gazebo'
    tmux send-keys -t 'gazebo' 'sternformost' C-m

    # start pilot system
    tmux new-window -t $SESSION:1 -n 'ControlCenter'
    tmux send-keys -t 'ControlCenter' 'rosrun stern4most_pilot_AI2 stern4most_pilot_AI2.py' C-m

    # start dashboard system
    tmux new-window -t $SESSION:2 -n 'dashboard'
    tmux send-keys -t 'dashboard' "rosrun stern4most_dashboard_AI2 stern4most_dashboard_AI2.py" C-m

    # start vision system
    tmux new-window -t $SESSION:3 -n 'vision'
    tmux send-keys -t 'vision' "rosrun stern4most_vision_AI2 stern4most_vision_AI2.py" C-m

    # start LIDAR system
    tmux new-window -t $SESSION:4 -n 'LIDAR'
    tmux send-keys -t 'LIDAR' "rosrun stern4most_lidar_AI2 stern4most_lidar_AI2.py" C-m

    tmux join-pane -v -s 2 -t 1
    tmux join-pane -v -s 3 -t 1
    tmux join-pane -v -s 4 -t 1

    # start referee system
    tmux new-window -t $SESSION:2 -n 'referee'
    tmux send-keys -t 'referee' "rosrun referee referee_service.py" C-m

    tmux new-window -t $SESSION:3 -n 'communication'
    tmux send-keys -t 'communication' "rosrun stern4most_communication_AI2 stern4most_communication_AI2.py" C-m

    tmux join-pane -v -s 3 -t 2

fi

# Attach Session, on the Main window
tmux attach-session -t $SESSION:1