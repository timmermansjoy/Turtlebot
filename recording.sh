#!/bin/sh

# Set Session Name
SESSION="stern4most"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
    # Start New Session with our name
    tmux new-session -d -s $SESSION

    # start rommel
    tmux rename-window -t 0 'gazebo'
    tmux send-keys -t 'gazebo' 'rommel' C-m

    # start pilot system
    tmux new-window -t $SESSION:1 -n 'ControlCenter'
    tmux send-keys -t 'ControlCenter' 'rosrun stern4most_pilot_AI2 stern4most_pilot_AI2.py' C-m

    # start dashboard system
    tmux new-window -t $SESSION:2 -n 'dashboard'
    tmux send-keys -t 'dashboard' "rosrun stern4most_dashboard_AI2 stern4most_dashboard_AI2.py" C-m

    # start AI system
    tmux new-window -t $SESSION:5 -n 'AI'
    tmux send-keys -t 'AI' "rosrun stern4most_AI_AI2 stern4most_AI_Record_AI2.py" C-m

    tmux join-pane -v -s 2 -t 1
    tmux join-pane -h -s 5 -t 1


fi

# Attach Session, on the Main window
tmux attach-session -t $SESSION:1
