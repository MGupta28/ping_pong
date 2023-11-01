# ping_pong game

To play the game, clone the repository to your workspace, execute the command

```bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Now, run `roscore`, `turtlesim_node` and the script to play the game
```bash
roscore
```
```bash
rosrun turtlesim turtlesim_node
```
```bash
rosrun ping_pong_game ping_pong_game.py
```

> Use the arrow keys and 'w' and 's' keys to control the paddle turtles
