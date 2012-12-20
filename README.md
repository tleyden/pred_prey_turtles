
## Overview

Simulation of two turtlesim turtles.

- One turtle is designated as a predator, the other turtle is the prey.  
- The predator is trying to eat the prey, and the prey is trying not to be eaten.
- predator.erl controls the predator, prey.erl controls the prey
- The predator watches the position of the prey, and the prey watches the position of the predator


## TODO

- Figure out how to spawn two turtles in the same world!

  ans: $ rosservice call spawn 2 2 0.2 ""
       $ rostopic pub -1 /turtle1/command_velocity turtlesim/Velocity  -- 2.0  1.8
       $ rostopic pub -1 /turtle2/command_velocity turtlesim/Velocity  -- 2.0  1.8

- Create prey.erl, which just moves randomly (could put this in helloworld example actually)

- Create predator.erl, which also just moves randomly

- Create simulation_engine.erl which watches both predator.erl and prey.erl positions, and detect collisions.  If collision, kill prey object.

- Make predator.erl smarter so that it moves towards prey

- Make prey smarter, so that it evades predator


## Running

Start roscore:

    roscore

Start the ros turtle sim:

    rosrun turtlesim turtlesim_node

Start the erlang port mapper daemon:

    epmd -daemon 

Start the python script:

    ./pred_prey_rosbridge.py

Start the erlang shell:

    erl -sname predprey@localhost -setcookie pred_prey_erlang_cookie

