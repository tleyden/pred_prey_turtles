
## Overview

Simulation of two turtlesim turtles.

- One turtle is designated as a predator, the other turtle is the prey.  
- The predator is trying to eat the prey, and the prey is trying not to be eaten.
- predator.erl controls the predator, prey.erl controls the prey
- The predator watches the position of the prey, and the prey watches the position of the predator


## TODO

- Finish prey.erl

  - Needs to be able to subscribe to two topics: self pose and predator pose

  - Make it move randomly whenever it has detected to not be moving (just like hello_ros_erlang)

- Create predator.erl, which subscribes to same topics and also moves randomly

- Add simple collision detection to prey.erl, which kills itself when predator hits it

- Consider moving collision detection to a new module called scape.erl or something similar


Phase 2:

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

