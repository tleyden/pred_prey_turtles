
## Overview

Simulation of two turtlesim turtles.

- One turtle is designated as a predator, the other turtle is the prey.  
- The predator is trying to eat the prey, and the prey is trying not to be eaten.
- predator.erl controls the predator, prey.erl controls the prey
- The predator watches the position of the prey, and the prey watches the position of the predator

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

In the erlang shell:

    erlang> scape:start()
    erlang> scape:stop()

to start and stop it

## Known issues

The ros turtle sim (rosrun turtlesim turtlesim_node) needs to be restarted each time the scape is restarted.

