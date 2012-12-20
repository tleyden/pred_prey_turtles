-module(prey).
-export([start/0, stop/0]).
-define(REMOTENODE,'pred_prey_rosbridge@localhost').
-define(REMOTEMAILBOX,hello_ros_erlang_mailbox).
-define(SELF_PROCESS,prey_process).
-define(TOPIC_SELF_POSE,"/prey/pose").
-define(TOPIC_PREDATOR_POSE,"/predator/pose").
-define(TURTLE_NAME,"prey").
-define(TURTLE_X,2).
-define(TURTLE_Y,2).
-define(TURTLE_THETA,0).

%% Prey turtle which receives predator coordinates and moves randomly.  
%% Eventually will have avoidance behavior to avoid being eaten.

start() ->
    connect_to_remote_node().

stop() ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {self(), stop},
    ?SELF_PROCESS ! stop,
    unregister(?SELF_PROCESS).

subscribe_to_topic(TopicName) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {self(), subscribe, TopicName}.

spawn_turtle(TurtleSpawnTuple) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {self(), spawn_turtle, TurtleSpawnTuple}.

start_process() ->
    Pid = spawn(fun loop/0),    
    register(?SELF_PROCESS, Pid).

remote_node_connected() ->
    start_process(),
    spawn_turtle({?TURTLE_NAME, ?TURTLE_X, ?TURTLE_Y, ?TURTLE_THETA}),
    subscribe_to_topic(?TOPIC_PREDATOR_POSE),  %% what if topic doesn't exist yet?
    subscribe_to_topic(?TOPIC_SELF_POSE).

connect_to_remote_node() ->
    %% in order to be able to receive messages from remote node, we must connect
    ConnectedRemoteNode = net_kernel:connect(?REMOTENODE),
    case ConnectedRemoteNode of
	true ->
	    remote_node_connected();
	false ->
	    io:format("Could not connect to ~w, is it running? ~n", [?REMOTENODE])
    end.

move_turtle_randomly(SenderNodeName, SenderProcessName, TurtleLinearVelocity, TurtleAngularVelocity) 
  when TurtleLinearVelocity < 0.01, TurtleAngularVelocity < 0.01 ->
    io:format("Turtle stopped moving, moving it~n"),
    random:seed(now()),
    NewLinearVelocity = random:uniform(5) - 2,
    NewAngularVelocity = random:uniform(5) - 2,
    {SenderProcessName, SenderNodeName} ! {self(), command_velocity, {NewLinearVelocity, NewAngularVelocity}};

move_turtle_randomly(_, _, _, _) ->
    true.

loop() ->
    receive 
	stop ->
	    io:format("Stopping loop~n");
	TurtleMessage ->
	    { {SenderNodeName, SenderProcessName}, TurtlePose } = TurtleMessage,
	    {TurtleXPosition, 
	     _TurtleYPosition, 
	     _TurtleTheta, 
	     TurtleLinearVelocity, 
	     TurtleAngularVelocity} = TurtlePose,
	    io:format("Sender Node Name: ~p Process Name: ~p~n", [SenderNodeName, SenderProcessName]),
	    TurtleXPositionString = io_lib:format("~.1f",[TurtleXPosition]),
	    io:format("Turtle X: ~p~n", TurtleXPositionString),
	    move_turtle_randomly(SenderNodeName, SenderProcessName, TurtleLinearVelocity, TurtleAngularVelocity),
	    loop()
    end.
