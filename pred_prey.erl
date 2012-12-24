-module(pred_prey).
-export([start/1, stop/1]).
-define(REMOTENODE,'pred_prey_rosbridge@localhost').
-define(REMOTEMAILBOX,pred_prey_erlang_mailbox).
-define(TOPIC_PREY_POSE,"/prey/pose").
-define(TOPIC_PREDATOR_POSE,"/predator/pose").
-define(TURTLE_X,5).
-define(TURTLE_Y,5).
-define(TURTLE_THETA,0).

%% Predator or prey turtle which receives predator coordinates and moves randomly.  
%% Eventually will have avoidance behavior to avoid being eaten.

start(TurtleType) ->
    connect_to_remote_node(TurtleType).

stop(TurtleType) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {whereis(TurtleType), stop},
    TurtleType ! stop,
    unregister(TurtleType).

subscribe_to_topic(TurtleType, TopicName) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {whereis(TurtleType), subscribe, TopicName}.

spawn_turtle(TurtleType, TurtleSpawnTuple) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {whereis(TurtleType), spawn, TurtleSpawnTuple}.

start_process(TurtleType) ->
    Pid = spawn(fun () -> loop(TurtleType) end),
    register(TurtleType, Pid).

remote_node_connected(TurtleType) ->
    start_process(TurtleType),
    spawn_turtle(TurtleType, {?TURTLE_X, ?TURTLE_Y, ?TURTLE_THETA, atom_to_list(TurtleType)}),
    subscribe_to_topic(TurtleType, ?TOPIC_PREDATOR_POSE),  %% what if topic doesn't exist yet?
    subscribe_to_topic(TurtleType, ?TOPIC_PREY_POSE).

connect_to_remote_node(TurtleType) ->
    %% in order to be able to receive messages from remote node, we must connect
    ConnectedRemoteNode = net_kernel:connect(?REMOTENODE),
    case ConnectedRemoteNode of
	true ->
	    remote_node_connected(TurtleType);
	false ->
	    io:format("Could not connect to ~w, is it running? ~n", [?REMOTENODE])
    end.

move_turtle_randomly(TurtleType, SenderNodeName, SenderProcessName, TurtleLinearVelocity, TurtleAngularVelocity) 
  when TurtleLinearVelocity < 0.01, TurtleAngularVelocity < 0.01 ->
    io:format("Turtle stopped moving, moving it~n"),
    random:seed(now()),
    NewLinearVelocity = random:uniform(5) - 2,
    NewAngularVelocity = random:uniform(5) - 2,
    {SenderProcessName, SenderNodeName} ! {self(), command_velocity, TurtleType, {NewLinearVelocity, NewAngularVelocity}};

move_turtle_randomly(_, _, _, _, _) ->
    true.

loop(TurtleType) ->
    receive 
	stop ->
	    io:format("~p received stop, process exiting~n", TurtleType);
	TurtleMessage ->
	    { {SenderNodeName, SenderProcessName}, MessageBody } = TurtleMessage,
	    {Topic,
	     TurtleXPosition, 
	     _TurtleYPosition, 
	     _TurtleTheta, 
	     TurtleLinearVelocity, 
	     TurtleAngularVelocity} = MessageBody,
	    io:format("Turtle: ~p Topic: ~p Sender: ~p Process: ~p~n", [TurtleType, Topic, SenderNodeName, SenderProcessName]),
	    TurtleXPositionString = io_lib:format("~.1f",[TurtleXPosition]),
	    io:format("Turtle X: ~p~n", TurtleXPositionString),
	    move_turtle_randomly(TurtleType, SenderNodeName, SenderProcessName, TurtleLinearVelocity, TurtleAngularVelocity),
	    loop(TurtleType)
    end.
