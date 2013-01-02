-module(pred_prey).
-export([start_prey/0, start_predator/0, stop_prey/0, stop_predator/0]).
-include("pred_prey.hrl").

%% Predator or prey turtle which receives predator coordinates and moves randomly.  
%% Eventually will have avoidance behavior to avoid being eaten.

%% Public functions

start_prey() ->
    start(prey).

start_predator() ->
    start(predator).

stop_prey() ->
    stop(prey).

stop_predator() ->
    stop(predator).

%% Private functions

start(TurtleType) ->
    connect_to_remote_node(TurtleType).

stop(TurtleType) ->
    ReturnAddress = whereis(TurtleType),
    %% stops the whole bridge .. not what we want.  {?REMOTEMAILBOX,?REMOTENODE} ! {ReturnAddress, stop},
    {?REMOTEMAILBOX,?REMOTENODE} ! {ReturnAddress, kill, atom_to_list(TurtleType)},
    TurtleType ! stop,
    unregister(TurtleType).

spawn_turtle(TurtleType) ->
    case TurtleType of
	prey ->
	    TurtleSpawnTuple = {2, 2, ?TURTLE_START_THETA, atom_to_list(TurtleType)};
	predator ->
	    TurtleSpawnTuple = {10, 10, ?TURTLE_START_THETA, atom_to_list(TurtleType)}
    end,
    ReturnAddress = whereis(TurtleType),
    {?REMOTEMAILBOX,?REMOTENODE} ! {ReturnAddress, spawn, TurtleSpawnTuple}.

start_process(TurtleType) ->
    Pid = spawn(fun () -> loop(TurtleType) end),
    register(TurtleType, Pid).

remote_node_connected(TurtleType) ->
    start_process(TurtleType),
    spawn_turtle(TurtleType),
    ReturnAddress = whereis(TurtleType),
    rosbridge:subscribe_to_topic(ReturnAddress, ?TOPIC_PREDATOR_POSE), 
    rosbridge:subscribe_to_topic(ReturnAddress, ?TOPIC_PREY_POSE).

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
    ReturnAddress = whereis(TurtleType),
    {SenderProcessName, SenderNodeName} ! {ReturnAddress, command_velocity, TurtleType, {NewLinearVelocity, NewAngularVelocity}};

move_turtle_randomly(_, _, _, _, _) ->
    true.

is_other_turtle(TurtleType, Topic) ->
    case re:run(atom_to_list(Topic), atom_to_list(TurtleType)) of
	{match,_} ->
	    false;
	nomatch -> 
	    true
    end.

handle_turtle_pose_message(TurtleType, SenderNodeName, SenderProcessName, MessageBody) ->
    {Topic,
     _TurtleXPosition, 
     _TurtleYPosition, 
     _TurtleTheta, 
     TurtleLinearVelocity, 
     TurtleAngularVelocity} = MessageBody,
    io:format("Turtle: ~p Topic: ~p Sender: ~p Process: ~p~n", [TurtleType, Topic, SenderNodeName, SenderProcessName]),
    case is_other_turtle(TurtleType, Topic) of
	false ->
	    move_turtle_randomly(TurtleType, SenderNodeName, SenderProcessName, TurtleLinearVelocity, TurtleAngularVelocity);
	_ ->
	    noop
    end.
    

loop(TurtleType) ->
    receive 
	stop ->
	    io:format("~p received stop, process exiting~n", [TurtleType]);
	{ {SenderNodeName, SenderProcessName}, MessageBody } ->
	    handle_turtle_pose_message(TurtleType, SenderNodeName, SenderProcessName, MessageBody),
	    loop(TurtleType)
    end.
