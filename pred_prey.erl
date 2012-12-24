-module(pred_prey).
-export([start/1, stop/1]).
-define(REMOTENODE,'pred_prey_rosbridge@localhost').
-define(REMOTEMAILBOX,pred_prey_erlang_mailbox).
-define(TOPIC_PREY_POSE,"/prey/pose").
-define(TOPIC_PREDATOR_POSE,"/predator/pose").
-define(TURTLE_START_THETA,0).

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

spawn_turtle(TurtleType) ->
    case TurtleType of
	prey ->
	    TurtleSpawnTuple = {2, 2, ?TURTLE_START_THETA, atom_to_list(TurtleType)};
	predator ->
	    TurtleSpawnTuple = {10, 10, ?TURTLE_START_THETA, atom_to_list(TurtleType)}
    end,
    {?REMOTEMAILBOX,?REMOTENODE} ! {whereis(TurtleType), spawn, TurtleSpawnTuple}.

start_process(TurtleType) ->
    Pid = spawn(fun () -> loop(TurtleType) end),
    register(TurtleType, Pid).

remote_node_connected(TurtleType) ->
    start_process(TurtleType),
    spawn_turtle(TurtleType),
    subscribe_to_topic(TurtleType, ?TOPIC_PREDATOR_POSE), 
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
	    io:format("~p received stop, process exiting~n", TurtleType);
	{ {SenderNodeName, SenderProcessName}, MessageBody } ->
	    handle_turtle_pose_message(TurtleType, SenderNodeName, SenderProcessName, MessageBody),
	    loop(TurtleType)
    end.
