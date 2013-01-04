
-module(scape).
-behavior(gen_server).
-export([start/0, stop/0]). 
-export([init/1, terminate/2, handle_call/3, handle_cast/2, handle_info/2, code_change/3]).
-export([euclidean_distance/2]).
-include("pred_prey.hrl").

%% start both predator and prey
%% subscribe to both the predator and prey position and detect collisions
%% shut down predator and prey, then shutdown self

start() ->
    gen_server:start({local, ?MODULE}, ?MODULE, [], []).

stop() ->
    gen_server:cast(?MODULE, terminate).

%% Callback

init([]) ->
    io:format("init called~n"),
    SuccessFunction = fun () -> pred_prey:start_prey(),
				pred_prey:start_predator(),				
				ReturnAddress = whereis(?MODULE),
				rosbridge:subscribe_to_topic(ReturnAddress, ?TOPIC_PREDATOR_POSE), 
				rosbridge:subscribe_to_topic(ReturnAddress, ?TOPIC_PREY_POSE)
		      end,
    FailureFunction = fun () -> 
			      io:format("Could not connect to ~w, is it running? ~n", [?REMOTENODE]),
			      stop()
		      end,
    rosbridge:connect(SuccessFunction, FailureFunction),
    InitialState = {'/prey/pose', undefined, '/predator/pose', undefined},
    {ok, InitialState}.

handle_info(Message, State) ->
    NewState = new_state_from_message(Message, State),
    CollisionDetected = detect_collision(NewState),
    case CollisionDetected of
	true ->
	    {stop, normal, NewState};
	_ ->
	    {noreply, NewState}
    end.

handle_call(_Request, _From, State) ->
    io:format("handle_call called.  State: ~p From: ~p ~n", [State,_From]),
    {reply, helloworld, State}.

handle_cast(stop, State) ->
    {stop, normal, State}.

terminate(_Reason,_State) ->
    io:format("terminate called.  State: ~p Reason: ~p ~n", [_State,_Reason]),
    pred_prey:stop_prey(),
    pred_prey:stop_predator(),
    ok.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

   
%% Private functions

new_state_from_message(Message, State) ->
    Topic = topic_from_message(Message),
    case Topic of 
	'/prey/pose' ->
	    {'/prey/pose', _Ignored, '/predator/pose', PreviousPredatorPose} = State,
	    NewState = {'/prey/pose', Message, '/predator/pose', PreviousPredatorPose};
	'/predator/pose' ->
	    {'/prey/pose', PreviousPreyPose, '/predator/pose', _Ignored} = State,
	    NewState = {'/prey/pose', PreviousPreyPose, '/predator/pose', Message};
	 _ ->
	    io:format("unknown topic ~p ~n", [Topic]),
	    NewState = State
    end,
    NewState.

detect_collision(State) ->
    case State of
	{'/prey/pose', undefined, '/predator/pose', undefined} ->
	    false;
	{'/prey/pose', undefined, '/predator/pose', _Value} -> 
	    false;
	{'/prey/pose', _Value, '/predator/pose', undefined} -> 
	    false;
	{'/prey/pose', LastPreyMessage, '/predator/pose', LastPredatorMessage} ->
	    detect_collision(LastPreyMessage, LastPredatorMessage)
    end.

detect_collision(LastPreyMessage, LastPredatorMessage) ->
    PreyXYPosition = xy_position_from_message(LastPreyMessage),
    PredatorXYPosition = xy_position_from_message(LastPredatorMessage),
    EuclideanDistance = euclidean_distance(PreyXYPosition, PredatorXYPosition),
    case EuclideanDistance of 
	N when N < 1.0 ->
	    true;
	_ ->
	    false
    end.

euclidean_distance({X1, Y1}, {X2, Y2}) ->
    math:sqrt(math:pow(X2 - X1, 2) + math:pow(Y2 - Y1, 2)).

xy_position_from_message(Message) ->
    {{_SenderNodeName, _SenderProcessName}, MessageBody} = Message,    
    {_Topic,
     TurtleXPosition, 
     TurtleYPosition, 
     _TurtleTheta, 
     _TurtleLinearVelocity, 
     _TurtleAngularVelocity} = MessageBody,
    {TurtleXPosition, TurtleYPosition}.

topic_from_message(Message) ->
    {{_SenderNodeName, _SenderProcessName}, MessageBody} = Message,    
    {Topic,
     _TurtleXPosition, 
     _TurtleYPosition, 
     _TurtleTheta, 
     _TurtleLinearVelocity, 
     _TurtleAngularVelocity} = MessageBody,
    Topic.
