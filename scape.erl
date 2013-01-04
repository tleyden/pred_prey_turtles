
-module(scape).
-behavior(gen_server).
-export([start/0, stop/0]). 
-export([init/1, terminate/2, handle_call/3, handle_cast/2, handle_info/2, code_change/3]).
-export([topic_from_message/1]).
-include("pred_prey.hrl").

%% start both predator and prey and link to their processes
%% subscribe to both the predator and prey position and detect collisions
%% shut down predator and prey, then shutdown self

start() ->
    % gen_server:start({local, ?MODULE}, ?MODULE, undefined, []).
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
    {ok, {'/prey/pose', undefined, '/predator/pose', undefined}}.

handle_info(Message, State) ->
    io:format("handle_info called.  Message: ~p ~n", [Message]),
    %% TODO: collision detection -> stop simulation
    Topic = topic_from_message(Message),
    case Topic of 
	'/prey/pose' ->
	    io:format("prey pose~n"),
	    {'/prey/pose', _Ignored, '/predator/pose', PreviousPredatorPose} = State,
	    NewState = {'/prey/pose', Message, '/predator/pose', PreviousPredatorPose};
	'/predator/pose' ->
	    io:format("predator pose~n"),
	    {'/prey/pose', PreviousPreyPose, '/predator/pose', _Ignored} = State,
	    NewState = {'/prey/pose', PreviousPreyPose, '/predator/pose', Message};
	 _ ->
	    io:format("unknown topic ~p ~n", [Topic]),
	    NewState = State
    end,
    io:format("handle_info called.  NewState: ~p ~n", [NewState]),
    {noreply, NewState}.

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

topic_from_message(Message) ->
    {{_SenderNodeName, _SenderProcessName}, MessageBody} = Message,    
    {Topic,
     _TurtleXPosition, 
     _TurtleYPosition, 
     _TurtleTheta, 
     _TurtleLinearVelocity, 
     _TurtleAngularVelocity} = MessageBody,
    Topic.
