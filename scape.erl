
-module(scape).
-behavior(gen_server).
-export([start/0, stop/0]). 
-export([init/1, terminate/2, handle_call/3, handle_cast/2, handle_info/2, code_change/3]).

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
    pred_prey:start_prey(),
    pred_prey:start_predator(),
    io:format("init called~n"),
    {ok, []}.

handle_info(Message, State) ->
    io:format("handle_info called.  Message: ~p ~n", [Message]),
    {noreply, State}.

handle_call(_Request, _From, State) ->
    io:format("changenotiifer handle_call called.  State: ~p From: ~p ~n", [State,_From]),
    {reply, helloworld, State}.

handle_cast(stop, State) ->
    pred_prey:stop_prey(),
    pred_prey:stop_predator(),
    {stop, normal, State}.

terminate(_Reason,_State) ->
    ok.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

    
