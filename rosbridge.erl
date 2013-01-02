-module(rosbridge).
-export([subscribe_to_topic/2, connect/2]).
-include("pred_prey.hrl").

subscribe_to_topic(ReturnAddress, TopicName) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {ReturnAddress, subscribe, TopicName}.

connect(SuccessFunction, FailureFunction) ->
    %% in order to be able to receive messages from remote node, we must connect
    ConnectedRemoteNode = net_kernel:connect(?REMOTENODE),
    case ConnectedRemoteNode of
	true ->
	    io:format("Connected to ~w ~n", [?REMOTENODE]),
	    SuccessFunction();
	false ->
	    FailureFunction()
    end.
