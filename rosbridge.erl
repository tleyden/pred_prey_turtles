-module(rosbridge).
-export([subscribe_to_topic/2]).
-include("pred_prey.hrl").

subscribe_to_topic(ReturnAddress, TopicName) ->
    {?REMOTEMAILBOX,?REMOTENODE} ! {ReturnAddress, subscribe, TopicName}.
