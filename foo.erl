-module(foo).
-define(TEST,"testval").
-export([test/0]).

test() ->
    io:format("val: ~p ~n", [?TEST]).
