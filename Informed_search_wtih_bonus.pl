% Solve predicate - entry point for the A* search
solve(Grid, Rows, Cols, MaxEnergy) :-
    length(Grid, N),              % Use built-in length predicate
    N =:= Rows * Cols,            % Validate grid dimensions
    heuristicFunction(Grid, 0, H0), % Renamed from countPackage
    G0 = 0,                       % Initial cost
    F0 is G0 + H0,                % Initial f-score
    E0 = MaxEnergy,               % Initial energy
    astar([state(Grid, null, G0, H0, F0, E0)], [], Cols, N, MaxEnergy),
    !.                            % Cut to stop at first solution

% A* search algorithm implementation
astar([], _, _, _, _) :-
    write("No path found."), nl, !.
astar(Open, Closed, Cols, N, MaxEnergy) :-
    getBestState(Open, state(State, Parent, G, H, F, E), RestOpen),
    (
        \+ member(p, State)
    ->  % Goal reached - no packages left
        write("Cost (Number of moves): "), write(G), nl
    ;   % Continue search
        findall(
            state(Child, State, G1, H1, F1, E1),
            (
                move(State, Child, _, Cols, N, E, E1, MaxEnergy),
                \+ in_closed(Child, E1, Closed),
                G1 is G + 1,
                heuristicFunction(Child, 0, H1),  % Renamed from countPackage
                F1 is G1 + H1
            ),
            Children
        ),
        add_children(Children, RestOpen, Open1),
        append(Closed, [state(State, Parent, G, H, F, E)], Closed1),
        astar(Open1, Closed1, Cols, N, MaxEnergy)
    ).

% Get the state with the lowest F value from the open list
getBestState(Open, Best, Rest) :-
    findMin(Open, Best),
    delete(Open, Best, Rest).

% Find the state with minimum F value
findMin([X], X) :- !.
findMin([X, Y | T], Min) :-
    X = state(_, _, _, _, Fx, _),
    Y = state(_, _, _, _, Fy, _),
    (   Fx =< Fy
    ->  findMin([X | T], Min)
    ;   findMin([Y | T], Min)
    ).

% Add children to the open list with proper ordering
add_children([], Open, Open).
add_children([state(S, P, G, H, F, E) | T], Open, Out) :-
    (   select(state(S, _, _, _, Fold, E), Open, OpenR), 
        Fold > F
    ->  % Replace if same state exists with higher F value
        add_children(T, [state(S, P, G, H, F, E) | OpenR], Out)
    ;   member(state(S, _, _, _, Fold2, E), Open), 
        Fold2 =< F
    ->  % Skip if same state exists with lower/equal F value
        add_children(T, Open, Out)
    ;   % Add new state otherwise
        add_children(T, [state(S, P, G, H, F, E) | Open], Out)
    ).

% Check if a state is in the closed list
in_closed(S, E, Closed) :-
    member(state(S, _, _, _, _, E), Closed).

% Heuristic function: count remaining packages (renamed from countPackage)
heuristicFunction([], C, C).
heuristicFunction([p | T], Acc, Tot) :-
    Acc1 is Acc + 1,
    heuristicFunction(T, Acc1, Tot).
heuristicFunction([X | T], Acc, Tot) :-
    X \= p,
    heuristicFunction(T, Acc, Tot).

% Generate valid moves for the drone
move(State, NewState, Took, Cols, N, E, E1, MaxEnergy) :-
    E > 0,                           % Ensure drone has energy
    nth0(I, State, d),               % Find drone position
    % Try each direction and get new position
    (   applyMove(down, I, J, Cols, N)
    ;   applyMove(left, I, J, Cols, _)
    ;   applyMove(right, I, J, Cols, _)
    ;   applyMove(up, I, J, Cols, _)
    ),
    validIndex(J, N),                % Validate position is in bounds
    nth0(J, State, Cell),            % Get cell at new position
    Cell \= o,                       % Ensure not obstacle
    ETemp is E - 1,                  % Decrease energy for move
    ( Cell = r -> E1 = MaxEnergy ; E1 = ETemp ),  % Recharge if cell is 'r'
    ( Cell = p -> Took = 1 ; Took = 0 ),          % Record if package taken
    substitute(I, State, '*', Temp), % Mark old position as visited
    substitute(J, Temp, d, NewState).% Place drone at new position

% Movement direction helpers with clear naming
applyMove(down, I, J, Cols, N) :-    J is I + Cols,    I < N - Cols.
applyMove(left, I, J, Cols, _) :-    J is I - 1,       I mod Cols =\= 0.
applyMove(right, I, J, Cols, _) :-   J is I + 1,       I mod Cols =\= Cols - 1.
applyMove(up, I, J, Cols, _) :-      J is I - Cols,    I >= Cols.

% Check if index is within grid bounds
validIndex(I, N) :- I >= 0, I < N.

% Replace element at index I in list with value V
substitute(0, [_|T], V, [V|T]).
substitute(I, [H|T], V, [H|R]) :-
    I > 0,
    I1 is I - 1,
    substitute(I1, T, V, R).