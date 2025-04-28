% Define the initial grid
initial_grid([
    ['D', 'P', '-', 'O'],
    ['-', 'O', '-', '-', 'P'],
    ['-', '-', 'O', 'P', '-'],
    ['P', 'O', '-', '-', '-'],
    ['-', '-', 'P', 'O', '-']
]).

% Find the drone's initial position
find_drone(Grid, (X, Y)) :-
    nth1(X, Grid, Row),
    nth1(Y, Row, 'D').

% Collect all initial delivery points (P's)
find_ps(Grid, Ps) :-
    findall((X, Y), (nth1(X, Grid, Row), nth1(Y, Row, 'P')), Ps).

% Movement directions (up, down, left, right)
move(up, (X, Y), (X1, Y)) :- X1 is X - 1, X1 >= 1.
move(down, (X, Y), (X1, Y)) :- X1 is X + 1.
move(left, (X, Y), (X, Y1)) :- Y1 is Y - 1, Y1 >= 1.
move(right, (X, Y), (X, Y1)) :- Y1 is Y + 1.

% Validate grid bounds dynamically
grid_size(Rows, Cols) :-
    initial_grid(Grid),
    length(Grid, Rows),
    ( Rows > 0 -> nth1(1, Grid, FirstRow), length(FirstRow, Cols) ; Cols = 0 ).

within_bounds(X, Y) :-
    grid_size(Rows, Cols),
    X >= 1, X =< Rows,
    Y >= 1, Y =< Cols.

% Get cell value
grid_cell(X, Y, Cell) :-
    initial_grid(Grid),
    nth1(X, Grid, Row),
    nth1(Y, Row, Cell).

% Generate valid next states
next_state(State, NextState) :-
    State = state((X, Y), RemPs, Path, Collected),
    move(_Dir, (X, Y), (NewX, NewY)),
    within_bounds(NewX, NewY),
    grid_cell(NewX, NewY, Cell),
    Cell \= 'O',
    ( Cell == 'P', member((NewX, NewY), RemPs) ->
        NewCollected is Collected + 1,
        select((NewX, NewY), RemPs, NewRemPs)
    ; NewCollected = Collected, NewRemPs = RemPs
    ),
    \+ member((NewX, NewY), Path),  % Prevent revisiting cells
    NextState = state((NewX, NewY), NewRemPs, [(NewX, NewY) | Path], NewCollected).

% BFS to find the path with maximum P's
bfs(Queue, BestState, BestState) :- Queue = [].
bfs([State | Rest], CurrentBest, FinalBest) :-
    (State = state(_, _, _, C), (CurrentBest = state(_, _, _, BC) ->
        (C > BC -> NewBest = State ; NewBest = CurrentBest)
    ; NewBest = State
    ),
    findall(NextState, next_state(State, NextState), NextStates),
    append(Rest, NextStates, NewQueue),
    bfs(NewQueue, NewBest, FinalBest).

% Initialize state
initial_state(State) :-
    initial_grid(Grid),
    find_drone(Grid, (X, Y)),
    find_ps(Grid, Ps),
    State = state((X, Y), Ps, [(X, Y)], 0).

% Main predicate
find_best_path(Path, Collected) :-
    initial_state(InitialState),
    bfs([InitialState], state((0,0), [], [], -1), BestState),
    BestState = state(_, _, RevPath, Collected),
    reverse(RevPath, Path).

% Example usage:
% ?- find_best_path(Path, Collected).