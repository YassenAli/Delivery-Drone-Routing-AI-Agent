% Define the initial grid
initial_grid([
    ['D', '-', 'P', '-', 'O'],
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
    ( (Cell == 'P', member((NewX, NewY), RemPs)) ->
        NewCollected is Collected + 1,
        select((NewX, NewY), RemPs, NewRemPs)
    ; NewCollected = Collected, NewRemPs = RemPs
    ),
    \+ member((NewX, NewY), Path),
    NextState = state((NewX, NewY), NewRemPs, [(NewX, NewY) | Path], NewCollected).

% BFS to find the path with maximum P's
bfs([], _, BestState, BestState).
bfs([State | Rest], Visited, CurrentBest, FinalBest) :-
    ( better(State, CurrentBest) -> NewBest = State ; NewBest = CurrentBest ),
    State = state((PosX, PosY), RemPs, _, _),
    ( member(((PosX, PosY), RemPs), Visited) ->
        bfs(Rest, Visited, NewBest, FinalBest)
    ;
        findall(NextState, next_state(State, NextState), NextStates),
        append(Rest, NextStates, NewQueue),
        bfs(NewQueue, [((PosX, PosY), RemPs) | Visited], NewBest, FinalBest)
    ).

better(state(_, _, _, C1), state(_, _, _, C2)) :- C1 > C2.

% Initialize state
initial_state(State) :-
    initial_grid(Grid),
    find_drone(Grid, (X, Y)),
    find_ps(Grid, Ps),
    sort(Ps, SortedPs),
    State = state((X, Y), SortedPs, [(X, Y)], 0).

% Generate the grid for a specific step in the path
generate_step_grid(InitialGrid, Path, StepIndex, GeneratedGrid) :-
    N is StepIndex + 1,
    positions_up_to(Path, N, Positions),
    nth0(StepIndex, Positions, CurrentPos),
    findall(
        NewRow,
        ( nth1(X, InitialGrid, InitialRow),
          findall(
              NewCell,
              ( nth1(Y, InitialRow, InitialCell),
                ( member((X,Y), Positions) ->
                    ( (X,Y) = CurrentPos -> NewCell = 'D'
                    ; NewCell = '*'
                    )
                ; NewCell = InitialCell
                )
              ),
              NewRow
          )
        ),
        GeneratedGrid
    ).

% Helper to get the first N elements of the path
positions_up_to(Path, N, Positions) :-
    length(Positions, N),
    append(Positions, _, Path).

% Print the grid in a readable format
print_grid(Grid) :-
    forall(member(Row, Grid),
           ( atomic_list_concat(Row, ' ', RowStr),
             write(RowStr), nl
           )).

% Main predicate: find the best path and print results
find_best_path(Path, Collected) :-
    initial_state(InitialState),
    bfs([InitialState], [], state((0,0), [], [], -1), BestState),
    BestState = state(_, _, RevPath, Collected),
    reverse(RevPath, Path),
    print_steps(Path).

% Print the initial grid, intermediate steps, and final grid
print_steps(Path) :-
    initial_grid(InitialGrid),
    write('Drone Route:'), nl,
    print_grid(InitialGrid), nl,
    write('Steps:'), nl,
    length(Path, PathLen),
    ( PathLen > 1 ->
        MaxStep is PathLen - 2,
        forall(between(0, MaxStep, StepIndex),
               ( generate_step_grid(InitialGrid, Path, StepIndex, Grid),
                 print_grid(Grid), nl
               ))
    ; true
    ),
    write('Final:'), nl,
    ( PathLen >= 1 ->
        LastStepIndex is PathLen - 1,
        generate_step_grid(InitialGrid, Path, LastStepIndex, FinalGrid)
    ; FinalGrid = InitialGrid
    ),
    print_grid(FinalGrid).

% Example usage:
% ?- find_best_path(Path, Collected).
