% Define the initial grid
initial_grid([
    ['D', '-', 'P', '-', 'O'],
    ['-', 'O', '-', '-', 'P'],
    ['_', '-', 'O', 'P', '-'],
    ['P', 'O', '-', '-', '-'],
    ['-', '-', 'P', 'O', '_']
]).

% Find the drone's initial position
find_drone(Grid, (X, Y)) :-
    nth1(X, Grid, Row),
    nth1(Y, Row, 'D').

% Collect all initial delivery points (P's)
find_ps(Grid, Ps) :-
    findall((X, Y), (nth1(X, Grid, Row), nth1(Y, Row, 'P')), Ps).

% Grid size
grid_size(Rows, Cols) :-
    initial_grid(Grid),
    length(Grid, Rows),
    ( Rows > 0 -> nth1(1, Grid, FirstRow), length(FirstRow, Cols) ; Cols = 0 ).

% Movement directions (up, down, left, right)
move(up, (X, Y), (X1, Y)) :-
    X1 is X - 1, X1 >= 1.
move(down, (X, Y), (X1, Y)) :-
    grid_size(Rows, _),
    X1 is X + 1, X1 =< Rows.
move(left, (X, Y), (X, Y1)) :-
    Y1 is Y - 1, Y1 >= 1.
move(right, (X, Y), (X, Y1)) :-
    grid_size(_, Cols),
    Y1 is Y + 1, Y1 =< Cols.

% Ensure a position is within bounds
within_bounds(X, Y) :-
    grid_size(Rows, Cols),
    X >= 1, X =< Rows,
    Y >= 1, Y =< Cols.

% Get cell value from the grid
grid_cell(X, Y, Cell) :-
    initial_grid(Grid),
    nth1(X, Grid, Row),
    nth1(Y, Row, Cell).

% Generate valid next states
next_state(state((X, Y), RemPs, Path, Collected), state((NewX, NewY), NewRemPs, [(NewX, NewY) | Path], NewCollected)) :-
    move(_, (X, Y), (NewX, NewY)),
    within_bounds(NewX, NewY),
    grid_cell(NewX, NewY, Cell),
    Cell \= 'O',
    \+ member((NewX, NewY), Path),
    ( Cell == 'P', member((NewX, NewY), RemPs)
      -> NewCollected is Collected + 1,
         select((NewX, NewY), RemPs, NewRemPs)
      ;  NewCollected = Collected,
         NewRemPs = RemPs
    ).

% Compare states based on number of collected packages
better(state(_, _, _, C1), state(_, _, _, C2)) :-
    C1 > C2.

% Breadth-first search
bfs([], _, BestState, BestState).
bfs([State | Rest], Visited, CurrentBest, FinalBest) :-
    State = state(Pos, RemPs, _, _),
    ( better(State, CurrentBest) -> NewBest = State ; NewBest = CurrentBest ),
    ( member((Pos, RemPs), Visited)
      -> bfs(Rest, Visited, NewBest, FinalBest)
      ;  findall(NextState, next_state(State, NextState), NextStates),
         append(Rest, NextStates, NewQueue),
         bfs(NewQueue, [(Pos, RemPs) | Visited], NewBest, FinalBest)
    ).

% Define initial state
initial_state(State) :-
    initial_grid(Grid),
    find_drone(Grid, (X, Y)),
    find_ps(Grid, Ps),
    sort(Ps, SortedPs),
    State = state((X, Y), SortedPs, [(X, Y)], 0).

% Find best path using BFS and print
find_best_path(Path, Collected) :-
    initial_state(InitialState),
    bfs([InitialState], [], state((1,1), [], [], -1), BestState),
    BestState = state(_, _, RevPath, Collected),
    reverse(RevPath, Path),
    print_steps(Path).

% Sublist for animation steps
sublist(List, N, SubList) :-
    length(SubList, N),
    append(SubList, _, List).

% Print the grid
print_grid(Grid) :-
    forall(member(Row, Grid),
           ( atomic_list_concat(Row, ' ', RowStr),
             write(RowStr), nl )),
    write('--------------------'), nl.

% Generate grid with visited path and drone
generate_step_grid(InitialGrid, Path, StepIndex, GeneratedGrid) :-
    N is StepIndex + 1,
    sublist(Path, N, SubPath),
    ( SubPath = []
      -> GeneratedGrid = InitialGrid
      ;  reverse(SubPath, [CurrentPos | VisitedPositions]),
         findall(
             Row,
             ( nth1(X, InitialGrid, InitialRow),
               findall(
                   Cell,
                   ( nth1(Y, InitialRow, InitialCell),
                     ( (X,Y) = CurrentPos
                       -> Cell = 'D'
                       ; member((X,Y), VisitedPositions)
                       -> Cell = '*'
                       ;  Cell = InitialCell
                     )
                   ),
                   Row
               )
             ),
             GeneratedGrid
         )
    ).

% Print all steps of the route
print_steps(Path) :-
    initial_grid(InitialGrid),
    write('Drone Route:'), nl, nl,
    print_grid(InitialGrid), nl,
    write('Steps:'), nl, nl,
    length(Path, PathLen),
    ( PathLen > 0
      -> Steps is PathLen - 1,
         forall(between(0, Steps, StepIndex),
                ( generate_step_grid(InitialGrid, Path, StepIndex, Grid),
                  print_grid(Grid), nl ))
      ; true
    ),
    write('Final:'), nl, nl,
    generate_step_grid(InitialGrid, Path, PathLen, FinalGrid),
    print_grid(FinalGrid).
