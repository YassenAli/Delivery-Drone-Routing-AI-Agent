% Generic Prolog delivery planner with dynamic MxN city grid

% Entry point: supply Grid as a list of lists of chars (MxN)
% Example invocation:
% ?- Grid = [ ['D','-','P'], ['-','O','-'], ['P','-','-'] ], find_best_path(Grid, Path, Collected).
% Grid = [['D','-','P','-','O'],['-','O','-','-','P'],['-','-','O','P','-'],['P','O','-','-','-'],['-','-','P','O','-']], find_best_path(Grid, Path, Collected).

% Find the drone's starting position
find_drone(Grid, (X, Y)) :-
    nth1(X, Grid, Row),
    nth1(Y, Row, 'D').

% Collect all delivery points
find_ps(Grid, Ps) :-
    findall((X, Y), (nth1(X, Grid, Row), nth1(Y, Row, 'P')), Ps).

% Four-directional moves
move(up,    (X, Y), (X1, Y)) :- X1 is X - 1.
move(down,  (X, Y), (X1, Y)) :- X1 is X + 1.
move(left,  (X, Y), (X, Y1)) :- Y1 is Y - 1.
move(right, (X, Y), (X, Y1)) :- Y1 is Y + 1.

% Determine grid dimensions
grid_size(Grid, Rows, Cols) :-
    length(Grid, Rows),
    ( Rows > 0 -> nth1(1, Grid, First), length(First, Cols) ; Cols = 0 ).

% Check if a position is inside the grid
within_bounds(Grid, X, Y) :-
    grid_size(Grid, Rows, Cols),
    X >= 1, X =< Rows,
    Y >= 1, Y =< Cols.

% Read a cell's content
grid_cell(Grid, X, Y, C) :-
    nth1(X, Grid, Row), nth1(Y, Row, C).

% Generate successor states
next_state(Grid, state(Pos, Rem, Path, Cnt),
           state(NewPos, NewRem, [NewPos|Path], NewCnt)) :-
    Pos = (X, Y),
    move(_, (X, Y), (NX, NY)),
    within_bounds(Grid, NX, NY),
    grid_cell(Grid, NX, NY, Cell),
    Cell \= 'O',
    ( Cell == 'P', select((NX, NY), Rem, NewRem) -> NewCnt is Cnt + 1
    ; NewRem = Rem, NewCnt = Cnt ),
    NewPos = (NX, NY),
    \+ member(NewPos, Path).

% Prefer more-collected states
better(state(_,_,_,C1), state(_,_,_,C2)) :- C1 > C2.

% BFS to find max collection
bfs(_, [], Best, Best).
bfs(Grid, [S|Qs], CurBest, Final) :-
    ( better(S, CurBest) -> NB = S ; NB = CurBest ),
    S = state(Pos, Rem, _, _),
    ( member((Pos, Rem), Qs) ->
        bfs(Grid, Qs, NB, Final)
    ;   findall(NX, next_state(Grid, S, NX), Ns),
        append(Qs, Ns, NewQs),
        bfs(Grid, NewQs, NB, Final)
    ).

% Initialize search state
initial_state(Grid, state(Start, SortedPs, [Start], 0)) :-
    find_drone(Grid, Start),
    find_ps(Grid, Ps), sort(Ps, SortedPs).

% Main: solve and print
find_best_path(Grid, Path, Collected) :-
    initial_state(Grid, Init),
    bfs(Grid, [Init], Init, state(_,_,Rev,Collected)),
    reverse(Rev, Path),
    print_steps(Grid, Path).

% Print initial, each step, and final grids
print_steps(Grid, Path) :-
    write('Initial Grid:'), nl, print_grid(Grid), nl,
    write('Steps:'), nl,
    length(Path, L),
    ( L > 1 ->
        Max is L - 2,
        forall(between(0, Max, I),
               ( generate_step_grid(Grid, Path, I, G), print_grid(G), nl ))
    ; true ),
    write('Final Grid:'), nl,
    generate_step_grid(Grid, Path, L-1, FG), print_grid(FG).

% Build the grid at step I (0-based)
generate_step_grid(Grid, Path, I, NewGrid) :-
    N1 is I + 1,
    positions_up_to(Path, N1, Visited),
    nth1(N1, Visited, DronePos),
    findall(Row2,
        ( nth1(X, Grid, Row1),
          findall(Cell2,
              ( nth1(Y, Row1, C1),
                ( member((X,Y), Visited)
                  -> ( (X,Y)==DronePos -> Cell2='D' ; Cell2='*' )
                  ; Cell2=C1 )
              ), Row2
          )
        ), NewGrid).

% Take first N elements of a list
positions_up_to(List, N, Prefix) :-
    length(Prefix, N), append(Prefix, _, List).

% Utility: print any grid
print_grid(G) :-
    forall(member(R, G), (atomic_list_concat(R, ' ', S), writeln(S))).
