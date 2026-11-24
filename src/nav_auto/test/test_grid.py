from nav_auto.grid import bfs, find_cell, grid_from_flat, path_to_moves

# 'r' robô, 't' alvo, 'b' parede, 'f' livre
MAZE = [
    'bbbbbbb',
    'brfffbb',
    'bfbbfbb',
    'bfffftb',
    'bbbbbbb',
]


def make_grid(rows):
    return [list(row) for row in rows]


def test_grid_from_flat_rebuilds_rows():
    flat = [c for row in MAZE for c in row]
    assert grid_from_flat(flat, [5, 7]) == make_grid(MAZE)


def test_bfs_finds_shortest_path():
    grid = make_grid(MAZE)
    start, goal = find_cell(grid, 'r'), find_cell(grid, 't')
    path = bfs(grid, start, goal)
    # Os dois caminhos possíveis têm 6 movimentos; a BFS deve achar um deles.
    assert path[0] == start and path[-1] == goal
    assert len(path) - 1 == 6


def test_path_to_moves():
    path = [(1, 1), (2, 1), (3, 1), (3, 2)]
    assert path_to_moves(path) == ['down', 'down', 'right']


def test_bfs_returns_none_when_unreachable():
    grid = make_grid([
        'bbbbb',
        'brbtb',
        'bbbbb',
    ])
    assert bfs(grid, (1, 1), (1, 3)) is None


def test_bfs_does_not_walk_through_walls():
    grid = make_grid(MAZE)
    path = bfs(grid, find_cell(grid, 'r'), find_cell(grid, 't'))
    assert all(grid[r][c] != 'b' for r, c in path)
