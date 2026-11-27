from collections import deque

FREE = 'f'
BLOCKED = 'b'
ROBOT = 'r'
TARGET = 't'
UNKNOWN = '?'

# Células por onde o robô pode passar.
PASSABLE = {FREE, ROBOT, TARGET}

# Direção -> deslocamento (linha, coluna). A ordem define a ordem de expansão.
DIRECTIONS = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
}


def grid_from_flat(flat, shape):
    rows, cols = int(shape[0]), int(shape[1])
    if len(flat) != rows * cols:
        raise ValueError(f'Grid com {len(flat)} células não bate com {rows}x{cols}')
    return [list(flat[r * cols:(r + 1) * cols]) for r in range(rows)]


def find_cell(grid, value):
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == value:
                return (r, c)
    return None


def is_passable(grid, cell):
    r, c = cell
    return 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] in PASSABLE


def neighbors(grid, cell):
    r, c = cell
    for direction, (dr, dc) in DIRECTIONS.items():
        nxt = (r + dr, c + dc)
        if is_passable(grid, nxt):
            yield direction, nxt


def bfs(grid, start, goal):
    came_from = {start: None}  # célula -> célula de onde viemos
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        if cell == goal:
            return _rebuild_path(came_from, goal)
        for _, nxt in neighbors(grid, cell):
            if nxt not in came_from:
                came_from[nxt] = cell
                queue.append(nxt)
    return None


def _rebuild_path(came_from, goal):
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def path_to_moves(path):
    delta_to_dir = {delta: name for name, delta in DIRECTIONS.items()}
    moves = []
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        moves.append(delta_to_dir[(r2 - r1, c2 - c1)])
    return moves


def render(grid, path=()):
    symbols = {BLOCKED: '|', FREE: ' ', ROBOT: 'R', TARGET: 'T', UNKNOWN: '?'}
    on_path = set(path)
    lines = []
    for r, row in enumerate(grid):
        line = ''
        for c, cell in enumerate(row):
            if (r, c) in on_path and cell == FREE:
                line += '·'
            else:
                line += symbols.get(cell, cell)
        lines.append(line)
    return '\n'.join(lines)
