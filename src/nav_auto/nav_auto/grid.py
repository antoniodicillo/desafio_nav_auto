"""Funções de grafo sobre o grid do labirinto (sem dependência do ROS).

O labirinto é uma matriz de células. Cada célula livre é um vértice do grafo,
e existe uma aresta entre duas células livres vizinhas (cima, baixo, esquerda,
direita). Todas as arestas têm o mesmo custo (1 movimento), então a busca em
largura (BFS) encontra o caminho com o menor número de movimentos.

Códigos das células (os mesmos usados pelo simulador culling_games):
    'f' livre, 'b' bloqueada (parede), 'r' robô, 't' alvo.
    '?' é usado na parte 2 para células ainda desconhecidas.
"""

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
    """Reconstrói a matriz a partir da lista achatada (linha a linha)."""
    rows, cols = int(shape[0]), int(shape[1])
    if len(flat) != rows * cols:
        raise ValueError(f'Grid com {len(flat)} células não bate com {rows}x{cols}')
    return [list(flat[r * cols:(r + 1) * cols]) for r in range(rows)]


def find_cell(grid, value):
    """Retorna (linha, coluna) da primeira célula com `value`, ou None."""
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == value:
                return (r, c)
    return None


def is_passable(grid, cell):
    r, c = cell
    return 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] in PASSABLE


def neighbors(grid, cell):
    """Gera (direção, vizinho) para cada vizinho transitável de `cell`."""
    r, c = cell
    for direction, (dr, dc) in DIRECTIONS.items():
        nxt = (r + dr, c + dc)
        if is_passable(grid, nxt):
            yield direction, nxt


def bfs(grid, start, goal):
    """Busca em largura de `start` até `goal`.

    Retorna a lista de células do caminho (incluindo início e fim), ou None se
    o alvo for inalcançável. Como a BFS visita as células em ordem crescente
    de distância, a primeira vez que o alvo é alcançado é pelo caminho mais
    curto. Complexidade: O(V + E), com V células e E ligações entre elas.
    """
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
    """Converte uma lista de células em comandos ('up', 'down', ...)."""
    delta_to_dir = {delta: name for name, delta in DIRECTIONS.items()}
    moves = []
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        moves.append(delta_to_dir[(r2 - r1, c2 - c1)])
    return moves


def render(grid, path=()):
    """Desenha o grid em texto, marcando o caminho com '·'.

    '█' parede, ' ' livre, 'R' robô, 'T' alvo, '?' desconhecido.
    """
    symbols = {BLOCKED: '█', FREE: ' ', ROBOT: 'R', TARGET: 'T', UNKNOWN: '?'}
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
