from nav_auto.grid import BLOCKED, DIRECTIONS, FREE, ROBOT, TARGET, UNKNOWN, bfs

# Campo da mensagem RobotSensors -> deslocamento (linha, coluna) da célula lida.
SENSOR_OFFSETS = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
    'up_left': (-1, -1),
    'up_right': (-1, 1),
    'down_left': (1, -1),
    'down_right': (1, 1),
}


def add(cell, delta):
    return (cell[0] + delta[0], cell[1] + delta[1])


def direction_between(a, b):
    delta = (b[0] - a[0], b[1] - a[1])
    return next(name for name, d in DIRECTIONS.items() if d == delta)


class Explorer:

    def __init__(self, sense, move):
        self.sense = sense      # sense() -> leitura dos 8 sensores
        self.move = move        # move(direção) -> True se o robô andou
        self.pos = (0, 0)       # coordenadas relativas: a partida é (0, 0)
        self.known = {}         # célula -> FREE, BLOCKED ou TARGET
        self.visited = set()
        self.target = None
        self.moves = 0

    def visit(self):
        self.known[self.pos] = FREE
        self.visited.add(self.pos)
        for name, value in self.sense().items():
            cell = add(self.pos, SENSOR_OFFSETS[name])
            self.known[cell] = value
            if value == TARGET:
                self.target = cell

    def step(self, direction):
        if not self.move(direction):
            raise RuntimeError(f'Movimento "{direction}" a partir de {self.pos} falhou')
        self.pos = add(self.pos, DIRECTIONS[direction])
        self.moves += 1

    def unvisited_neighbor(self):
        # O alvo é TARGET, não FREE: o robô nunca entra nele explorando.
        for direction, delta in DIRECTIONS.items():
            cell = add(self.pos, delta)
            if self.known.get(cell) == FREE and cell not in self.visited:
                return direction
        return None

    def explore(self):
        # DFS: avança para uma vizinha nova; num beco sem saída, volta um passo.
        # Termina com a pilha vazia, de volta na partida.
        self.visit()
        stack = [self.pos]  # caminho da partida até a posição atual
        while stack:
            direction = self.unvisited_neighbor()
            if direction is not None:
                self.step(direction)
                stack.append(self.pos)
                self.visit()
            else:
                stack.pop()
                if stack:
                    self.step(direction_between(self.pos, stack[-1]))

    def to_grid(self):
        # Mesmo formato de /get_map: '?' no que não foi visto, 'r' na partida.
        # A célula relativa (l, c) fica em grid[l - top][c - left].
        top = min(r for r, _ in self.known)
        left = min(c for _, c in self.known)
        rows = max(r for r, _ in self.known) - top + 1
        cols = max(c for _, c in self.known) - left + 1
        grid = [[UNKNOWN] * cols for _ in range(rows)]
        for (r, c), value in self.known.items():
            grid[r - top][c - left] = value
        grid[-top][-left] = ROBOT
        return grid, (top, left)

    def best_route(self):
        grid, (top, left) = self.to_grid()
        path = bfs(grid, (-top, -left), (self.target[0] - top, self.target[1] - left))
        return [(r + top, c + left) for r, c in path] if path else None

    def follow(self, path):
        for cell in path[1:]:
            self.step(direction_between(self.pos, cell))


def compare_with_real(known, real_grid, origin):
    # Compara só parede x transitável: no mapa real o 'r' pode estar em cima
    # de uma célula livre ou do alvo.
    mismatches = []
    for cell, value in known.items():
        r, c = add(cell, origin)
        if (value == BLOCKED) != (real_grid[r][c] == BLOCKED):
            mismatches.append((r, c))
    return mismatches
