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

# O próprio robô ('r') nunca aparece: os sensores leem as células ao redor dele.
SENSOR_VALUES = {FREE, BLOCKED, TARGET}


class MapConflict(Exception):
    pass


def add(cell, delta):
    return (cell[0] + delta[0], cell[1] + delta[1])


def direction_between(a, b):
    delta = (b[0] - a[0], b[1] - a[1])
    for name, d in DIRECTIONS.items():
        if d == delta:
            return name
    raise ValueError(f'{a} e {b} não são vizinhas')


class Explorer:

    def __init__(self, sense, move):
        self.sense = sense      # sense(accept) -> leitura dos 8 sensores
        self.move = move        # move(direção) -> True se o robô andou
        self.start = (0, 0)     # coordenadas relativas: a partida é (0, 0)
        self.pos = self.start
        self.known = {}         # célula -> FREE, BLOCKED ou TARGET
        self.visited = set()
        # Livres, não visitadas, vizinhas diretas de alguma visitada.
        self.frontier = set()
        self.target = None
        self.moves = 0

    def agrees_with_map(self, reading):
        # Células conhecidas nunca mudam; se a leitura discorda, ela é velha.
        for name, offset in SENSOR_OFFSETS.items():
            value = reading[name]
            if value not in SENSOR_VALUES:
                return False
            known = self.known.get(add(self.pos, offset))
            if known is not None and known != value:
                return False
        return True

    def visit(self):
        reading = self.sense(self.agrees_with_map)
        if not self.agrees_with_map(reading):
            raise MapConflict(f'Leitura em {self.pos} contradiz o mapa: {reading}')
        self.known[self.pos] = FREE
        self.visited.add(self.pos)
        self.frontier.discard(self.pos)
        for name, offset in SENSOR_OFFSETS.items():
            cell = add(self.pos, offset)
            value = reading[name]
            self.known[cell] = value
            if value == TARGET:
                self.target = cell
            # Diagonais não entram na fronteira: podem estar fechadas por paredes.
            elif value == FREE and name in DIRECTIONS and cell not in self.visited:
                self.frontier.add(cell)

    def step(self, direction):
        if not self.move(direction):
            raise RuntimeError(f'Movimento "{direction}" a partir de {self.pos} falhou')
        self.pos = add(self.pos, DIRECTIONS[direction])
        self.moves += 1

    def follow(self, path):
        if path[0] != self.pos:
            raise ValueError(f'O caminho começa em {path[0]}, mas o robô está em {self.pos}')
        for cell in path[1:]:
            self.step(direction_between(self.pos, cell))

    def unvisited_neighbor(self):
        # O alvo não é FREE, então o robô nunca entra nele explorando.
        for direction, delta in DIRECTIONS.items():
            cell = add(self.pos, delta)
            if self.known.get(cell) == FREE and cell not in self.visited:
                return direction
        return None

    def explore(self):
        self.visit()
        stack = [self.pos]  # caminho da partida até a posição atual
        while self.frontier:
            direction = self.unvisited_neighbor()
            if direction is not None:
                self.step(direction)
                stack.append(self.pos)
                self.visit()
            else:
                stack.pop()
                if not stack:  # impossível com a fronteira cheia (docs/part2.md)
                    raise RuntimeError('Pilha vazia com a fronteira ainda cheia')
                self.step(direction_between(self.pos, stack[-1]))
        return self.moves

    def to_grid(self):
        # Mesmo formato de /get_map; a célula relativa (l, c) fica em
        # grid[l - top][c - left].
        rows = [r for r, _ in self.known]
        cols = [c for _, c in self.known]
        top, left = min(rows), min(cols)
        grid = [[UNKNOWN] * (max(cols) - left + 1) for _ in range(max(rows) - top + 1)]
        for (r, c), value in self.known.items():
            grid[r - top][c - left] = value
        grid[self.start[0] - top][self.start[1] - left] = ROBOT
        return grid, (top, left)

    def plan(self, origin, goal, avoid_target=False):
        grid, (top, left) = self.to_grid()
        if avoid_target and self.target is not None:
            grid[self.target[0] - top][self.target[1] - left] = BLOCKED
        path = bfs(grid, (origin[0] - top, origin[1] - left), (goal[0] - top, goal[1] - left))
        if path is None:
            return None
        return [(r + top, c + left) for r, c in path]

    def go_home(self):
        path = self.plan(self.pos, self.start, avoid_target=True)
        if path is None:
            raise RuntimeError('Não há caminho de volta à partida no mapa construído')
        self.follow(path)
        return len(path) - 1

    def best_route(self):
        if self.target is None:
            return None
        return self.plan(self.start, self.target)


def compare_with_real(known, real_grid, origin):
    # Compara só parede x transitável: no mapa real o 'r' pode estar em cima
    # de uma célula livre ou do alvo.
    mismatches = []
    for cell, value in known.items():
        r, c = add(cell, origin)
        if not (0 <= r < len(real_grid) and 0 <= c < len(real_grid[0])):
            mismatches.append(((r, c), value, None))
            continue
        real = real_grid[r][c]
        if (value == BLOCKED) != (real == BLOCKED):
            mismatches.append(((r, c), value, real))
    return mismatches
