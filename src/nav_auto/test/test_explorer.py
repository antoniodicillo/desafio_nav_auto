import csv
import pathlib
from collections import deque

import pytest

from nav_auto.explorer import SENSOR_OFFSETS, Explorer, add, compare_with_real
from nav_auto.grid import BLOCKED, DIRECTIONS, FREE, ROBOT, TARGET, bfs, find_cell

# Labirintos do simulador (submódulo src/culling_games).
MAPS_DIR = pathlib.Path(__file__).resolve().parents[2] / 'culling_games' / 'src' / 'cg' / 'maps'
MAP_FILES = sorted(MAPS_DIR.glob('*.csv'))


# Imita o simulador: guarda o grid real e responde a sense() e move().
class FakeRobot:

    def __init__(self, grid):
        self.grid = [row[:] for row in grid]
        self.start = find_cell(self.grid, ROBOT)
        self.target = find_cell(self.grid, TARGET)
        self.grid[self.start[0]][self.start[1]] = FREE  # a posição fica em self.pos
        self.pos = self.start
        self.allow_target = False  # pisar no alvo só é permitido na rota final

    def sense(self):
        return {name: self.grid[self.pos[0] + dr][self.pos[1] + dc]
                for name, (dr, dc) in SENSOR_OFFSETS.items()}

    def move(self, direction):
        nxt = add(self.pos, DIRECTIONS[direction])
        if self.grid[nxt[0]][nxt[1]] == BLOCKED:
            return False
        assert self.allow_target or nxt != self.target, 'o robô pisou no alvo explorando'
        self.pos = nxt
        return True


def load_map(path):
    with open(path) as f:
        return [row for row in csv.reader(f) if row]


# Células alcançáveis da partida sem passar pelo alvo (flood fill).
def reachable_without_target(grid, start):
    seen = {start}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        for delta in DIRECTIONS.values():
            nxt = add(cell, delta)
            if nxt not in seen and grid[nxt[0]][nxt[1]] in (FREE, ROBOT):
                seen.add(nxt)
                queue.append(nxt)
    return seen


@pytest.mark.skipif(not MAP_FILES, reason='submódulo culling_games não encontrado')
@pytest.mark.parametrize('map_path', MAP_FILES, ids=lambda p: p.name)
def test_explores_every_simulator_map(map_path):
    real = load_map(map_path)
    robot = FakeRobot(real)
    explorer = Explorer(robot.sense, robot.move)
    origin = robot.start  # posição real da célula relativa (0, 0)

    explorer.explore()

    # Achou o alvo no lugar certo, o mapa bate com o real, visitou todas as
    # células alcançáveis e terminou de volta na partida.
    assert add(explorer.target, origin) == robot.target
    assert compare_with_real(explorer.known, real, origin) == []
    assert {add(c, origin) for c in explorer.visited} == reachable_without_target(real, origin)
    assert robot.pos == robot.start

    # A rota no mapa construído é a mesma da parte 1 (BFS no mapa real).
    route = explorer.best_route()
    assert [add(c, origin) for c in route] == bfs(real, robot.start, robot.target)

    robot.allow_target = True
    explorer.follow(route)
    assert robot.pos == robot.target
