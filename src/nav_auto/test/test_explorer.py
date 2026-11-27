import csv
import pathlib
from collections import deque

import pytest

from nav_auto.explorer import (SENSOR_OFFSETS, Explorer, MapConflict, add,
                               compare_with_real)
from nav_auto.grid import BLOCKED, DIRECTIONS, FREE, ROBOT, TARGET, UNKNOWN, bfs, find_cell

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

    def sense(self, accept):
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


def to_abs(cells, origin):
    return [add(c, origin) for c in cells]


@pytest.mark.skipif(not MAP_FILES, reason='submódulo culling_games não encontrado')
@pytest.mark.parametrize('map_path', MAP_FILES, ids=lambda p: p.name)
def test_explores_every_simulator_map(map_path):
    real = load_map(map_path)
    robot = FakeRobot(real)
    explorer = Explorer(robot.sense, robot.move)
    origin = robot.start  # posição real da célula relativa (0, 0)

    explorer.explore()

    # Achou o alvo no lugar certo e o mapa construído bate com o real.
    assert add(explorer.target, origin) == robot.target
    assert compare_with_real(explorer.known, real, origin) == []
    # Visitou todas as células alcançáveis (sem passar pelo alvo).
    assert set(to_abs(explorer.visited, origin)) == reachable_without_target(real, robot.start)

    explorer.go_home()
    assert robot.pos == robot.start

    # A rota no mapa construído é a mesma da parte 1 (BFS no mapa real).
    route = explorer.best_route()
    assert to_abs(route, origin) == bfs(real, robot.start, robot.target)

    robot.allow_target = True
    explorer.follow(route)
    assert robot.pos == robot.target


def test_ignores_pocket_seen_only_diagonally():
    # A célula livre em (1, 3) só é vista na diagonal, a partir de (2, 2), e está
    # fechada por paredes: o robô a anota no mapa, mas não tenta chegar nela.
    real = [list(row) for row in [
        'bbbbb',
        'brbfb',
        'bffbb',
        'bbtbb',
        'bbbbb',
    ]]
    robot = FakeRobot(real)
    explorer = Explorer(robot.sense, robot.move)
    explorer.explore()
    assert explorer.known[(0, 2)] == FREE      # célula (1, 3) do mapa real
    assert (0, 2) not in explorer.visited
    assert explorer.target == (2, 1)


def test_to_grid_has_get_map_format():
    real = [list(row) for row in [
        'bbbbb',
        'brffb',
        'bbbtb',
        'bbbbb',
    ]]
    robot = FakeRobot(real)
    explorer = Explorer(robot.sense, robot.move)
    explorer.explore()
    grid, _ = explorer.to_grid()
    assert find_cell(grid, ROBOT) == robot.start
    assert find_cell(grid, TARGET) == robot.target
    assert all(cell != UNKNOWN for cell in grid[1])


def test_conflicting_reading_raises():
    # Sensor quebrado: sempre diz "livre à direita, parede no resto", em
    # qualquer posição. Depois do 1º passo, ele diz que a esquerda (de onde o
    # robô veio, que é livre) é parede.
    reading = {name: BLOCKED for name in SENSOR_OFFSETS}
    reading['right'] = FREE
    explorer = Explorer(lambda accept: reading, lambda direction: True)
    with pytest.raises(MapConflict):
        explorer.explore()
