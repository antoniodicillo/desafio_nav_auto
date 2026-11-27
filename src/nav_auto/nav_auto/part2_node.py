import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset

from nav_auto.explorer import SENSOR_OFFSETS, Explorer, MapConflict, add, compare_with_real
from nav_auto.grid import (BLOCKED, DIRECTIONS, ROBOT, TARGET, bfs, find_cell,
                           grid_from_flat, render)

# Mensagens dos sensores a esperar depois de esvaziar a fila: a primeira ainda
# pode ter sido publicada antes do movimento.
FRESH_MESSAGES = 2


class Part2Mapper(Node):

    def __init__(self):
        super().__init__('part2_mapper')
        # Pausas entre movimentos, só para dar para ver o robô na janela.
        self.declare_parameter('step_delay', 0.0)
        self.declare_parameter('route_step_delay', 0.1)
        self.declare_parameter('reset_first', True)
        # O jogo leva ~0,6 a 0,9 s para aplicar um reset (recria a janela e
        # recarrega as imagens); movimentos feitos antes disso se perdem.
        self.declare_parameter('reset_wait', 3.0)
        self.declare_parameter('verify', True)
        self.declare_parameter('sensor_timeout', 3.0)
        self.step_delay = self.get_parameter('step_delay').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value

        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.reset_client = self.create_client(Reset, '/reset')
        self.get_map_client = self.create_client(GetMap, '/get_map')
        self.create_subscription(RobotSensors, '/culling_games/robot_sensors',
                                 self.on_sensors, qos_profile_sensor_data)

        self.last_reading = None
        self.sensor_count = 0
        # Odometria própria, conferida com a posição informada pelo simulador.
        # origin = posição real da partida; só é usada na conferência final.
        self.odom = (0, 0)
        self.origin = None

    def on_sensors(self, msg):
        self.last_reading = {name: getattr(msg, name) for name in SENSOR_OFFSETS}
        self.sensor_count += 1

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_services(self):
        for client in (self.move_client, self.reset_client, self.get_map_client):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando o serviço {client.srv_name}... '
                                       '(o jogo está rodando? ros2 run cg maze)')

    def wait_for_sensors(self):
        while self.sensor_count == 0:
            self.get_logger().info('Esperando mensagens dos sensores...')
            self.spin_for(1.0)

    def drain(self):
        # Processa o que já estava na fila: mensagens de antes do movimento.
        while True:
            before = self.sensor_count
            rclpy.spin_once(self, timeout_sec=0)
            if self.sensor_count == before:
                return

    def sense(self, accept):
        self.drain()
        baseline = self.sensor_count
        deadline = time.monotonic() + self.sensor_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.sensor_count >= baseline + FRESH_MESSAGES and accept(self.last_reading):
                return self.last_reading
        raise RuntimeError(f'Nenhuma leitura válida dos sensores em {self.sensor_timeout} s '
                           f'(última: {self.last_reading})')

    def move(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        result = self.call(self.move_client, request)
        if not result.success:
            return False
        # result.target_pos entrega a posição do alvo: ignorado de propósito.
        self.odom = add(self.odom, DIRECTIONS[direction])
        reported = (int(result.robot_pos[0]), int(result.robot_pos[1]))
        if self.origin is None:
            self.origin = (reported[0] - self.odom[0], reported[1] - self.odom[1])
        elif add(self.origin, self.odom) != reported:
            raise RuntimeError(f'Odometria {self.odom} não bate com a posição informada '
                               f'pelo simulador {reported} (o reset pode ter sido aplicado '
                               'durante a exploração: aumente reset_wait)')
        if self.step_delay > 0:
            time.sleep(self.step_delay)
        return True

    def run(self):
        self.wait_for_services()
        self.wait_for_sensors()

        if self.get_parameter('reset_first').value:
            response = self.call(self.reset_client, Reset.Request())
            self.get_logger().info(f'Labirinto reiniciado: {response.loaded_map_name}')
            self.spin_for(self.get_parameter('reset_wait').value)

        explorer = Explorer(self.sense, self.move)

        # 1. Exploração
        t0 = time.perf_counter()
        exploration_moves = explorer.explore()
        elapsed = time.perf_counter() - t0
        self.get_logger().info(f'Exploração concluída: {len(explorer.visited)} células '
                               f'visitadas, {exploration_moves} movimentos, {elapsed:.1f} s')
        grid, _ = explorer.to_grid()
        print(render(grid), flush=True)
        if explorer.target is None:
            self.get_logger().error('O alvo não foi encontrado: ele não é alcançável.')
            return False
        self.get_logger().info(f'Alvo visto em {explorer.target} (relativo à partida)')

        # 2. Volta à partida
        home_moves = explorer.go_home()
        self.get_logger().info(f'De volta à partida com {home_moves} movimentos')

        # 3. Planejamento no mapa construído
        route = explorer.best_route()
        grid, (top, left) = explorer.to_grid()
        self.get_logger().info(f'Rota ótima no mapa construído (BFS): {len(route) - 1} movimentos')
        print(render(grid, [(r - top, c - left) for r, c in route]), flush=True)

        # 4. Execução
        self.step_delay = self.get_parameter('route_step_delay').value
        explorer.follow(route)
        self.get_logger().info(f'Alvo alcançado com {len(route) - 1} movimentos!')

        # 5. Conferência com o mapa real
        if self.get_parameter('verify').value:
            return self.verify(explorer, route)
        return True

    def verify(self, explorer, route):
        response = self.call(self.get_map_client, GetMap.Request())
        real = grid_from_flat(response.occupancy_grid_flattened, response.occupancy_grid_shape)
        start = self.origin
        target = add(self.origin, explorer.target)
        log = self.get_logger()

        mismatches = compare_with_real(explorer.known, real, self.origin)
        log.info(f'Mapa: {len(explorer.known)} células conhecidas, {len(mismatches)} '
                 'diferentes do mapa real')
        for cell, ours, theirs in mismatches[:10]:
            log.error(f'  célula {cell}: nosso mapa "{ours}", mapa real "{theirs}"')

        total_free = sum(cell != BLOCKED for row in real for cell in row)
        known_free = sum(value != BLOCKED for value in explorer.known.values())
        log.info(f'Células transitáveis do labirinto real conhecidas: {known_free}/{total_free}')

        # Depois da vitória, o robô ('r') fica em cima do alvo e o 't' some.
        won = find_cell(real, TARGET) is None and real[target[0]][target[1]] == ROBOT
        log.info(f'Alvo em {target} no simulador; jogo vencido lá: {"sim" if won else "não"}')

        # Rota da parte 1: BFS no mapa real.
        part1 = bfs(real, start, target)
        ours = [add(self.origin, cell) for cell in route]
        same_length = part1 is not None and len(part1) == len(ours)
        log.info(f'Rota da parte 1 (mapa real): {len(part1) - 1 if part1 else "-"} movimentos; '
                 f'rota da parte 2 (mapa construído): {len(ours) - 1} movimentos; '
                 f'{"idênticas" if part1 == ours else "diferentes"}')

        ok = not mismatches and won and same_length
        if ok:
            log.info('Conferência OK: o mapa construído reproduz a rota ótima da parte 1.')
        else:
            log.error('Conferência falhou.')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = Part2Mapper()
    try:
        node.run()
    except (RuntimeError, MapConflict) as error:
        node.get_logger().error(str(error))
        if not node.get_parameter('reset_first').value:
            node.get_logger().error('Se o jogo já foi vencido, reinicie com: ros2 service call '
                                    '/reset cg_interfaces/srv/Reset "{is_random: false}"')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
