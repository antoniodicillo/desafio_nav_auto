import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset

from nav_auto.explorer import SENSOR_OFFSETS, Explorer, add, compare_with_real
from nav_auto.grid import DIRECTIONS, bfs, grid_from_flat, render

# O jogo leva ~0,6 a 0,9 s para aplicar um reset; movimentos antes disso se perdem.
RESET_WAIT = 3.0
# Mensagens a esperar depois de esvaziar a fila: a primeira ainda pode ser de
# antes do movimento.
FRESH_MESSAGES = 2
SENSOR_TIMEOUT = 3.0
# Pausa entre movimentos da rota final, só para dar para ver na janela.
ROUTE_STEP_DELAY = 0.1


class Part2Mapper(Node):

    def __init__(self):
        super().__init__('part2_mapper')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.reset_client = self.create_client(Reset, '/reset')
        self.get_map_client = self.create_client(GetMap, '/get_map')
        self.create_subscription(RobotSensors, '/culling_games/robot_sensors',
                                 self.on_sensors, qos_profile_sensor_data)
        self.last_reading = None
        self.sensor_count = 0
        self.step_delay = 0.0
        self.origin = None  # posição real da partida, só para a conferência

    def on_sensors(self, msg):
        self.last_reading = {name: getattr(msg, name) for name in SENSOR_OFFSETS}
        self.sensor_count += 1

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_services(self):
        for client in (self.move_client, self.reset_client, self.get_map_client):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando o serviço {client.srv_name}... '
                                       '(o jogo está rodando? ros2 run cg maze)')

    def sense(self):
        # Esvazia a fila (mensagens de antes do movimento) e espera mensagens novas.
        count = -1
        while count != self.sensor_count:
            count = self.sensor_count
            rclpy.spin_once(self, timeout_sec=0)
        deadline = time.monotonic() + SENSOR_TIMEOUT
        while self.sensor_count < count + FRESH_MESSAGES:
            if time.monotonic() > deadline:
                raise RuntimeError('Os sensores pararam de publicar (docs/setup.md, Passo 7)')
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.last_reading

    def move(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        result = self.call(self.move_client, request)
        # result.target_pos entrega a posição do alvo: ignorado de propósito.
        if result.success and self.origin is None:
            dr, dc = DIRECTIONS[direction]
            self.origin = (int(result.robot_pos[0]) - dr, int(result.robot_pos[1]) - dc)
        time.sleep(self.step_delay)
        return result.success

    def run(self):
        self.wait_for_services()
        self.call(self.reset_client, Reset.Request())  # recomeça o mesmo labirinto
        end = time.monotonic() + RESET_WAIT
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

        # 1. Exploração com DFS (termina de volta na partida)
        explorer = Explorer(self.sense, self.move)
        t0 = time.perf_counter()
        explorer.explore()
        self.get_logger().info(f'Exploração: {len(explorer.visited)} células visitadas, '
                               f'{explorer.moves} movimentos, {time.perf_counter() - t0:.1f} s')
        if explorer.target is None:
            self.get_logger().error('O alvo não foi encontrado.')
            return

        # 2. Rota com BFS no mapa construído
        route = explorer.best_route()
        grid, (top, left) = explorer.to_grid()
        self.get_logger().info(f'Rota no mapa construído (BFS): {len(route) - 1} movimentos')
        print(render(grid, [(r - top, c - left) for r, c in route]), flush=True)
        self.step_delay = ROUTE_STEP_DELAY
        explorer.follow(route)
        self.get_logger().info('Alvo alcançado!')

        # 3. Conferência: só agora pede o mapa real
        response = self.call(self.get_map_client, GetMap.Request())
        real = grid_from_flat(response.occupancy_grid_flattened, response.occupancy_grid_shape)
        mismatches = compare_with_real(explorer.known, real, self.origin)
        part1 = bfs(real, self.origin, add(self.origin, explorer.target)) or []
        ours = [add(self.origin, cell) for cell in route]
        self.get_logger().info(f'Conferência: {len(mismatches)} células diferentes do mapa '
                               f'real; rota da parte 1: {len(part1) - 1} movimentos; '
                               f'rota da parte 2: {len(ours) - 1} movimentos; '
                               f'{"idênticas" if part1 == ours else "diferentes"}')


def main(args=None):
    rclpy.init(args=args)
    node = Part2Mapper()
    try:
        node.run()
    except RuntimeError as error:
        node.get_logger().error(str(error))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
