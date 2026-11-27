import time

import rclpy
from rclpy.node import Node

from cg_interfaces.srv import GetMap, MoveCmd

from nav_auto.grid import ROBOT, TARGET, bfs, find_cell, grid_from_flat, path_to_moves, render


class Part1Navigator(Node):

    def __init__(self):
        super().__init__('part1_navigator')
        # Pausa entre movimentos, só para dar para ver o robô andando na janela.
        self.declare_parameter('step_delay', 0.05)
        self.step_delay = self.get_parameter('step_delay').value

        self.get_map_client = self.create_client(GetMap, '/get_map')
        self.move_client = self.create_client(MoveCmd, '/move_command')

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_services(self):
        for client in (self.get_map_client, self.move_client):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando o serviço {client.srv_name}... '
                                       '(o jogo está rodando? ros2 run cg maze)')

    def run(self):
        self.wait_for_services()

        # 1. Mapa completo e espera o pygame renderizar
        response = self.call(self.get_map_client, GetMap.Request())
        grid = grid_from_flat(response.occupancy_grid_flattened, response.occupancy_grid_shape)
        start = find_cell(grid, ROBOT)
        goal = find_cell(grid, TARGET)
        self.get_logger().info(f'Mapa recebido: {len(grid)}x{len(grid[0])}, '
                               f'robô em {start}, alvo em {goal}')
        if goal is None:
            self.get_logger().error(
                'Alvo não encontrado no mapa: o jogo já foi vencido. Reinicie com: '
                "ros2 service call /reset cg_interfaces/srv/Reset \"{is_random: false}\"")
            return False

        # 2. Planejamento
        t0 = time.perf_counter()
        path = bfs(grid, start, goal)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        if path is None:
            self.get_logger().error('Não existe caminho até o alvo.')
            return False
        moves = path_to_moves(path)
        self.get_logger().info(f'Rota ótima (BFS): {len(moves)} movimentos, '
                               f'calculada em {elapsed_ms:.2f} ms')
        print(render(grid, path), flush=True)

        # 3. Execucao
        for i, direction in enumerate(moves):
            request = MoveCmd.Request()
            request.direction = direction
            result = self.call(self.move_client, request)
            robot_pos = tuple(int(v) for v in result.robot_pos)
            expected = path[i + 1]
            if not result.success or robot_pos != expected:
                self.get_logger().error(f'Passo {i + 1} ({direction}) falhou: robô em '
                                        f'{robot_pos}, esperado {expected}')
                return False
            if self.step_delay > 0:
                time.sleep(self.step_delay)

        self.get_logger().info(f'Alvo alcançado em {goal} com {len(moves)} movimentos!')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Part1Navigator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
