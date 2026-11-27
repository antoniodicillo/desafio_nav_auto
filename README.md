# Ponderada - Mapeamento do Labirinto

Solução para o desafio de navegação autônoma em labirinto usando ROS 2 Jazzy e
o simulador [culling_games](https://github.com/rmnicola/culling_games).

- **Parte 1 (navegação com mapa):** com o mapa completo, encontra a rota mais
  curta até o alvo com busca em largura (BFS) e a executa.
  [Documentação](docs/part1.md)
- **Parte 2 (mapeamento do labirinto):** sem o mapa, explora o labirinto com
  busca em profundidade (DFS) usando os sensores, constrói o mapa e prova que
  ele reproduz a rota ótima da parte 1. [Documentação](docs/part2.md)

Como o projeto funciona (ROS 2, o simulador, o labirinto como grafo) e como
montar o ambiente: [docs/setup.md](docs/setup.md).

## Como funciona

**Parte 1.** O nó pede o mapa ao simulador (`/get_map`), roda uma BFS da
posição do robô até o alvo, converte o caminho em comandos (`up`, `down`,
`left`, `right`) e os envia um por vez (`/move_command`), conferindo a
posição do robô a cada passo. Em um grafo não ponderado, como este grid, a
BFS sempre acha o caminho com o menor número de movimentos.

**Parte 2.** O nó não usa o mapa para navegar:

1. **Explora com DFS:** em cada célula nova, lê os 8 sensores
   (`/culling_games/robot_sensors`) e anota o que viu; anda para uma vizinha
   livre ainda não visitada ou, num beco sem saída, volta um passo. Termina
   quando já visitou tudo, de volta na partida. O robô vê o alvo pelos
   sensores, mas não entra nele, porque isso terminaria o jogo.
2. **Planeja e executa** a rota até o alvo com a mesma BFS da parte 1, mas
   no mapa construído.
3. **Confere:** só então pede o mapa real e compara. O mapa construído deve
   ter zero diferenças, e a rota deve ser a mesma da parte 1.

## Resultados

| Labirinto | Parte 1 (rota) | Parte 2 (exploração) | Parte 2 (rota) | Mapa construído |
| --- | --- | --- | --- | --- |
| `1.csv` | 102 movimentos | 780 movimentos, 9,2 s | 102, idêntica à da parte 1 | 0 diferenças |
| `5.csv` | 204 movimentos | 780 movimentos, 9,2 s | 204, idêntica | 0 diferenças |
| `7.csv` | 96 movimentos | 780 movimentos, 9,3 s | 96, idêntica | 0 diferenças |
| `test.csv` | 25 movimentos | 1454 movimentos, 16,9 s | 25, idêntica | 0 diferenças |
| `--generate` | 74 movimentos | 780 movimentos, 9,2 s | 74, idêntica | 0 diferenças |

Os testes unitários repetem a parte 2 nos 11 labirintos do simulador, sem
precisar do jogo aberto: em todos, o robô visita todas as células
alcançáveis, o mapa bate com o real e a rota é idêntica à da parte 1. Mais
resultados em [part1.md](docs/part1.md#resultados) e
[part2.md](docs/part2.md#resultados).

## Estrutura

```
desafio_nav_auto/
├── .gitattributes              # força quebras de linha LF (código roda no WSL)
├── docs/
│   ├── setup.md                # teoria do projeto + montagem do ambiente
│   ├── part1.md                # parte 1, com o código função por função
│   └── part2.md                # parte 2, com o código função por função
└── src/
    ├── culling_games/          # simulador (submódulo git, código do professor)
    └── nav_auto/               # minha solução (pacote ROS 2)
        ├── nav_auto/
        │   ├── grid.py         # grid e BFS (sem ROS), usado nas duas partes
        │   ├── part1_node.py   # nó da parte 1
        │   ├── explorer.py     # exploração com DFS (sem ROS)
        │   └── part2_node.py   # nó da parte 2
        └── test/
            ├── test_grid.py      # testes da BFS
            └── test_explorer.py  # testes da exploração nos 11 labirintos
```

## Início rápido

```bash
git clone --recurse-submodules https://github.com/antoniodicillo/desafio_nav_auto.git
cd desafio_nav_auto
colcon build
source install/setup.bash
```

Terminal 1, o jogo (com a janela aberta):

```bash
ros2 run cg maze -- --map 1.csv
```

Terminal 2, a parte 1 ou a parte 2:

```bash
ros2 run nav_auto part1
ros2 run nav_auto part2
```

A parte 2 reinicia o labirinto sozinha no começo, então pode rodar logo
depois da parte 1, no mesmo jogo.

Testes (não precisam do jogo aberto):

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test
```
