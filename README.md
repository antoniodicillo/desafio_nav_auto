# Desafio de Navegação Autônoma

Solução para o desafio de navegação autônoma em labirinto usando ROS 2 Jazzy e
o simulador [culling_games](https://github.com/rmnicola/culling_games).

- **Parte 1 — navegação com mapa:** com o mapa completo, encontra a rota mais
  curta até o alvo com busca em largura (BFS) e a executa.
  [Documentação](docs/02-parte1-navegacao-com-mapa.md)
- **Parte 2 — mapeamento do labirinto:** em desenvolvimento.

## Estrutura

```
desafio_nav_auto/
├── .gitattributes              # força quebras de linha LF (código roda no WSL)
├── docs/
│   ├── 01-setup-ambiente.md    # como montar o ambiente (WSL2 + ROS 2 Jazzy)
│   └── 02-parte1-navegacao-com-mapa.md
└── src/
    ├── culling_games/          # simulador (submódulo git, código do professor)
    └── nav_auto/               # nossa solução (pacote ROS 2)
        ├── nav_auto/
        │   ├── grid.py         # BFS e funções do grid (sem ROS)
        │   └── part1_node.py   # nó da parte 1
        └── test/
            └── test_grid.py    # testes unitários da BFS
```

## Início rápido

```bash
git clone --recurse-submodules https://github.com/antoniodicillo/desafio_nav_auto.git
cd desafio_nav_auto
colcon build
source install/setup.bash
```

Terminal 1 — o jogo:

```bash
ros2 run cg maze
```

Terminal 2 — parte 1:

```bash
ros2 run nav_auto part1
```

Para o passo a passo completo do ambiente, veja
[docs/01-setup-ambiente.md](docs/01-setup-ambiente.md).
