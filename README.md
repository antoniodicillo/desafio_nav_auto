# Desafio de Navegação Autônoma

Solução para o desafio de navegação autônoma em labirinto usando ROS 2 Jazzy e
o simulador [culling_games](https://github.com/rmnicola/culling_games).

## Estrutura

```
desafio_nav_auto/
├── .gitattributes              # força quebras de linha LF (código roda no WSL)
├── docs/
│   └── 01-setup-ambiente.md   # como montar o ambiente (WSL2 + ROS 2 Jazzy)
└── src/
    └── culling_games/          # simulador (submódulo git, código do professor)
```

## Início rápido

```bash
git clone --recurse-submodules https://github.com/antoniodicillo/desafio_nav_auto.git
cd desafio_nav_auto
colcon build
source install/setup.bash
ros2 run cg maze
```

Para o passo a passo completo do ambiente, veja
[docs/01-setup-ambiente.md](docs/01-setup-ambiente.md).
