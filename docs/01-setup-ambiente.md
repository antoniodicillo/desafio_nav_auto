# 01 — Configuração do Ambiente (Windows + WSL2 + ROS 2 Jazzy)

Este documento registra, passo a passo, como o ambiente de desenvolvimento do
desafio de navegação autônoma foi montado em uma máquina Windows 11 usando
WSL2. O objetivo é que qualquer pessoa consiga reproduzir o ambiente do zero.

## Visão geral

| Item | Versão usada |
| --- | --- |
| Sistema hospedeiro | Windows 11 |
| WSL | 2.7.12 (com WSLg para janelas gráficas) |
| Distribuição Linux | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco (`ros-jazzy-desktop`) |
| Python | 3.12 (padrão do Ubuntu 24.04) |
| Simulador | [culling_games](https://github.com/rmnicola/culling_games) (Pygame) |

**Por que Ubuntu 24.04 e não a versão mais recente?** Cada distribuição do
ROS 2 tem suporte oficial para uma versão específica do Ubuntu. O ROS 2 Jazzy
(LTS, usado na disciplina) roda no Ubuntu 24.04. Usar outra versão do Ubuntu
obrigaria a usar outra distribuição do ROS 2.

## Passo 1 — Instalar a distribuição Ubuntu 24.04 no WSL2

No PowerShell (Windows):

```powershell
wsl --install -d Ubuntu-24.04
```

Na primeira execução, o Ubuntu pede para criar um usuário e uma senha. Esse
usuário terá permissão de `sudo`.

Para conferir se a distribuição está rodando em WSL **2**:

```powershell
wsl -l -v
```

A coluna `VERSION` deve mostrar `2`.

> **Dica:** se você já tiver outra distribuição instalada (ex.: `Ubuntu`
> genérico), sempre abra a correta com `wsl -d Ubuntu-24.04`, ou defina-a como
> padrão com `wsl --set-default Ubuntu-24.04`.

## Passo 2 — Configurar o locale UTF-8

O ROS 2 exige um locale com suporte a UTF-8. Dentro do Ubuntu:

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

## Passo 3 — Adicionar o repositório do ROS 2

Habilitar o repositório `universe` do Ubuntu:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
```

Instalar o pacote `ros2-apt-source`, que configura a chave e o repositório
oficiais do ROS 2:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## Passo 4 — Instalar o ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

- `ros-jazzy-desktop`: ROS 2 completo, com RViz, demos e bibliotecas
  (inclui `rclpy`).
- `ros-dev-tools`: ferramentas de desenvolvimento, incluindo o `colcon` e o
  `rosdep`.

## Passo 5 — Instalar as dependências do projeto

O simulador `culling_games` usa o Pygame para desenhar o labirinto:

```bash
sudo apt install -y python3-pygame python3-colcon-common-extensions git
```

> Usamos o Pygame do `apt` (e não do `pip`) porque o Ubuntu 24.04 bloqueia
> `pip install` no Python do sistema, e o ROS 2 usa esse Python.

## Passo 6 — Carregar o ROS 2 automaticamente em cada terminal

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Para testar:

```bash
ros2 --help
```

## Passo 7 — Clonar este repositório

Neste projeto, o repositório fica em uma pasta do **Windows** (editada pelo VS
Code no Windows) e é compilado e executado **dentro do WSL**, que enxerga os
discos do Windows em `/mnt/c/`.

No PowerShell (Windows), na pasta onde o projeto deve ficar:

```powershell
git clone --recurse-submodules https://github.com/antoniodicillo/desafio_nav_auto.git
cd desafio_nav_auto
git config core.autocrlf false
git -C src/culling_games config core.autocrlf false
```

> **Por que desativar o `autocrlf`?** Por padrão, o Git no Windows converte as
> quebras de linha para o formato do Windows (CRLF). O código roda no Linux,
> que espera LF; arquivos com CRLF podem quebrar scripts e o build. O arquivo
> `.gitattributes` deste repositório já força LF nos nossos arquivos; o
> comando acima garante o mesmo dentro do submódulo.

No terminal do Ubuntu, entre na mesma pasta pelo caminho `/mnt/c/...`. Neste
computador:

```bash
cd /mnt/c/Users/anton/OneDrive/Documentos/GitHub/ier/desafio_nav_auto
```

> **Atenção:** compilar em `/mnt/c` é mais lento do que no sistema de arquivos
> do Linux (`~/`), e o OneDrive vai tentar sincronizar as pastas `build/`,
> `install/` e `log/`. Se o build ficar lento ou der erro de arquivo travado,
> pause a sincronização do OneDrive durante o build ou exclua essas pastas da
> sincronização.

O simulador `culling_games` entra como **submódulo git** em
`src/culling_games`. Assim, o código do professor fica separado do nosso e pode
ser atualizado com:

```bash
git submodule update --remote src/culling_games
```

## Passo 8 — Compilar o workspace

Na raiz do repositório:

```bash
colcon build
source install/setup.bash
```

O `colcon` encontra automaticamente os pacotes `cg`, `cg_interfaces` e
`cg_teleop` dentro de `src/culling_games/src/`.

## Passo 9 — Testar o simulador

Terminal 1:

```bash
source install/setup.bash
ros2 run cg maze
```

Uma janela do Pygame com o labirinto deve abrir no Windows (graças ao WSLg).

Terminal 2:

```bash
source install/setup.bash
ros2 run cg_teleop teleop_keyboard
```

Use `w`/`a`/`s`/`d` para mover o robô e verificar que tudo funciona.

### Lendo o tópico dos sensores

O tópico `/culling_games/robot_sensors` é publicado com o perfil de QoS
`sensor_data` (entrega *best effort*). Para lê-lo pela linha de comando, use o
mesmo perfil:

```bash
ros2 topic echo --qos-profile sensor_data /culling_games/robot_sensors
```

Nos nós Python, assine o tópico com `qos_profile_sensor_data`
(`from rclpy.qos import qos_profile_sensor_data`). Os primeiros segundos após
iniciar o jogo podem não ter mensagens enquanto o ROS descobre os nós.

## Resultado da verificação (24/11/2026)

Ambiente montado conforme os passos acima e testado nesta máquina:

| Teste | Resultado |
| --- | --- |
| `colcon build` (`cg_interfaces`, `cg`, `cg_teleop`) | OK — 3 pacotes em ~3 min (em `/mnt/c`, dentro do OneDrive) |
| `ros2 run cg maze -- --map test.csv` | OK — janela do Pygame abre via WSLg |
| Serviços `/move_command`, `/reset`, `/get_map` disponíveis | OK |
| `/move_command` com `direction: right` | OK — `success=True`, `robot_pos=[1, 2]`, `target_pos=[13, 14]` |
| `/get_map` | OK — grade 29 × 29 |
| Tópico `/culling_games/robot_sensors` | OK — ~17 mensagens/s (o código pede 100/s, mas o loop de desenho do Pygame, sem limite de FPS, disputa o processador com o publicador) |

Avisos esperados durante o build, que podem ser ignorados:

- `Clock skew detected`: diferença de relógio entre o Windows e o WSL ao
  compilar em `/mnt/c`.
- `pygame was not built with support for avx2`: apenas desempenho de desenho.

## Editando o código pelo VS Code

Como o projeto está em uma pasta do Windows, basta abri-lo normalmente no VS
Code. Para compilar e rodar, use um terminal do Ubuntu dentro do VS Code
(menu do terminal → **Ubuntu-24.04 (WSL)**) ou abra o Ubuntu pelo Windows e
entre na pasta via `/mnt/c/...`.

## Problemas comuns

| Sintoma | Causa provável | Solução |
| --- | --- | --- |
| `ros2: command not found` | ROS não foi carregado no terminal | `source /opt/ros/jazzy/setup.bash` |
| `Package 'cg' not found` | Workspace não foi carregado | `source install/setup.bash` na raiz do repo |
| Janela do Pygame não abre | WSLg desativado ou WSL desatualizado | `wsl --update` no PowerShell e `wsl --shutdown` |
| `colcon build` muito lento ou erro de arquivo travado | OneDrive sincronizando `build/`/`install/`/`log/` | Pausar o OneDrive durante o build |
| `/usr/bin/env: 'python3\r': No such file or directory` | Arquivos com quebra de linha CRLF | `git config core.autocrlf false` e refazer o checkout (Passo 7) |
| `src/culling_games` vazio | Clonou sem os submódulos | `git submodule update --init` |
