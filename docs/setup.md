# Setup e funcionamento do projeto

Este documento tem duas metades:

1. **Como o projeto funciona, na teoria** (seções 1 a 5): o que é o desafio,
   como o ROS 2 e o simulador funcionam por dentro e como o labirinto vira um
   problema de grafos.
2. **Como montar o ambiente** (seções 6 a 9): o passo a passo para instalar e
   rodar tudo do zero, em Windows 11 + WSL2 + ROS 2 Jazzy, e os problemas que
   já apareceram e como resolvê-los.

Os detalhes de cada parte do desafio estão em [part1.md](part1.md)
(navegação com mapa) e [part2.md](part2.md) (mapeamento do labirinto).

## Sumário

- [1. O desafio](#1-o-desafio)
- [2. Visão geral: dois programas conversando pelo ROS](#2-visão-geral-dois-programas-conversando-pelo-ros)
- [3. Conceitos de ROS 2 usados](#3-conceitos-de-ros-2-usados)
- [4. O simulador por dentro](#4-o-simulador-por-dentro)
- [5. O labirinto como grafo](#5-o-labirinto-como-grafo)
- [6. Montando o ambiente](#6-montando-o-ambiente)
- [7. Verificação do ambiente](#7-verificação-do-ambiente)
- [8. Editando pelo VS Code](#8-editando-pelo-vs-code)
- [9. Problemas que enfrentei](#9-problemas-que-enfrentei)

---

## 1. O desafio

Um robô precisa chegar a um alvo dentro de um labirinto. O labirinto é um
**grid**: uma matriz de células do mesmo tamanho, cada uma livre ou ocupada.

| Cor na janela | Código | Significado |
| --- | --- | --- |
| Azul | `r` | o robô |
| Vermelho | `t` | o alvo |
| Branco | `f` | célula livre (o robô pode passar) |
| Cinza escuro | `b` | parede (o robô não pode passar) |

O desafio tem duas partes:

- **Parte 1 (navegação com mapa):** o robô tem acesso ao mapa inteiro e deve
  encontrar a **rota otimizada** (menor número de movimentos) até o alvo.
- **Parte 2 (mapeamento):** o robô **não** tem o mapa. Ele precisa andar pelo
  labirinto usando seus sensores, construir o mapa no caminho e, no fim,
  provar que o mapa construído basta para reproduzir a rota da parte 1.

## 2. Visão geral: dois programas conversando pelo ROS

Quando tudo está rodando, existem dois programas (processos) independentes:

```text
 Terminal 1: ros2 run cg maze                 Terminal 2: ros2 run nav_auto part1 (ou part2)
┌─────────────────────────────────┐            ┌──────────────────────────────────┐
│ Simulador (nó "Culling_Games")  │            │ Minha solução (nó part1/part2)   │
│                                 │  /get_map  │                                  │
│  • guarda o labirinto (matriz)  │◄───────────│  • pede o mapa (só parte 1)      │
│  • desenha a janela do Pygame   │/move_command  • decide para onde ir           │
│  • move o robô quando pedem     │◄───────────│  • manda um movimento por vez    │
│  • publica os sensores          │  /reset    │  • (parte 2) lê os sensores      │
│                                 │◄───────────│                                  │
│                                 │ /culling_games/robot_sensors (100×/s)         │
│                                 │───────────►│                                  │
└─────────────────────────────────┘            └──────────────────────────────────┘
```

- O **simulador** (`src/culling_games`, código do professor) é o "mundo": dono
  do labirinto e da posição do robô.
- A **minha solução** (`src/nav_auto`) é o "cérebro": não tem acesso direto à
  memória do simulador. Tudo o que ela sabe e faz passa pelo ROS.

O arquivo CSV do labirinto só é lido pelo simulador, quando ele começa (ou é
reiniciado). Durante o jogo, o estado real é a matriz na memória do
simulador; meus nós nunca abrem o CSV nem a janela.

## 3. Conceitos de ROS 2 usados

O **ROS 2** (Robot Operating System) é um conjunto de bibliotecas e
ferramentas para fazer programas de robótica conversarem entre si. Os
conceitos que este projeto usa:

| Conceito | O que é | Onde aparece aqui |
| --- | --- | --- |
| **Nó** (*node*) | Um participante da rede ROS, normalmente um programa. | O simulador é o nó `Culling_Games`; a parte 1 é o nó `part1_navigator`; a parte 2, `part2_mapper`. |
| **Serviço** (*service*) | Pergunta e resposta, como chamar uma função em outro programa: o cliente manda um pedido (*request*) e espera a resposta (*response*). | `/get_map`, `/move_command`, `/reset`. |
| **Tópico** (*topic*) | Um canal de mensagens contínuo: um nó publica, quem quiser assina. Ninguém espera resposta. | `/culling_games/robot_sensors`. |
| **Interface** | O formato dos dados de um serviço (`.srv`) ou de uma mensagem (`.msg`). | Pacote `cg_interfaces`. |
| **QoS** (*quality of service*) | Regras de entrega. *Best effort* (perfil `sensor_data`) entrega o que der, sem reenviar o que se perdeu, e guarda só as últimas mensagens. | O tópico dos sensores usa `qos_profile_sensor_data`; quem assina precisa usar o mesmo perfil. |
| **Spin** / executor | O laço que processa o que chegou (respostas de serviço, mensagens) e chama as funções de *callback*. Sem *spin*, nada é entregue. | `rclpy.spin_until_future_complete` (esperar uma resposta) e `rclpy.spin_once` (processar o que chegou). |
| **Parâmetro** | Configuração de um nó passada na linha de comando. | `step_delay` da parte 1 (`--ros-args -p step_delay:=0.0`). |
| **Pacote** | Uma unidade de código ROS, com `package.xml`. | `cg`, `cg_interfaces`, `cg_teleop` (simulador) e `nav_auto` (meu). |
| **Workspace** e **colcon** | Uma pasta com `src/` cheia de pacotes; `colcon build` compila todos e instala em `install/`. `source install/setup.bash` faz o terminal enxergá-los. | A raiz deste repositório. |
| **Entry point** | O nome do executável que `ros2 run pacote nome` roda, definido no `setup.py`. | `part1` e `part2` em `src/nav_auto/setup.py`. |

### As interfaces do simulador

Definidas em `src/culling_games/src/cg_interfaces/`:

```text
# srv/GetMap.srv (pedido vazio; a resposta traz a matriz "achatada" e as dimensões)
---
string[] occupancy_grid_flattened
uint8[<=2] occupancy_grid_shape

# srv/MoveCmd.srv (pedido: a direção; resposta: se andou e as posições)
string direction            # 'up', 'down', 'left' ou 'right'
---
bool success
int8[2] robot_pos
int8[2] target_pos

# srv/Reset.srv (reinicia o labirinto atual, um aleatório ou um pelo nome)
bool is_random
string map_name
---
bool success
string loaded_map_name

# msg/RobotSensors.msg (o que está nas 8 células ao redor do robô)
string up
string down
string left
string right
string up_left
string up_right
string down_left
string down_right
```

"Achatada" quer dizer que a matriz vem como uma lista única, linha após
linha: em um grid 29 × 29, os 29 primeiros itens são a linha 0, os 29
seguintes a linha 1, e assim por diante.

## 4. O simulador por dentro

O código do simulador está em `src/culling_games/src/cg/cg/`. Ele é um
submódulo git: o código é do professor, e eu só o uso.

### 4.1 O labirinto

Os labirintos ficam em `src/culling_games/src/cg/maps/` (`1.csv` a `10.csv`
e `test.csv`). Cada arquivo é um grid 29 × 29 de letras:

```text
b,b,b,b,b,b,b,b,b,b,b,b,...
b,r,b,f,f,f,f,f,f,f,f,f,...
b,f,b,f,b,b,b,b,b,b,b,f,...
```

- A borda é sempre parede.
- O robô começa em (1, 1) e o alvo fica perto do centro.
- Os mapas `1` a `10` são **labirintos perfeitos**: existe exatamente um
  caminho entre quaisquer duas células livres. O `test.csv` é uma sala
  aberta, sem paredes internas.
- `ros2 run cg maze -- --generate` cria um labirinto novo com uma DFS
  aleatória (*recursive backtracker*, em `main.py`), que também gera
  labirintos perfeitos.

**Coordenadas:** sempre `(linha, coluna)`, com (0, 0) no canto superior
esquerdo. `up` diminui a linha, `down` aumenta, `left` diminui a coluna e
`right` aumenta.

### 4.2 Os arquivos

| Arquivo | Papel |
| --- | --- |
| `main.py` | Lê os argumentos (`--map`, `--generate`), escolhe o CSV, cria o `Game` e inicia as duas *threads* (abaixo). |
| `Game.py` | O nó ROS: cria os serviços e o publicador dos sensores, e roda o laço do jogo. |
| `Maze.py` | Guarda a matriz (`occupancy_grid`) e a desenha com o Pygame. |
| `Robot.py` | Acha o `r` na matriz, move o robô e lê as 8 células ao redor. |
| `Utils/` | Ler CSV, achar uma letra na matriz, achatar a matriz, criar a janela. |

### 4.3 Duas *threads* ao mesmo tempo

O simulador faz duas coisas em paralelo:

- **Thread principal: o laço do Pygame** (`Game.run`): repete, sem pausa,
  "processa teclado → redesenha a matriz → atualiza a janela". Também aplica
  um *reset* pendente.
- **Thread do ROS** (`rclpy.spin(game)`): atende os serviços e, a cada 10 ms,
  um *timer* chama `publish_sensor_data`, que publica os sensores.

As duas mexem na mesma matriz. Mover o robô é só editar a matriz: a célula
antiga vira `f`, a nova vira `r`, e o próximo quadro do Pygame desenha o
resultado.

### 4.4 Comportamentos que importam para a solução

1. **Mover:** `/move_command` só recusa (`success = False`) se a célula de
   destino for parede. A resposta traz a posição do robô depois do movimento.
2. **Vitória:** quando o robô entra na célula do alvo, ela vira `r`. O `t`
   some da matriz, a janela mostra uma imagem de vitória e **todos os
   movimentos seguintes são recusados**. Para jogar de novo, é preciso chamar
   `/reset`.
3. **`target_pos` entrega o alvo:** toda resposta de `/move_command` traz a
   posição do alvo, mesmo que o robô nunca o tenha visto. Na parte 2, esse
   campo é ignorado de propósito.
4. **Reset atrasado:** `/reset` só marca um pedido e responde na hora. O laço
   do Pygame aplica o *reset* no próximo quadro, recriando a janela e
   recarregando as imagens de vitória, o que levou **0,6 a 0,9 s** nesta
   máquina. Movimentos feitos nesse meio-tempo vão para o robô antigo e se
   perdem quando o novo aparece na partida.
5. **Os sensores leem só as 8 células vizinhas** (inclusive diagonais), nunca
   a do próprio robô. Os valores possíveis são `f`, `b` e `t`.
6. **O tópico dos sensores depende do relógio do sistema.** O *timer* do ROS
   usa o relógio de parede. Se o relógio voltar para trás, o *timer* espera o
   relógio "alcançar" o horário marcado e os sensores param de publicar,
   enquanto os serviços continuam funcionando. No WSL isso acontece quando
   dois serviços de sincronização de horário brigam (veja o
   [Passo 7](#passo-7-evitar-saltos-de-relógio-no-wsl)).

## 5. O labirinto como grafo

Para usar algoritmos de grafos, o labirinto é modelado assim:

- **Vértices:** cada célula transitável (`f`, `r` ou `t`).
- **Arestas:** ligam duas células transitáveis vizinhas na horizontal ou na
  vertical. Diagonais não são movimentos.
- **Pesos:** todo movimento custa 1. O grafo é **não ponderado**.

Paredes não são vértices, então nenhum caminho passa por elas. O grafo não é
montado em memória: os vizinhos de uma célula são calculados na hora,
olhando as 4 células ao redor.

Dois algoritmos clássicos de busca percorrem esse grafo de jeitos diferentes:

| | **BFS** (busca em largura) | **DFS** (busca em profundidade) |
| --- | --- | --- |
| Estrutura | Fila (o primeiro que entra é o primeiro que sai) | Pilha (o último que entra é o primeiro que sai) |
| Como avança | Em "anéis": todas as células a 1 passo, depois a 2, ... | Vai fundo por um caminho até não dar mais, depois volta |
| Garante o caminho mais curto? | Sim, em grafo não ponderado | Não |
| Precisa "pular" de um lugar para outro? | Sim: a próxima célula da fila pode estar longe | Não: a próxima célula é sempre vizinha da atual (ou o passo de volta) |
| Uso neste projeto | Calcular a rota ótima (partes 1 e 2) | Explorar o labirinto fisicamente (parte 2) |

A última linha da tabela é o motivo de usar os dois: a BFS é ótima para
**calcular** uma rota quando o mapa já está na memória, e a DFS é natural
para **explorar** com um robô de verdade, que só consegue andar para uma
célula vizinha por vez.

### Organização do meu código

```text
src/nav_auto/
├── package.xml, setup.py, setup.cfg, resource/   # arquivos do pacote ROS
├── nav_auto/
│   ├── grid.py         # grid e BFS: Python puro, usado nas duas partes
│   ├── part1_node.py   # nó da parte 1
│   ├── explorer.py     # exploração com DFS: Python puro (parte 2)
│   └── part2_node.py   # nó da parte 2
└── test/
    ├── test_grid.py      # testes da BFS
    └── test_explorer.py  # testes da exploração, nos 11 labirintos reais
```

Os algoritmos (`grid.py`, `explorer.py`) não importam nada do ROS. Os nós
(`part1_node.py`, `part2_node.py`) só fazem a ponte entre o algoritmo e o
simulador. Assim, os algoritmos podem ser testados sem abrir o jogo.

---

## 6. Montando o ambiente

| Item | Versão usada |
| --- | --- |
| Sistema hospedeiro | Windows 11 |
| WSL | 2.7.12 (com WSLg para janelas gráficas) |
| Distribuição Linux | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco (`ros-jazzy-desktop`) |
| Python | 3.12 (padrão do Ubuntu 24.04) |
| Pygame | 2.5.2 (pacote `python3-pygame` do apt) |
| Simulador | [culling_games](https://github.com/rmnicola/culling_games) |

**Por que Ubuntu 24.04?** Cada distribuição do ROS 2 tem suporte oficial
para uma versão do Ubuntu. O ROS 2 Jazzy (LTS, usado na disciplina) roda no
Ubuntu 24.04.

### Passo 1: Instalar o Ubuntu 24.04 no WSL2

No PowerShell (Windows):

```powershell
wsl --install -d Ubuntu-24.04
```

Na primeira execução, o Ubuntu pede para criar um usuário e uma senha. Para
conferir que a distribuição roda em WSL **2** (coluna `VERSION`):

```powershell
wsl -l -v
```

> **Atenção:** se houver outra distribuição instalada (por exemplo, um
> `Ubuntu` genérico, sem ROS), sempre abra a correta com
> `wsl -d Ubuntu-24.04`, ou defina-a como padrão com
> `wsl --set-default Ubuntu-24.04`.

### Passo 2: Configurar o locale UTF-8

O ROS 2 exige um locale com UTF-8. Dentro do Ubuntu:

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

### Passo 3: Adicionar o repositório do ROS 2

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

O pacote `ros2-apt-source` configura a chave e o repositório oficiais do
ROS 2.

### Passo 4: Instalar o ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

- `ros-jazzy-desktop`: o ROS 2 completo (inclui o `rclpy`, a biblioteca de
  Python).
- `ros-dev-tools`: ferramentas de desenvolvimento, como `colcon` e `rosdep`.

### Passo 5: Instalar as dependências do projeto

```bash
sudo apt install -y python3-pygame python3-colcon-common-extensions python3-pytest git
```

> Usei o Pygame do `apt` (e não do `pip`) porque o Ubuntu 24.04 bloqueia
> `pip install` no Python do sistema, e o ROS 2 usa esse Python.

### Passo 6: Carregar o ROS 2 em cada terminal

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
ros2 --help    # teste
```

### Passo 7: Evitar saltos de relógio no WSL

O WSL acerta o relógio do Linux pelo relógio do Windows. Se o Ubuntu também
rodar o próprio serviço de horário (`systemd-timesyncd`) e o relógio do
Windows estiver adiantado ou atrasado, os dois brigam: o relógio do Linux
pula para frente e para trás a cada poucos segundos. Isso congela o tópico
dos sensores do simulador (seção 4.4, item 6).

Como o WSL já sincroniza o relógio, desative o serviço do Ubuntu:

```bash
sudo systemctl disable --now systemd-timesyncd
```

Para desfazer: `sudo systemctl enable --now systemd-timesyncd`.

Para conferir se o relógio está estável (deve mostrar `saltos: 0`):

```bash
python3 -c '
import time
b = time.time() - time.monotonic(); n = 0
for _ in range(40):
    time.sleep(0.25); o = time.time() - time.monotonic()
    n += abs(o - b) > 0.01; b = o
print("saltos:", n)'
```

### Passo 8: Clonar este repositório

O repositório fica em uma pasta do **Windows** (editada pelo VS Code) e é
compilado e executado **dentro do WSL**, que enxerga os discos do Windows em
`/mnt/c/`.

No PowerShell, na pasta onde o projeto deve ficar:

```powershell
git clone --recurse-submodules https://github.com/antoniodicillo/desafio_nav_auto.git
cd desafio_nav_auto
git config core.autocrlf false
git -C src/culling_games config core.autocrlf false
```

> **Por que desativar o `autocrlf`?** O Git no Windows converte as quebras de
> linha para o formato do Windows (CRLF). O código roda no Linux, que espera
> LF; arquivos com CRLF quebram scripts e o build. O `.gitattributes` deste
> repositório já força LF nos meus arquivos; o comando acima garante o
> mesmo dentro do submódulo.

No Ubuntu, entre na mesma pasta pelo caminho `/mnt/c/...`. Neste computador:

```bash
cd /mnt/c/Users/anton/OneDrive/Documentos/GitHub/desafio_nav_auto
```

> **Atenção:** compilar em `/mnt/c` é mais lento que no sistema de arquivos
> do Linux (`~/`), e o OneDrive tenta sincronizar `build/`, `install/` e
> `log/`. Se o build ficar lento ou der erro de arquivo travado, pause o
> OneDrive durante o build.

O simulador entra como **submódulo git** em `src/culling_games`, separado do
meu código. Para atualizá-lo:

```bash
git submodule update --remote src/culling_games
```

### Passo 9: Compilar o workspace

Na raiz do repositório:

```bash
colcon build
source install/setup.bash
```

O `colcon` encontra sozinho os pacotes `cg`, `cg_interfaces` e `cg_teleop`
(em `src/culling_games/src/`) e o `nav_auto` (em `src/nav_auto`). Depois de
mudar o código do `nav_auto`, basta recompilar só ele:

```bash
colcon build --packages-select nav_auto
```

> Se você **mover ou renomear a pasta** do repositório, apague `build/`,
> `install/` e `log/` e compile de novo: o `colcon` grava caminhos absolutos
> nessas pastas.

### Passo 10: Testar o simulador

Terminal 1:

```bash
source install/setup.bash
ros2 run cg maze                  # labirinto aleatório
ros2 run cg maze -- --map 1.csv   # um labirinto específico
```

Uma janela do Pygame com o labirinto abre no Windows (graças ao WSLg).

Terminal 2:

```bash
source install/setup.bash
ros2 run cg_teleop teleop_keyboard
```

Use `w`/`a`/`s`/`d` para mover o robô. Para ver os sensores (com o mesmo
perfil de QoS do publicador):

```bash
ros2 topic echo --qos-profile sensor_data /culling_games/robot_sensors
```

### Passo 11: Rodar os testes unitários

Os testes não precisam do simulador aberto:

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test
```

Resultado esperado: `16 passed`.

## 7. Verificação do ambiente

Testes feitos nesta máquina (24/11 e 26/11/2026):

| Teste | Resultado |
| --- | --- |
| `colcon build` (`cg_interfaces`, `cg`, `cg_teleop`, `nav_auto`) | OK (~3 min em `/mnt/c`, dentro do OneDrive) |
| `ros2 run cg maze -- --map test.csv` | OK: janela do Pygame abre via WSLg |
| Serviços `/move_command`, `/reset`, `/get_map` | OK |
| `/move_command` com `direction: right` | OK: `success=True`, `robot_pos=[1, 2]`, `target_pos=[13, 14]` |
| `/get_map` | OK: grade 29 × 29 |
| Tópico `/culling_games/robot_sensors` | 100 mensagens/s, como o código pede, enquanto o relógio está estável. Com o relógio pulando (Passo 7), o tópico para depois de 2 a 4 s. |
| Testes unitários | `16 passed` |

Avisos esperados, que podem ser ignorados:

- `Clock skew detected` durante o build: diferença de relógio entre o Windows
  e o WSL ao compilar em `/mnt/c`.
- `pygame was not built with support for avx2`: afeta só o desempenho do
  desenho.

## 8. Editando pelo VS Code

Como o projeto está em uma pasta do Windows, basta abri-lo no VS Code. Para
compilar e rodar, use um terminal do Ubuntu dentro do VS Code (menu do
terminal → **Ubuntu-24.04 (WSL)**) ou abra o Ubuntu e entre na pasta pelo
caminho `/mnt/c/...`.

## 9. Problemas que enfrentei

| Sintoma | Causa provável | Solução |
| --- | --- | --- |
| `ros2: command not found` | ROS não foi carregado, ou o terminal está na distribuição errada | `source /opt/ros/jazzy/setup.bash`; confira se está no `Ubuntu-24.04` |
| `Package 'cg' not found` | Workspace não foi carregado | `source install/setup.bash` na raiz do repositório |
| `The build time path ".../install/cg_interfaces" doesn't exist` | A pasta do repositório foi movida ou renomeada depois do build | `rm -rf build install log`, `colcon build` e `source install/setup.bash` em um terminal novo |
| Janela do Pygame abre, mas fica em branco | O WSLg travou | `wsl --shutdown` no PowerShell e abrir o Ubuntu de novo |
| Janela do Pygame não abre | WSLg desativado ou WSL desatualizado | `wsl --update` e `wsl --shutdown` no PowerShell |
| Os sensores param de publicar depois de alguns segundos (a parte 2 acusa "Os sensores pararam de publicar"), mas os serviços funcionam | O relógio do WSL está pulando (dois serviços de horário brigando) | [Passo 7](#passo-7-evitar-saltos-de-relógio-no-wsl) e reiniciar o jogo |
| Duas instâncias do jogo respondendo ao mesmo tempo (movimentos e sensores sem sentido) | Um `ros2 run cg maze` antigo continua rodando em outro terminal | Fechar todos os jogos (`pkill -f lib/cg/maze`) e abrir só um |
| `colcon build` muito lento ou erro de arquivo travado | OneDrive sincronizando `build/`, `install/` ou `log/` | Pausar o OneDrive durante o build |
| `/usr/bin/env: 'python3\r': No such file or directory` | Arquivos com quebra de linha CRLF | `git config core.autocrlf false` e refazer o checkout (Passo 8) |
| `src/culling_games` vazio | Clonou sem os submódulos | `git submodule update --init` |
| A parte 1 diz "Alvo não encontrado no mapa" | O jogo já foi vencido (o alvo some) | `ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"` |
