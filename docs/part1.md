# Parte 1: Navegação com mapa

**Objetivo:** com acesso ao mapa completo do labirinto, encontrar a rota
otimizada (com o menor número de movimentos) do robô até o alvo e executá-la.

**Em uma linha:** pede o mapa -> BFS acha o caminho mais curto -> converte o
caminho em `up`/`down`/`left`/`right` -> manda um movimento por vez e confere
cada um.

Para os conceitos de ROS e o funcionamento do simulador, veja
[setup.md](setup.md).

## Sumário

- [Como executar](#como-executar)
- [Passo a passo do que acontece](#passo-a-passo-do-que-acontece)
- [Por que BFS?](#por-que-bfs)
- [O código, função por função](#o-código-função-por-função)
- [Resultados](#resultados)
- [Testes unitários](#testes-unitários)

## Como executar

Terminal 1 (o jogo):

```bash
source install/setup.bash
ros2 run cg maze                    # labirinto aleatório
ros2 run cg maze -- --map 1.csv     # ou um labirinto específico
```

Terminal 2 (a solução):

```bash
colcon build --packages-select nav_auto
source install/setup.bash
ros2 run nav_auto part1
```

O parâmetro `step_delay` (padrão `0.05` s) é a pausa entre movimentos, só
para dar tempo de ver o robô andando. Para executar sem pausa:

```bash
ros2 run nav_auto part1 --ros-args -p step_delay:=0.0
```

> Se o jogo já tiver sido vencido, o alvo some do mapa e o nó avisa. Reinicie
> o labirinto com
> `ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"`.

## Passo a passo do que acontece

**0. Esperar o jogo.** Se o simulador ainda não estiver rodando, o nó
printa "Esperando o serviço..." a cada segundo e não faz mais nada.

**1. Pedir o mapa.** O `/get_map` devolve a matriz inteira como uma lista
única, linha após linha, mais as dimensões:

```text
flat  = ['b','b','b', ... (29 itens da linha 0) ..., 'b','r','b','f', ...]
shape = [29, 29]
```

O nó corta essa lista de volta em linhas de 29 e procura `r` (partida) e
`t` (alvo). Se não houver `t`, o jogo já foi vencido: o robô está em cima do
alvo. O nó avisa e para.

**2. Achar o caminho mais curto com BFS.** Explicado na próxima seção.

**3. Converter em comandos.** Para cada par de células consecutivas do
caminho, a diferença de posição vira um comando:

```text
(1,1) -> (2,1)   linha +1    -> 'down'
(9,3) -> (9,4)   coluna +1   -> 'right'
```

**4. Executar.** Para cada comando, o nó chama `/move_command`, recebe a
nova posição do robô e confere se ela é a célula prevista pelo caminho. Se
um movimento falhar, o nó para e avisa, em vez de seguir com um plano
errado. Depois do último passo o robô está no alvo e a janela mostra a
imagem de vitória.

## Por que BFS?

O labirinto é um grafo não ponderado (veja
[setup.md, seção 5](setup.md#5-o-labirinto-como-grafo)): cada célula
transitável é um vértice, células vizinhas são ligadas por arestas e todo
movimento custa 1.

A **busca em largura** (BFS) explora esse grafo em "anéis", como uma onda na
água: primeiro todas as células a 1 movimento da partida, depois todas a 2,
e assim por diante. Ela usa uma **fila** (o primeiro que entra é o primeiro
que sai) para garantir essa ordem.

Consequência: **a primeira vez que a BFS chega ao alvo, ela chegou pelo
menor número possível de movimentos.** Em um grafo não ponderado, a BFS
sempre encontra o caminho mais curto, que é exatamente a "rota otimizada"
pedida.

A BFS processa todas as células de distância 1 antes das de distância 2, e
assim por diante. Quando ela tira o `T` da fila, a distância dele (6) é a
menor possível.

### O algoritmo

```text
fila      <- [início]
veio_de   <- {início: nenhum}          # também serve como "visitados"

enquanto a fila não estiver vazia:
    célula <- remove o primeiro da fila
    se célula é o alvo: reconstrói o caminho e termina
    para cada vizinho transitável da célula:
        se vizinho ainda não foi visitado:
            veio_de[vizinho] <- célula
            coloca vizinho no fim da fila

se a fila esvaziar sem achar o alvo: não existe caminho
```

**Reconstrução do caminho:** `veio_de` guarda, para cada célula, de qual
célula a BFS chegou nela. Partindo do alvo e seguindo `veio_de` até a
partida, obtemos o caminho de trás para frente; basta invertê-lo. No exemplo
acima:

```text
alvo (3,1) <- (3,2) <- (3,3) <- (2,3) <- (1,3) <- (1,2) <- (1,1) partida
caminho = [(1,1), (1,2), (1,3), (2,3), (3,3), (3,2), (3,1)]
comandos = right, right, down, down, left, left
```

### Complexidade

- **Tempo:** O(V + E). Cada célula entra na fila no máximo uma vez, e cada
  ligação é examinada no máximo duas vezes. No grid, cada célula tem no
  máximo 4 vizinhos (E ≤ 4V), então o tempo é O(V).
- **Memória:** O(V), para a fila e o dicionário `veio_de`.

Um labirinto 29 × 29 tem 841 células; a BFS roda em menos de 1 ms.

### Comparação com outros algoritmos

| Algoritmo | Acha o caminho mais curto aqui? | Observação |
| --- | --- | --- |
| **BFS** | Sim | Ideal para grafos não ponderados. Escolhido. |
| DFS | Não | Acha *um* caminho, não necessariamente o mais curto. É útil para **explorar** (parte 2). |
| Dijkstra | Sim | Generaliza a BFS para arestas com pesos diferentes. Com todos os pesos iguais a 1, faz o mesmo que a BFS, com o custo extra de uma fila de prioridade. |
| A* | Sim, com heurística admissível (ex.: distância de Manhattan) | Usa uma estimativa da distância até o alvo para examinar menos células. Mesmo resultado, mais complexo; a diferença é irrelevante em um grid deste tamanho. |

## O código, função por função

Esta seção cobre todos os arquivos do pacote `nav_auto` usados na parte 1.
Os arquivos da parte 2 (`explorer.py`, `part2_node.py`, `test_explorer.py`)
estão em [part2.md](part2.md#o-código-função-por-função).

### `nav_auto/grid.py`: o grid e a BFS
Python puro, sem ROS. É usado pela parte 1 e reaproveitado pela parte 2. 
[código](../src/nav_auto/nav_auto/grid.py)

#### Constantes

| Nome | Valor | Para que serve |
| --- | --- | --- |
| `FREE`, `BLOCKED`, `ROBOT`, `TARGET` | `'f'`, `'b'`, `'r'`, `'t'` | Os mesmos códigos de célula do simulador. |
| `UNKNOWN` | `'?'` | Célula que o robô ainda não viu (só aparece na parte 2). |
| `PASSABLE` | `{FREE, ROBOT, TARGET}` | Células por onde o robô pode passar. `UNKNOWN` fica de fora: na parte 2, o robô só planeja por onde já viu. |
| `DIRECTIONS` | `{'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)}` | Cada comando e o deslocamento (linha, coluna) que ele causa. A ordem do dicionário é a ordem em que a BFS examina os vizinhos; quando existem vários caminhos mínimos, ela decide qual é encontrado. |

#### `grid_from_flat(flat, shape)`

Reconstrói a matriz a partir da resposta do `/get_map`.

- **Recebe:** `flat`, a lista achatada (linha após linha), e `shape`,
  `[linhas, colunas]`.
- **Faz:** confere se `len(flat) == linhas × colunas` (senão, `ValueError`)
  e corta a lista em fatias de `colunas` itens: a linha `r` é
  `flat[r*colunas : (r+1)*colunas]`.
- **Devolve:** lista de listas, `grid[linha][coluna]`.

#### `find_cell(grid, value)`

Percorre a matriz linha por linha e devolve `(linha, coluna)` da primeira
célula igual a `value`, ou `None` se não houver nenhuma. Usada para achar o
robô (`'r'`) e o alvo (`'t'`).

#### `is_passable(grid, cell)`

Devolve `True` se `cell` está dentro da matriz **e** é transitável (está em
`PASSABLE`). A checagem de limites evita erro de índice ao olhar vizinhos de
células na borda.

#### `neighbors(grid, cell)`

Gerador: para cada direção de `DIRECTIONS`, calcula a célula vizinha e, se
ela for transitável, produz o par `(direção, vizinha)`. É aqui que o grafo
"existe": as arestas não ficam guardadas em lugar nenhum, são calculadas na
hora.

#### `bfs(grid, start, goal)`

A busca em largura descrita acima.

- **Recebe:** a matriz, a célula de partida e a do alvo.
- **Estruturas:** `queue`, uma `deque` (fila com inserção e remoção O(1) nas
  pontas), e `came_from`, um dicionário `célula -> célula anterior`. A
  partida entra com `None`. O dicionário serve também de conjunto de
  visitados: uma célula só entra na fila se ainda não estiver nele.
- **Laço:** tira a primeira célula da fila (`popleft`). Se for o alvo,
  reconstrói o caminho. Senão, para cada vizinho transitável ainda não
  visitado, anota de onde veio e o põe no fim da fila.
- **Devolve:** a lista de células do caminho, da partida ao alvo (inclusive),
  ou `None` se a fila esvaziar sem achar o alvo.

#### `_rebuild_path(came_from, goal)`

Começa em `[goal]` e vai acrescentando `came_from` da última célula até
chegar na partida (cujo `came_from` é `None`). Inverte a lista e a devolve.
O `_` no nome indica que é uma função auxiliar, só usada pela `bfs`.

#### `path_to_moves(path)`

Converte o caminho em comandos. Monta o dicionário inverso de `DIRECTIONS`
(`(-1, 0) -> 'up'`, ...) e, para cada par de células consecutivas, calcula a
diferença `(l2 - l1, c2 - c1)` e procura o comando correspondente. Um
caminho com N células gera N − 1 comandos.

#### `render(grid, path=())`

Desenha a matriz como texto para o terminal: `|` parede, espaço livre, `R`
robô, `T` alvo, `?` desconhecido. Células livres que estão em `path` viram
`·`, mostrando a rota. Devolve uma string com uma linha de texto por linha da
matriz.

### `nav_auto/part1_node.py`: o nó da parte 1

Faz a ponte entre a BFS e o simulador. [código](../src/nav_auto/nav_auto/part1_node.py)

#### `class Part1Navigator(Node)`

O nó ROS. Herdar de `rclpy.node.Node` dá acesso a parâmetros, *logger* e
clientes de serviço.

#### `__init__(self)`

- Cria o nó com o nome `part1_navigator`.
- Declara o parâmetro `step_delay` (padrão `0.05` s) e lê seu valor.
- Cria dois **clientes de serviço**: um para `/get_map` (tipo `GetMap`) e um
  para `/move_command` (tipo `MoveCmd`). Criar o cliente não chama nada
  ainda; só prepara a conexão.

#### `call(self, client, request)`

Chama um serviço e espera a resposta. `call_async` envia o pedido e devolve
um *future* (uma "promessa" de resposta). `rclpy.spin_until_future_complete`
roda o laço do ROS até a resposta chegar. Devolve a resposta.

#### `wait_for_services(self)`

Para cada cliente, espera até 1 s o serviço aparecer na rede; enquanto ele
não aparecer, printa "Esperando o serviço..." e tenta de novo. Evita chamar
um serviço que ainda não existe, se o jogo for aberto depois.

#### `run(self)`

O roteiro da parte 1, em três etapas. Devolve `True` se o robô chegou ao
alvo e `False` se algo deu errado.

1. **Mapa:** chama `/get_map`, reconstrói a matriz com `grid_from_flat` e
   acha partida e alvo com `find_cell`. Sem alvo, avisa que o jogo já foi
   vencido e devolve `False`.
2. **Planejamento:** roda `bfs`, medindo o tempo com `time.perf_counter`.
   Sem caminho, devolve `False`. Converte o caminho com `path_to_moves`,
   registra o número de movimentos e printa o labirinto com `render`.
3. **Execução:** para cada comando, monta um `MoveCmd.Request`, chama
   `/move_command` e compara `robot_pos` da resposta com a célula esperada
   (`path[i + 1]`). Se o movimento falhar ou o robô estiver em outro lugar,
   registra o erro e devolve `False`. Entre os passos, dorme `step_delay`.

O campo `target_pos` da resposta não é usado: a parte 1 já sabe onde está o
alvo pelo mapa.

#### `main(args=None)`

O ponto de entrada (`ros2 run nav_auto part1`). Inicializa o ROS
(`rclpy.init`), cria o nó e chama `run()`. `Ctrl+C` (`KeyboardInterrupt`)
encerra sem mostrar erro. O bloco `finally` sempre destrói o nó e desliga o
ROS, mesmo se algo falhar no meio.

### `test/test_grid.py`: testes da BFS

Usa um labirinto pequeno (5 x 7) com dois caminhos de mesmo tamanho até o alvo. [código](../src/nav_auto/test/test_grid.py)

| Teste | O que confere |
| --- | --- |
| `test_grid_from_flat_rebuilds_rows` | Achatar `MAZE` e reconstruir devolve a matriz original. |
| `test_bfs_finds_shortest_path` | O caminho começa na partida, termina no alvo e tem 6 movimentos, o mínimo. |
| `test_path_to_moves` | `[(1,1), (2,1), (3,1), (3,2)]` vira `['down', 'down', 'right']`. |
| `test_bfs_returns_none_when_unreachable` | Com o alvo cercado de paredes, a BFS devolve `None`. |
| `test_bfs_does_not_walk_through_walls` | Nenhuma célula do caminho é parede. |

A função auxiliar `make_grid(rows)` transforma uma lista de strings em
matriz (lista de listas de letras).

### Arquivos do pacote ROS

| Arquivo | Para que serve |
| --- | --- |
| [`package.xml`](../src/nav_auto/package.xml) | Identifica o pacote para o ROS: nome, versão, licença e dependências (`rclpy`, `cg_interfaces`; `python3-pytest` para testes). `build_type` `ament_python` diz que é um pacote Python. |
| [`setup.py`](../src/nav_auto/setup.py) | Instalação do pacote Python. A parte importante são os `entry_points`: `part1 = nav_auto.part1_node:main` cria o executável `part1`, que chama `main()` de `part1_node.py` (e o mesmo para `part2`). Também instala o `package.xml` e o marcador do índice do ROS. |
| [`setup.cfg`](../src/nav_auto/setup.cfg) | Diz para instalar os executáveis em `lib/nav_auto`, que é onde o `ros2 run` procura. |
| `resource/nav_auto` | Arquivo vazio. Sua presença no índice do ROS (`share/ament_index`) é o que faz o `ros2 run` achar o pacote. |
| `nav_auto/__init__.py` | Arquivo vazio que torna a pasta `nav_auto` um pacote Python importável (`from nav_auto.grid import ...`). |

## Resultados

Testado em 24/11/2026 (e de novo em 26/11 no `1.csv`) no ambiente descrito
em [setup.md](setup.md):

| Labirinto | Tamanho | Partida -> Alvo | Movimentos da rota | Tempo da BFS | Resultado |
| --- | --- | --- | --- | --- | --- |
| `test.csv` (sala aberta) | 29 × 29 | (1, 1) -> (13, 14) | 25 | < 1 ms | Alvo alcançado |
| `1.csv` | 29 × 29 | (1, 1) -> (14, 14) | 102 | 0,24 ms | Alvo alcançado |
| `7.csv` | 29 × 29 | (1, 1) -> (14, 14) | 96 | 0,20 ms | Alvo alcançado |
| `--generate` (aleatório) | 29 × 29 | (1, 1) -> (14, 14) | 36 | 0,14 ms | Alvo alcançado |

No `test.csv`, que não tem paredes internas, a rota de 25 movimentos é igual
à distância de Manhattan entre partida e alvo (12 linhas + 13 colunas), o
menor valor possível. Isso confirma, em um caso fácil de verificar à mão, que
a rota encontrada é ótima.

## Testes unitários

Os testes rodam sem o simulador:

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test/test_grid.py
```

Resultado: `5 passed`. O que cada teste confere está na tabela da seção
[`test/test_grid.py`](#testtest_gridpy-testes-da-bfs).
