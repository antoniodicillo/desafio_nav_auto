# Parte 2: Mapeamento do labirinto

**Objetivo:** sem acesso ao mapa, o robô deve navegar pelo labirinto usando
os sensores, construir o mapa no processo e, depois, provar que o mapa
construído é suficiente para reproduzir a rota ótima da parte 1.

**Em uma linha:** explora com DFS lendo os sensores, terminando de volta na
partida → BFS no mapa construído → executa a rota → só no fim pede o mapa real
para conferir que a rota é a mesma da parte 1.

Para os conceitos de ROS e o funcionamento do simulador, veja
[setup.md](setup.md). A BFS, reaproveitada aqui, está explicada em
[part1.md](part1.md#por-que-bfs).

## Sumário

- [Como executar](#como-executar)
- [Passo a passo do que acontece](#passo-a-passo-do-que-acontece)
- [A exploração em detalhe](#a-exploração-em-detalhe)
- [Por que o mapa construído basta](#por-que-o-mapa-construído-basta)
- [A conferência final](#a-conferência-final)
- [Por que DFS?](#por-que-dfs)
- [O código, função por função](#o-código-função-por-função)
- [Resultados](#resultados)
- [Testes unitários](#testes-unitários)
- [Limitações](#limitações)

## Como executar

> **Antes:** o relógio do WSL precisa estar estável, senão o simulador para
> de publicar os sensores depois de alguns segundos. Veja o
> [Passo 7 do setup](setup.md#passo-7-evitar-saltos-de-relógio-no-wsl).

Terminal 1 (o jogo, **com a janela**):

```bash
source install/setup.bash
ros2 run cg maze -- --map 1.csv
```

Terminal 2 (a solução):

```bash
colcon build --packages-select nav_auto
source install/setup.bash
ros2 run nav_auto part2
```

O nó reinicia o labirinto sozinho no começo, então dá para rodá-lo várias
vezes seguidas, mesmo depois de uma vitória (por exemplo, logo depois da
parte 1).

## Passo a passo do que acontece

```text
Part2Mapper                                     Simulador (cg maze)
     │                                                 │
 0.  │ ── /reset (mesmo labirinto) ───────────────────►│
     │  espera 3 s (o jogo recria o labirinto)         │
     │                                                 │
 1.  │  EXPLORAÇÃO (DFS)                               │
     │   repete até a pilha esvaziar:                  │
     │   ◄── /culling_games/robot_sensors ─────────────│   8 células ao redor
     │   anota no mapa; escolhe uma vizinha nova       │
     │   ── /move_command {direction} ────────────────►│
     │   (num beco sem saída, volta um passo)          │
     │   termina de volta na partida                   │
     │                                                 │
 2.  │  ROTA: BFS partida → alvo no mapa construído    │
     │   ── /move_command ... ────────────────────────►│   o robô chega ao alvo
     │                                                 │
 3.  │  CONFERÊNCIA                                    │
     │   ── /get_map (a única vez) ───────────────────►│
     │   compara mapas e rotas                         │
```

O `/get_map` só é chamado na etapa 3, depois que a rota já foi calculada e
executada. Ele não influencia em nada a navegação; serve só para provar que
o resultado está certo.

## A exploração em detalhe

A lógica da exploração fica em
[`explorer.py`](../src/nav_auto/nav_auto/explorer.py), em Python puro. O nó
ROS só entrega a ela duas funções: uma que lê os sensores e outra que move o
robô.

### O que o robô sabe

O robô não sabe onde está no labirinto. Ele usa **coordenadas relativas**: a
célula onde começou é `(0, 0)`, e cada movimento bem-sucedido soma o
deslocamento da direção. Isso é **odometria**: saber a posição contando os
próprios passos.

O mapa é um dicionário `célula → valor`, que começa vazio e cresce conforme o
robô vê coisas. Uma célula que não está no dicionário é desconhecida. Isso
evita ter que saber o tamanho do labirinto de antemão.

### Sentir: os 8 sensores

Ao chegar em uma célula nova, o robô lê o tópico
`/culling_games/robot_sensors`, que diz o que há nas 8 células ao redor:

```text
 up_left │   up   │ up_right           b │ f │ b
─────────┼────────┼─────────          ───┼───┼───
  left   │ (robô) │  right       →     b │ R │ f
─────────┼────────┼─────────          ───┼───┼───
down_left│  down  │down_right          b │ b │ b
```

Cada uma das 8 células é anotada no mapa (`f`, `b` ou `t`), e a célula do
próprio robô é anotada como livre. Quando um sensor mostra `t`, a posição do
alvo fica guardada.

### Andar: DFS com uma pilha

A busca em profundidade (DFS) usa uma **pilha** com o caminho da partida até
a posição atual:

1. Se alguma vizinha direta (cima, baixo, esquerda, direita) é livre e ainda
   não foi visitada, o robô **anda para ela**, empilha a célula e lê os
   sensores lá.
2. Se não há nenhuma (beco sem saída, ou tudo em volta já visitado), o robô
   **desempilha** a célula atual e **volta um passo** para a célula que ficou
   no topo da pilha.
3. Quando a pilha esvazia, a exploração acabou, e o robô está de volta na
   partida.

Exemplo em um labirinto pequeno (`R` partida, `T` alvo, `|` parede). As
vizinhas são testadas na ordem de `DIRECTIONS`: cima, baixo, esquerda,
direita. Este é o rastro real do código nesse labirinto:

```text
|||||||        Passo  Movimento  Chega em   O que acontece
|R  | |          1    right      (1,2)      célula nova
||| | |          2    right      (1,3)      célula nova
|T    |          3    down       (2,3)      célula nova
|||||||          4    down       (3,3)      célula nova
                 5    left       (3,2)      célula nova; o sensor da esquerda vê o T
                 6    right      (3,3)      beco sem saída (o T não conta): volta
                 7    right      (3,4)      célula nova
                 8    right      (3,5)      célula nova
                 9    up         (2,5)      célula nova
                10    up         (1,5)      célula nova; beco sem saída
             11-18    volta      ...        desempilha até a partida (1,1): fim
```

São 10 células visitadas e 18 movimentos, ou seja, 2 × (10 − 1): cada
ligação entre células é percorrida uma vez na ida e uma na volta. Depois
disso, a rota final tem 6 movimentos (`right, right, down, down, left,
left`), a mesma que a BFS acha no mapa real.

Na prática, o robô percorre cada corredor indo e voltando, como quem
explora uma caverna com um fio amarrado na entrada.

### Por que a DFS visita tudo

A DFS só desempilha uma célula quando ela não tem mais vizinhas livres sem
visitar. Pegue qualquer célula livre alcançável e um caminho da partida até
ela: a primeira célula não visitada desse caminho seria vizinha de uma célula
visitada, que só sairia da pilha depois de visitá-la. Logo, quando a pilha
esvazia, todas as células alcançáveis foram visitadas.

O robô explora tudo mesmo depois de ver o alvo. Parar antes seria mais
rápido, mas num labirinto com ciclos o caminho mais curto até o alvo pode
passar por células que ele ainda não viu. Explorar tudo garante que o mapa
contém a rota ótima.

### Não pisar no alvo

Quando o robô entra no alvo, o jogo acaba: o `t` some e todos os movimentos
seguintes são recusados. Por isso, durante a exploração, o alvo **não conta
como célula livre**: ele fica no mapa como `t`, e o robô só anda para
células `f`. O alvo é anotado assim que um sensor o vê (da célula ao lado),
mas só a rota final entra nele.

### Leituras velhas dos sensores

O simulador publica os sensores a cada 10 ms, sem dizer de qual posição é a
leitura. O ROS guarda as mensagens que chegam numa fila (até 5, no perfil
`sensor_data`), e o nó só as processa quando roda o *spin*. Quando a
resposta de um movimento chega, a fila pode ainda ter leituras de antes do
movimento, que descrevem a posição antiga. Usá-las estragaria o mapa. Esse
erro apareceu nos testes reais: o mapa ganhou uma célula livre que era
parede, e o simulador recusou o movimento.

Por isso, `Part2Mapper.sense` faz duas coisas:

1. **Esvazia a fila:** processa tudo o que já estava nela, sem usar nada.
2. **Espera mensagens novas** (`FRESH_MESSAGES = 2`): a primeira ainda pode
   ter sido publicada pouco antes do movimento e estar a caminho; a segunda
   já é de depois do movimento.

O robô só lê os sensores quando chega em uma célula **nova**. Nos passos de
volta, ele já conhece tudo em volta, então não espera os sensores e anda
mais rápido.

### O reset demora

Para começar do zero, o nó chama `/reset` antes de explorar. O simulador
responde na hora, mas só aplica o *reset* no próximo quadro do Pygame, e
aplicar é lento: ele recria a janela e recarrega as 18 imagens de vitória.
Medido nesta máquina: **0,6 a 0,9 s**. Movimentos feitos nesse meio-tempo vão
para o robô antigo e se perdem quando o novo robô aparece na partida.

Nos testes reais, com uma espera de 0,5 s, o robô explorou umas 12 células e
foi "teletransportado" de volta para a partida no meio da exploração. O
simulador não avisa quando terminou o *reset*, então o nó espera
`RESET_WAIT = 3.0` segundos, cerca de 3× o pior caso medido.

### A posição real só serve para conferir

A resposta do `/move_command` traz a posição do robô no simulador
(`robot_pos`). No primeiro movimento, o nó a usa para descobrir onde fica a
partida no simulador (`origin`). Isso só é usado na conferência final, para
comparar o mapa construído com o real, nunca para decidir para onde ir.

O campo `target_pos` da mesma resposta entrega a posição do alvo, mesmo que o
robô nunca o tenha visto. Ele é **ignorado de propósito**: a parte 2 precisa
achar o alvo pelos sensores.

## Por que o mapa construído basta

Depois da exploração, o robô roda a mesma `bfs()` da parte 1 no mapa
construído, convertido em matriz no mesmo formato do `/get_map`:
`?` nas células nunca vistas e `r` na partida. Células `?` não são
transitáveis, então a rota só passa por onde o robô viu.

**Afirmação:** essa rota é a mesma que a parte 1 calcula com o mapa real.

**Por quê:**

1. A exploração visitou **todas** as células alcançáveis sem passar pelo alvo
   (seção [Por que a DFS visita tudo](#por-que-a-dfs-visita-tudo)). Cada uma
   foi vista e anotada corretamente, assim como suas 4 vizinhas. Logo, o mapa
   construído tem exatamente as mesmas células e ligações que o mapa real
   nessa região.
2. Células que só seriam alcançáveis **passando pelo alvo** podem ter ficado
   de fora. Elas não importam: qualquer caminho que passe por elas teria que
   passar pelo alvo antes, e a BFS para assim que chega ao alvo.
3. A BFS examina os vizinhos sempre na mesma ordem (`DIRECTIONS`). Com o
   mesmo grafo e a mesma ordem, ela visita as células na mesma sequência e
   encontra **o mesmo caminho**, e não só um caminho do mesmo tamanho.

O teste `test_explores_every_simulator_map` confere isso nos 11 labirintos
do simulador, e a conferência final confere de novo no simulador de
verdade.

## A conferência final

Depois de chegar ao alvo, o nó chama `/get_map` pela primeira e única vez e
confere duas coisas:

| Checagem | Como |
| --- | --- |
| O mapa está certo | Cada célula conhecida, transladada por `origin`, deve ser parede no mapa real se e somente se for parede no meu (`compare_with_real`). Deve dar 0 diferenças. |
| A rota é a da parte 1 | BFS no mapa real, da partida ao alvo (o que a parte 1 faz), comparada com a minha rota. Devem ser idênticas. |

A linha final do log mostra as duas, por exemplo:
`Conferência: 0 células diferentes do mapa real; rota da parte 1: 102 movimentos; rota da parte 2: 102 movimentos; idênticas`.

## Por que DFS?

A BFS e a DFS gastam o mesmo tempo de **cálculo**: as duas visitam cada
célula uma vez. A diferença está nos **movimentos do robô**. Explorar em
ordem de BFS obriga o robô a atravessar o labirinto a cada célula, porque a
próxima célula da fila costuma estar em outro corredor. Na DFS, a próxima
célula é sempre vizinha (um passo à frente ou um passo de volta).

Movimentos para visitar todas as células, medidos nos labirintos do
simulador:

| Labirinto | Explorando em ordem de BFS | Explorando com DFS |
| --- | --- | --- |
| `1.csv` | 4162 | 780 (5× menos) |
| `7.csv` | 8258 | 780 (11× menos) |
| `test.csv` (sala aberta) | 2704 | 1454 (2× menos) |

Por isso cada algoritmo é usado onde ganha: DFS para **explorar** e BFS no
mapa pronto para **calcular** a rota mais curta.

Outras estratégias e seus problemas:

| Estratégia | Problema |
| --- | --- |
| Seguir a parede (mão direita) | Simples, mas pode dar voltas para sempre num labirinto com ciclos e não garante visitar todas as células. |
| Parar ao ver o alvo | Mais rápido, mas o mapa pode não conter a rota ótima num labirinto com ciclos. |

**Custo da DFS:** cada ligação entre células é percorrida exatamente duas
vezes (ida e volta), então são 2 × (V − 1) movimentos, com V células
alcançáveis. Os sensores são lidos uma vez por célula (V leituras).

## O código, função por função

### `nav_auto/explorer.py`: a exploração

[Código](../src/nav_auto/nav_auto/explorer.py). Python puro, sem ROS: recebe
`sense` e `move` como funções, então roda igual no nó e nos testes.

#### Constantes e funções auxiliares

| Nome | O que é |
| --- | --- |
| `SENSOR_OFFSETS` | Cada campo da mensagem `RobotSensors` e o deslocamento (linha, coluna) da célula que ele lê. Ex.: `'up_left': (-1, -1)`. |
| `add(cell, delta)` | Soma um deslocamento a uma célula: `add((1, 1), (0, 1)) == (1, 2)`. |
| `direction_between(a, b)` | O comando que leva de `a` à vizinha `b`: `direction_between((1, 1), (2, 1)) == 'down'`. |

#### `class Explorer`

Guarda o estado da exploração. Atributos:

| Atributo | O que guarda |
| --- | --- |
| `sense` | Função `sense() -> leitura`, um dicionário como `{'up': 'f', 'up_left': 'b', ...}`. |
| `move` | Função `move(direção) -> bool` (`True` se o robô andou). |
| `pos` | Posição atual, pela odometria. A partida é `(0, 0)`. |
| `known` | O mapa: dicionário `célula → 'f'/'b'/'t'`. |
| `visited` | Conjunto de células onde o robô já esteve. |
| `target` | Posição do alvo (relativa), ou `None` enquanto não foi visto. |
| `moves` | Total de movimentos feitos. |

#### `visit(self)`

Marca a posição atual como livre e visitada, chama `sense()` e anota no mapa
cada uma das 8 células lidas. Se alguma for `t`, guarda a posição do alvo.

#### `step(self, direction)`

Chama `move(direction)`. Se o simulador recusar, lança `RuntimeError`, o que
nunca deve acontecer, porque o robô só anda para células que viu livres.
Senão, atualiza `pos` (odometria) e soma 1 em `moves`.

#### `unvisited_neighbor(self)`

Devolve a direção da primeira vizinha direta (na ordem de `DIRECTIONS`) que
está no mapa como `f` e não foi visitada, ou `None`. O alvo está no mapa
como `t`, não `f`, então nunca é escolhido.

#### `explore(self)`

A DFS. Lê os sensores na partida e começa a pilha com ela. Enquanto a pilha
não estiver vazia: se há vizinha não visitada, anda para ela, empilha e
chama `visit`; senão, desempilha e, se ainda sobrar algo na pilha, volta um
passo para o novo topo. Termina na partida.

#### `to_grid(self)`

Converte o mapa (dicionário) em matriz no formato do `/get_map`. Calcula o
retângulo que contém todas as células conhecidas (menor e maior linha e
coluna), preenche com `?` e copia os valores conhecidos. Marca a partida com
`r`. Devolve `(grid, (top, left))`: a célula relativa `(l, c)` fica em
`grid[l - top][c - left]`.

#### `best_route(self)`

Gera a matriz com `to_grid`, roda a `bfs()` de `grid.py` da partida até o
alvo e converte o caminho de volta para coordenadas relativas. Devolve a
lista de células, ou `None` se não houver caminho.

#### `follow(self, path)`

Percorre um caminho chamando `step` com `direction_between` para cada célula
depois da primeira (que é a posição atual).

#### `compare_with_real(known, real_grid, origin)`

Função solta, usada na conferência e nos testes. Para cada célula do mapa
construído, soma `origin` para obter a posição no mapa real e compara só
"parede ou não". Não compara a letra exata porque, no mapa real, o `r` pode
estar em cima de uma célula livre ou do alvo. Devolve a lista das células
diferentes, que deve ser vazia.

### `nav_auto/part2_node.py`: o nó da parte 2

[Código](../src/nav_auto/nav_auto/part2_node.py). Faz a ponte entre o
`Explorer` e o simulador.

#### Constantes

| Nome | Valor | Para que serve |
| --- | --- | --- |
| `RESET_WAIT` | `3.0` s | Quanto esperar o jogo aplicar o *reset* ([O reset demora](#o-reset-demora)). |
| `FRESH_MESSAGES` | `2` | Mensagens novas a esperar depois de esvaziar a fila ([Leituras velhas dos sensores](#leituras-velhas-dos-sensores)). |
| `SENSOR_TIMEOUT` | `3.0` s | Quanto esperar pelos sensores antes de desistir com erro. |
| `ROUTE_STEP_DELAY` | `0.1` s | Pausa entre movimentos da rota final, para dar para ver a rota na janela. A exploração não tem pausa. |

#### `class Part2Mapper(Node)` e `__init__(self)`

Cria o nó `part2_mapper`, clientes para `/move_command`, `/reset` e
`/get_map`, e assina o tópico dos sensores com o perfil `sensor_data` (o
mesmo do publicador; com outro perfil, as mensagens não chegam).

#### `on_sensors(self, msg)`

*Callback* do tópico: o ROS a chama a cada mensagem que chega (durante um
*spin*). Guarda a leitura como dicionário (`{'up': 'f', ...}`) em
`last_reading` e soma 1 em `sensor_count`. O contador é o que permite saber
se chegou uma mensagem nova.

#### `call(self, client, request)` e `wait_for_services(self)`

Iguais às da parte 1: `call` chama um serviço e roda o laço do ROS até a
resposta chegar; `wait_for_services` espera os três serviços existirem.

#### `sense(self)`

A função `sense` que o `Explorer` recebe. Primeiro esvazia a fila: chama
`rclpy.spin_once` com `timeout_sec=0` (processa uma mensagem, se houver, sem
esperar) até o contador parar de subir. Depois processa mensagens até chegarem
`FRESH_MESSAGES` novas e devolve a última leitura. Se elas não chegarem em
`SENSOR_TIMEOUT` segundos, lança `RuntimeError`. Usa `time.monotonic`, que
não é afetado por ajustes no relógio do sistema.

#### `move(self, direction)`

A função `move` que o `Explorer` recebe. Chama `/move_command` e devolve
`success`. No primeiro movimento bem-sucedido, calcula `origin` (a posição
real da partida) a partir de `robot_pos` menos o deslocamento da direção.
Ignora `target_pos`. Dorme `step_delay`, que é 0 na exploração e
`ROUTE_STEP_DELAY` na rota final.

#### `run(self)`

O roteiro:

0. Espera os serviços, chama `/reset` com o pedido vazio (reinicia o mesmo
   labirinto) e processa mensagens por `RESET_WAIT` segundos.
1. Cria o `Explorer` com `self.sense` e `self.move` e chama `explore()`,
   medindo o tempo. Se o alvo não foi visto, avisa e para.
2. Calcula a rota com `best_route()`, imprime o mapa construído com a rota e
   a executa com `follow`.
3. Conferência: chama `/get_map`, reconstrói o mapa real, conta as
   diferenças com `compare_with_real`, roda a BFS da parte 1 no mapa real e
   registra os dois resultados numa linha.

#### `main(args=None)`

Como na parte 1: inicializa o ROS, cria o nó e chama `run()`. Um
`RuntimeError` (movimento recusado, sensores parados) vira uma mensagem de
erro no log, em vez de um *traceback*. O `finally` sempre destrói o nó e
desliga o ROS.

### `test/test_explorer.py`: testes da exploração

[Código](../src/nav_auto/test/test_explorer.py). Testa o `Explorer` sem o
simulador, com um robô de mentira que lê os **mesmos CSVs** do simulador.

| Nome | O que faz |
| --- | --- |
| `MAPS_DIR`, `MAP_FILES` | A pasta `src/culling_games/src/cg/maps` e a lista dos seus CSVs. |
| `FakeRobot` | Imita o simulador. Guarda o grid real e a posição do robô. `sense()` devolve as 8 células ao redor, e `move()` recusa paredes e **falha o teste** se o robô pisar no alvo durante a exploração (`allow_target` só é ligado para a rota final). |
| `load_map(path)` | Lê um CSV como matriz. |
| `reachable_without_target(grid, start)` | *Flood fill* (uma BFS sem alvo): todas as células alcançáveis da partida sem passar pelo alvo. É a resposta certa do que a exploração deveria visitar. |
| `test_explores_every_simulator_map` | Roda uma vez para cada um dos 11 labirintos. Confere que o alvo foi achado no lugar certo, que o mapa construído não tem nenhuma diferença do real, que as células visitadas são exatamente as alcançáveis, que o robô termina na partida, que a rota no mapa construído é **idêntica** à BFS no mapa real (a rota da parte 1) e que segui-la chega ao alvo. É pulado se o submódulo não estiver presente. |

## Resultados

### Nos 11 labirintos do simulador (testes, robô de mentira)

Números de `test_explores_every_simulator_map`, que usa exatamente a mesma
lógica do nó. A ordem das decisões é determinística, então o simulador de
verdade faz os mesmos movimentos.

| Labirinto | Células visitadas | Movimentos de exploração | Rota (parte 2) | Rota (parte 1) |
| --- | --- | --- | --- | --- |
| `1.csv` | 391 | 780 | 102 | 102 |
| `2.csv` | 391 | 780 | 102 | 102 |
| `3.csv` | 391 | 780 | 192 | 192 |
| `4.csv` | 391 | 780 | 114 | 114 |
| `5.csv` | 391 | 780 | 204 | 204 |
| `6.csv` | 391 | 780 | 110 | 110 |
| `7.csv` | 391 | 780 | 96 | 96 |
| `8.csv` | 391 | 780 | 164 | 164 |
| `9.csv` | 391 | 780 | 190 | 190 |
| `10.csv` | 391 | 780 | 48 | 48 |
| `test.csv` | 728 | 1454 | 25 | 25 |

O alvo não conta como célula visitada. Em todos os labirintos, o robô
visitou todas as células alcançáveis, o mapa construído não teve nenhuma
diferença do real e a rota da parte 2 foi idêntica à da parte 1. Os
movimentos de exploração são sempre 2 × (células − 1), já incluindo a volta
à partida.

### No simulador de verdade

`ros2 run nav_auto part2`, com o jogo aberto com janela, em 27/11/2026:

| Labirinto | Visitadas | Movimentos de exploração | Tempo de exploração | Rota (parte 2) | Rota (parte 1) | Diferenças no mapa |
| --- | --- | --- | --- | --- | --- | --- |
| `1.csv` | 391 | 780 | 9,2 s | 102 | 102 (idêntica) | 0 |
| `3.csv` ¹ | 391 | 780 | 9,1 s | 192 | 192 (idêntica) | 0 |
| `5.csv` | 391 | 780 | 9,2 s | 204 | 204 (idêntica) | 0 |
| `7.csv` | 391 | 780 | 9,3 s | 96 | 96 (idêntica) | 0 |
| `test.csv` | 728 | 1454 | 16,9 s | 25 | 25 (idêntica) | 0 |
| `--generate` | 391 | 780 | 9,2 s | 74 | 74 (idêntica) | 0 |

¹ No `3.csv`, a parte 1 rodou antes no mesmo jogo e venceu. A parte 2
reiniciou o labirinto sozinha e funcionou.

Os movimentos de exploração são exatamente os previstos pelos testes. O
tempo é quase todo espera pelos sensores: duas mensagens a 100 por segundo,
em cada célula nova.

Saída do `1.csv` (o mapa impresso foi omitido aqui; é igual ao da
[parte 1](part1.md#exemplo-de-saída-1csv)):

```text
[INFO] [part2_mapper]: Exploração: 391 células visitadas, 780 movimentos, 9.2 s
[INFO] [part2_mapper]: Rota no mapa construído (BFS): 102 movimentos
[INFO] [part2_mapper]: Alvo alcançado!
[INFO] [part2_mapper]: Conferência: 0 células diferentes do mapa real; rota da parte 1: 102 movimentos; rota da parte 2: 102 movimentos; idênticas
```

## Testes unitários

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test
```

Resultado: `16 passed` (5 da parte 1 e 11 labirintos da parte 2).

## Limitações

- **O tópico dos sensores depende do relógio do sistema.** Se o relógio do
  WSL pular para trás, o simulador para de publicar os sensores e o nó
  desiste com "Os sensores pararam de publicar". A solução está no
  [Passo 7 do setup](setup.md#passo-7-evitar-saltos-de-relógio-no-wsl).
- **O reset é esperado por tempo, não por um sinal.** O simulador não avisa
  quando terminou de aplicar um *reset*, então o nó espera 3 s. Numa máquina
  muito mais lenta pode ser preciso aumentar `RESET_WAIT`.
- **Sem janela funciona, mas é bem mais lento.** Com o jogo rodando sem
  janela (`SDL_VIDEODRIVER=dummy`), o simulador publica os sensores com muito
  menos frequência, e a exploração do `1.csv` leva cerca de 2 minutos em vez
  de 9 s.
- **Exploração completa.** O robô explora todo o labirinto alcançável mesmo
  depois de ver o alvo. Isso é o que garante a rota ótima em qualquer
  labirinto, mas custa mais movimentos do que parar cedo.
