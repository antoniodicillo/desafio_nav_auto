# Parte 2: Mapeamento do labirinto

**Objetivo:** sem acesso ao mapa, o robô deve navegar pelo labirinto usando
os sensores, construir o mapa no processo e, depois, provar que o mapa
construído é suficiente para reproduzir a rota ótima da parte 1.

**Em uma linha:** explora com DFS lendo os sensores → volta à partida pelo
próprio mapa → BFS no mapa construído → executa a rota → só no fim pede o
mapa real para conferir que a rota é a mesma da parte 1.

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
vezes seguidas, mesmo depois de uma vitória.

### Parâmetros

| Parâmetro | Padrão | O que faz |
| --- | --- | --- |
| `step_delay` | `0.0` | Pausa (s) entre movimentos durante a exploração e a volta. |
| `route_step_delay` | `0.1` | Pausa (s) entre movimentos na rota final, para dar para ver a rota na janela. |
| `reset_first` | `true` | Chama `/reset` (mesmo labirinto) antes de começar. |
| `reset_wait` | `3.0` | Quanto tempo (s) esperar o jogo aplicar o *reset* antes de começar a explorar (veja [O reset demora](#o-reset-demora)). |
| `verify` | `true` | No fim, pede o mapa real e compara (mapa e rota). |
| `sensor_timeout` | `3.0` | Quanto tempo (s) esperar por uma leitura válida dos sensores antes de desistir. |

Exemplo: `ros2 run nav_auto part2 --ros-args -p route_step_delay:=0.0 -p verify:=false`

## Passo a passo do que acontece

```text
Part2Mapper                                     Simulador (cg maze)
     │                                                 │
 0.  │  espera serviços e sensores                     │
     │ ── /reset (mesmo labirinto) ───────────────────►│
     │  espera reset_wait (o jogo recria o labirinto)  │
     │                                                 │
 1.  │  EXPLORAÇÃO (DFS)                               │
     │   repete até não sobrar célula para visitar:    │
     │   ◄── /culling_games/robot_sensors ─────────────│   8 células ao redor
     │   anota no mapa; escolhe uma vizinha nova       │
     │   ── /move_command {direction} ────────────────►│
     │   (num beco sem saída, volta um passo)          │
     │                                                 │
 2.  │  VOLTA À PARTIDA (BFS no mapa construído)       │
     │   ── /move_command ... ────────────────────────►│
     │                                                 │
 3.  │  PLANEJAMENTO: BFS partida → alvo no mapa construído
     │                                                 │
 4.  │  EXECUÇÃO da rota                               │
     │   ── /move_command ... ────────────────────────►│   o robô chega ao alvo
     │                                                 │
 5.  │  CONFERÊNCIA                                    │
     │   ── /get_map (a única vez) ───────────────────►│
     │   compara mapas e rotas                         │
```

O `/get_map` só é chamado na etapa 5, depois que a rota já foi calculada e
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
                10    up         (1,5)      célula nova; fronteira vazia: fim
```

Depois disso, a volta à partida leva 8 movimentos e a rota final tem 6
(`right, right, down, down, left, left`), a mesma que a BFS acha no mapa
real.

Na prática, o robô percorre cada corredor indo e voltando, como quem
explora uma caverna com um fio amarrado na entrada.

### A fronteira: quando parar

A **fronteira** é o conjunto de células livres que o robô já viu, ainda não
visitou e que são **vizinhas diretas** de alguma célula visitada. A
exploração termina quando a fronteira fica vazia.

**Por que isso garante que o robô viu tudo o que é alcançável:** pegue
qualquer célula livre alcançável e um caminho da partida até ela. Se ela
ainda não foi visitada, siga o caminho até a primeira célula não visitada.
A célula anterior foi visitada, então os sensores viram esta como livre, e
ela está na fronteira. Logo, fronteira vazia significa que todas as células
alcançáveis foram visitadas.

**Por que não parar antes:** o robô poderia parar assim que visse o alvo,
mas num labirinto com ciclos o caminho que ele fez até lá pode não ser o
mais curto, e o caminho mais curto pode passar por células que ele ainda não
viu. Explorar tudo garante que o mapa contém a rota ótima.

**Por que a pilha nunca esvazia antes da hora:** uma célula da fronteira é
vizinha de uma célula visitada `v`. A DFS só desempilha `v` quando `v` não
tem mais vizinhas não visitadas, então ela visita essa célula antes de
desempilhar `v`. Enquanto houver fronteira, `v` continua na pilha. O código
tem uma checagem para esse caso mesmo assim, que nunca deve disparar.

**Por que as diagonais não entram na fronteira:** uma célula vista só na
diagonal pode estar cercada de paredes, sem nenhum caminho até ela (veja o
teste `test_ignores_pocket_seen_only_diagonally`). Se ela entrasse na
fronteira, a exploração nunca terminaria. Ela fica anotada no mapa, mas o
robô não tenta alcançá-la. Se ela for alcançável, um dia ela será vizinha
direta de uma célula visitada e entrará na fronteira por esse caminho.

### Não pisar no alvo

Quando o robô entra no alvo, o jogo acaba: o `t` some e todos os movimentos
seguintes são recusados. Por isso, durante a exploração, o alvo **não conta
como célula livre**. Ele é anotado no mapa assim que um sensor o vê (da
célula ao lado), mas o robô nunca entra nele. A volta à partida também trata
o alvo como parede. Só a rota final entra no alvo.

### Leituras velhas dos sensores

O simulador publica os sensores a cada 10 ms, sem dizer de qual posição é a
leitura. Logo depois de um movimento, a próxima mensagem que chega pode ter
sido publicada **antes** do movimento: ela descreve a posição antiga. Usá-la
estragaria o mapa.

O problema é pior do que parece. O ROS guarda as mensagens que chegam numa
fila (até 5, no perfil `sensor_data`), e o nó só as processa quando roda o
*spin*. Quando a resposta do movimento chega, a fila pode ainda ter várias
leituras de antes do movimento. Contar "duas mensagens processadas depois da
resposta" não basta: elas podem ser exatamente essas leituras velhas que
estavam na fila. Esse erro apareceu nos testes reais: o mapa ganhou uma
célula livre que era parede, e o simulador recusou o movimento.

Três proteções, juntas:

1. **Esvaziar a fila** (`Part2Mapper.drain`): logo depois do movimento, o nó
   processa tudo o que já estava na fila, sem usar nada.
2. **Esperar mensagens novas** (`FRESH_MESSAGES = 2`): depois de esvaziar a
   fila, o nó espera chegarem duas mensagens. A primeira ainda pode ter sido
   publicada pouco antes do movimento e estar a caminho; a segunda já é de
   depois do movimento.
3. **Conferir com o mapa** (`Explorer.agrees_with_map`): uma célula já
   conhecida nunca muda (paredes e alvo são fixos). Se a leitura disser algo
   diferente sobre uma célula conhecida, ela é velha e o nó espera a próxima.

A conferência com o mapa sozinha não basta. Num corredor reto, a leitura da
posição anterior é coerente com tudo o que já se sabe e afirmaria que a
célula seguinte é livre sem que ninguém a tenha visto. Por isso as três são
usadas juntas.

O robô só lê os sensores quando chega em uma célula **nova**. Nos passos de
volta (desempilhando), ele já conhece tudo em volta, então não espera os
sensores e anda mais rápido.

### O reset demora

Para começar do zero, o nó chama `/reset` antes de explorar. O simulador
responde na hora, mas só aplica o *reset* no próximo quadro do Pygame, e
aplicar é lento: ele recria a janela e recarrega as 18 imagens de vitória.
Medido nesta máquina: **0,6 a 0,9 s**. Movimentos feitos nesse meio-tempo vão
para o robô antigo e se perdem quando o novo robô aparece na partida.

Nos testes reais, com uma espera de 0,5 s, o robô explorou umas 12 células e
foi "teletransportado" de volta para a partida no meio da exploração. O
simulador não avisa quando terminou o *reset*, então o nó espera
`reset_wait` segundos (padrão 3,0, cerca de 3× o pior caso medido). Se mesmo
assim o *reset* chegar atrasado, a conferência da odometria (abaixo) percebe
e para com um erro que sugere aumentar `reset_wait`.

### Conferindo a odometria

A resposta do `/move_command` traz a posição do robô no simulador
(`robot_pos`). No primeiro movimento, o nó descobre onde fica a partida
(`origin` = posição informada − odometria). A cada movimento seguinte, ele
confere se `origin + odometria` é igual à posição informada. Se não for, o
nó para com erro. Essa posição real só é usada nessa checagem e na
conferência final, nunca para decidir para onde ir.

O campo `target_pos` da mesma resposta entrega a posição do alvo, mesmo que o
robô nunca o tenha visto. Ele é **ignorado de propósito**: a parte 2 precisa
achar o alvo pelos sensores.

### Volta à partida

Quando a fronteira esvazia, o robô está no fim do último corredor explorado.
Para reproduzir a rota da parte 1, ele precisa estar na partida. Em vez de
chamar `/reset`, que reiniciaria o jogo, ele **volta pelo próprio mapa**:
roda a BFS da posição atual até a partida, com o alvo tratado como parede, e
segue o caminho. Isso já é uma primeira prova de que o mapa funciona.

## Por que o mapa construído basta

Depois da exploração, o robô roda a mesma `bfs()` da parte 1 no mapa
construído, convertido em matriz no mesmo formato do `/get_map`:
`?` nas células nunca vistas e `r` na partida. Células `?` não são
transitáveis, então a rota só passa por onde o robô viu.

**Afirmação:** essa rota é a mesma que a parte 1 calcula com o mapa real.

**Por quê:**

1. A exploração visitou **todas** as células alcançáveis sem passar pelo alvo
   (seção [A fronteira](#a-fronteira-quando-parar)). Cada uma foi vista e
   anotada corretamente, assim como suas 4 vizinhas. Logo, o mapa construído
   tem exatamente as mesmas células e ligações que o mapa real nessa região.
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

Com `verify` ligado (padrão), depois de chegar ao alvo, o nó chama `/get_map`
pela primeira e única vez e confere:

| Checagem | Como |
| --- | --- |
| O mapa está certo | Cada célula conhecida, transladada por `origin`, deve ser parede no mapa real se e somente se for parede no nosso (`compare_with_real`). Deve dar 0 diferenças. |
| Quanto do labirinto foi visto | Células transitáveis conhecidas / células transitáveis do mapa real (só informativo). |
| O alvo estava certo | Depois da vitória, o `t` some do mapa real e a célula do alvo vira `r`. Isso deve acontecer exatamente onde nosso mapa diz que está o alvo. |
| A rota é a da parte 1 | BFS no mapa real, da partida ao alvo (o que a parte 1 faz), comparada com a nossa rota. Devem ter o mesmo tamanho (e, pelo argumento acima, ser idênticas). |

Se tudo bater, o nó escreve
`Conferência OK: o mapa construído reproduz a rota ótima da parte 1.`

## Por que DFS?

| Estratégia de exploração | Problema |
| --- | --- |
| **DFS (escolhida)** | O próximo passo é sempre uma célula vizinha (avançar ou voltar), o que combina com um robô que só anda uma célula por vez. Visita tudo, com no máximo 2 movimentos por ligação entre células. |
| BFS para explorar | A BFS processa as células em ordem de distância, então a próxima célula da fila costuma estar longe do robô. Ele teria que atravessar o labirinto a cada passo, gastando muito mais movimentos. A BFS é ótima para **calcular** rotas num mapa pronto, e por isso é usada nas etapas 2 e 3. |
| Seguir a parede (mão direita) | Simples, mas pode ficar dando voltas para sempre num labirinto com ciclos, e não garante visitar todas as células. |
| Parar ao ver o alvo | Mais rápido, mas o mapa pode não conter a rota ótima num labirinto com ciclos. |

**Custo:** a DFS percorre cada ligação da árvore de exploração no máximo
duas vezes (ida e volta), então são no máximo 2 × (V − 1) movimentos, com V
células alcançáveis. Os sensores são lidos uma vez por célula (V leituras).
Como a exploração para quando a fronteira esvazia, sem voltar pela pilha até
a partida, ela gasta menos que isso. Em seguida, a volta à partida usa o
caminho mais curto.

## O código, função por função

### `nav_auto/explorer.py`: a exploração

[Código](../src/nav_auto/nav_auto/explorer.py). Python puro, sem ROS: recebe
`sense` e `move` como funções, então roda igual no nó e nos testes.

#### Constantes e funções auxiliares

| Nome | O que é |
| --- | --- |
| `SENSOR_OFFSETS` | Cada campo da mensagem `RobotSensors` e o deslocamento (linha, coluna) da célula que ele lê. Ex.: `'up_left': (-1, -1)`. |
| `SENSOR_VALUES` | `{'f', 'b', 't'}`: os únicos valores válidos de um sensor. Uma leitura com outro valor é rejeitada. |
| `MapConflict` | Exceção lançada quando uma leitura contradiz o mapa. |
| `add(cell, delta)` | Soma um deslocamento a uma célula: `add((1, 1), (0, 1)) == (1, 2)`. |
| `direction_between(a, b)` | O comando que leva de `a` à vizinha `b`: `direction_between((1, 1), (2, 1)) == 'down'`. Lança `ValueError` se não forem vizinhas. |

#### `class Explorer`

Guarda o estado da exploração e faz as contas. Atributos:

| Atributo | O que guarda |
| --- | --- |
| `sense` | Função `sense(accept) -> leitura`. `accept` é uma função que diz se a leitura é coerente com o mapa; o nó a usa para descartar leituras velhas. |
| `move` | Função `move(direção) -> bool` (`True` se o robô andou). |
| `start`, `pos` | Partida (sempre `(0, 0)`) e posição atual, pela odometria. |
| `known` | O mapa: dicionário `célula → 'f'/'b'/'t'`. |
| `visited` | Conjunto de células onde o robô já esteve. |
| `frontier` | Livres, não visitadas, vizinhas diretas de visitadas. |
| `target` | Posição do alvo (relativa), ou `None` enquanto não foi visto. |
| `moves` | Total de movimentos feitos. |

#### `agrees_with_map(self, reading)`

Para cada um dos 8 sensores: se o valor não é `f`, `b` ou `t`, devolve
`False`; se a célula correspondente (posição atual + deslocamento) já está no
mapa com outro valor, devolve `False`. Senão, `True`. É a "conferência com o
mapa" contra leituras velhas.

#### `visit(self)`

Lê os sensores na posição atual e atualiza tudo:

1. Chama `sense(self.agrees_with_map)` e confere a leitura de novo. Se não
   bater, lança `MapConflict`. Num robô de verdade isso indicaria sensor com
   defeito; aqui, um erro no código.
2. Marca a posição atual como livre e visitada, e a tira da fronteira.
3. Para cada sensor, anota o valor no mapa. Se for `t`, guarda a posição do
   alvo. Se for `f`, de um sensor direto (não diagonal), e a célula não foi
   visitada, põe na fronteira.

#### `step(self, direction)`

Chama `move(direction)`. Se o simulador recusar, lança `RuntimeError`, o que
nunca deve acontecer, porque o robô só anda para células que viu livres.
Senão, atualiza `pos` (odometria) e soma 1 em `moves`.

#### `follow(self, path)`

Percorre um caminho: confere que a primeira célula é a posição atual e chama
`step` com `direction_between` para cada célula seguinte.

#### `unvisited_neighbor(self)`

Devolve a direção da primeira vizinha direta (na ordem de `DIRECTIONS`) que
está no mapa como `f` e não foi visitada, ou `None`. O alvo está no mapa
como `t`, não `f`, então nunca é escolhido.

#### `explore(self)`

A DFS. Lê os sensores na partida e começa a pilha com ela. Enquanto a
fronteira não estiver vazia: se há vizinha não visitada, anda para ela,
empilha e chama `visit`; senão, desempilha e volta um passo para o novo topo
da pilha. Devolve o número de movimentos.

#### `to_grid(self)`

Converte o mapa (dicionário) em matriz no formato do `/get_map`. Calcula o
retângulo que contém todas as células conhecidas (menor e maior linha e
coluna), preenche com `?` e copia os valores conhecidos. Marca a partida com
`r`. Devolve `(grid, (topo, esquerda))`: a célula relativa `(l, c)` fica em
`grid[l - topo][c - esquerda]`.

#### `plan(self, origin, goal, avoid_target=False)`

Caminho mais curto de `origin` a `goal` usando só o mapa construído. Gera a
matriz com `to_grid`, opcionalmente transforma o alvo em parede, converte as
duas células para índices da matriz, roda a `bfs()` de `grid.py` e converte o
caminho de volta para coordenadas relativas. Devolve a lista de células ou
`None`.

#### `go_home(self)`

`plan` da posição atual até a partida, evitando o alvo, e `follow` no
caminho. Devolve o número de movimentos.

#### `best_route(self)`

`plan` da partida até o alvo, sem evitar o alvo, já que é para chegar nele.
Devolve `None` se o alvo não foi visto.

#### `compare_with_real(known, real_grid, origin)`

Função solta, usada na conferência e nos testes. Para cada célula do mapa
construído, soma `origin` para obter a posição no mapa real e compara só
"parede ou não". Não compara a letra exata porque, no mapa real, o `r` pode
estar em cima de uma célula livre ou do alvo. Células fora do mapa real
também contam como diferença. Devolve a lista
`[(célula, nosso valor, valor real), ...]`, que deve ser vazia.

### `nav_auto/part2_node.py`: o nó da parte 2

[Código](../src/nav_auto/nav_auto/part2_node.py). Faz a ponte entre o
`Explorer` e o simulador.

#### `FRESH_MESSAGES = 2`

Quantas mensagens novas dos sensores esperar depois de esvaziar a fila (veja
[Leituras velhas dos sensores](#leituras-velhas-dos-sensores)).

#### `class Part2Mapper(Node)` e `__init__(self)`

Cria o nó `part2_mapper`, declara os parâmetros da tabela
[Parâmetros](#parâmetros), cria clientes para `/move_command`, `/reset` e
`/get_map` e assina o tópico dos sensores com o perfil `sensor_data` (o mesmo
do publicador; com outro perfil, as mensagens não chegam). Também zera a
odometria (`odom`) e deixa `origin` vazio até o primeiro movimento.

#### `on_sensors(self, msg)`

*Callback* do tópico: o ROS a chama a cada mensagem que chega (durante um
*spin*). Guarda a leitura como dicionário (`{'up': 'f', ...}`) em
`last_reading` e soma 1 em `sensor_count`. O contador é o que permite saber
se uma mensagem chegou depois de um certo momento.

#### `call(self, client, request)`

Igual à da parte 1: chama o serviço e roda o laço do ROS até a resposta
chegar. Mensagens dos sensores que chegarem enquanto isso também são
processadas.

#### `spin_for(self, seconds)`

Processa mensagens durante um tempo, chamando `rclpy.spin_once` em laço.
Diferente de `time.sleep`, que deixaria as mensagens paradas na fila.

#### `wait_for_services(self)` e `wait_for_sensors(self)`

Esperam os três serviços aparecerem e a primeira mensagem dos sensores
chegar, imprimindo um aviso a cada segundo.

#### `drain(self)`

Esvazia a fila de mensagens dos sensores: chama `rclpy.spin_once` com
`timeout_sec=0` (processa uma mensagem, se houver, sem esperar) até o
contador parar de subir. As leituras processadas aqui são de antes do último
movimento e não são usadas.

#### `sense(self, accept)`

A função `sense` que o `Explorer` recebe. Primeiro chama `drain()`. Depois
anota o contador e processa mensagens até que (a) tenham chegado pelo menos
`FRESH_MESSAGES` mensagens novas e (b) a última leitura seja aceita por
`accept`. Se isso não acontecer em `sensor_timeout` segundos, lança
`RuntimeError`. Usa `time.monotonic`, que não é afetado por ajustes no
relógio do sistema.

#### `move(self, direction)`

A função `move` que o `Explorer` recebe. Chama `/move_command`. Se o
simulador recusar, devolve `False`. Senão, atualiza a odometria própria, usa
`robot_pos` para descobrir `origin` (no primeiro movimento) ou conferir a
odometria (nos seguintes), dorme `step_delay` e devolve `True`. Se a
odometria não bater, lança `RuntimeError`; a mensagem lembra que a causa mais
provável é um *reset* atrasado. Ignora `target_pos`.

#### `run(self)`

O roteiro:

0. Espera serviços e sensores. Se `reset_first`, chama `/reset` com o pedido
   vazio (reinicia o mesmo labirinto) e processa mensagens por `reset_wait`
   segundos, o tempo de o jogo aplicar o *reset*
   ([O reset demora](#o-reset-demora)).
1. Cria o `Explorer` com `self.sense` e `self.move` e chama `explore()`,
   medindo o tempo. Imprime o mapa construído. Se o alvo não foi visto,
   avisa e devolve `False`.
2. `go_home()`.
3. `best_route()` e imprime o mapa com a rota.
4. Troca a pausa para `route_step_delay` e executa a rota com `follow`.
5. Se `verify`, chama `verify` e devolve o resultado dela.

#### `verify(self, explorer, route)`

A [conferência final](#a-conferência-final). Chama `/get_map`, reconstrói o
mapa real e converte a partida e o alvo para coordenadas reais (somando
`origin`). Registra o número de diferenças entre os mapas
(`compare_with_real`) e lista até 10 delas, a cobertura, se o jogo foi
vencido na célula certa e o tamanho das duas rotas (a da parte 1, por BFS no
mapa real, e a nossa). Devolve `True` só se não houver diferenças, o jogo
tiver sido vencido no alvo previsto e as rotas tiverem o mesmo tamanho.

#### `main(args=None)`

Como na parte 1: inicializa o ROS, cria o nó e chama `run()`. Erros
esperados (`RuntimeError`, `MapConflict`) viram uma mensagem de erro no log
em vez de um *traceback*. Se `reset_first` estiver desligado, também lembra
como reiniciar o jogo. O `finally` sempre destrói o nó e desliga o ROS.

### `test/test_explorer.py`: testes da exploração

[Código](../src/nav_auto/test/test_explorer.py). Testam o `Explorer` sem o
simulador, com um robô de mentira que lê os **mesmos CSVs** do simulador.

#### Auxiliares

| Nome | O que faz |
| --- | --- |
| `MAPS_DIR`, `MAP_FILES` | A pasta `src/culling_games/src/cg/maps` e a lista dos seus CSVs. |
| `FakeRobot` | Imita o simulador. Guarda o grid real e a posição do robô. `sense()` devolve as 8 células ao redor, e `move()` recusa paredes e **falha o teste** se o robô pisar no alvo durante a exploração (`allow_target` só é ligado para a rota final). |
| `load_map(path)` | Lê um CSV como matriz. |
| `reachable_without_target(grid, start)` | *Flood fill* (uma BFS sem alvo): todas as células alcançáveis da partida sem passar pelo alvo. É a resposta certa do que a exploração deveria visitar. |
| `to_abs(cells, origin)` | Converte coordenadas relativas em reais. |

#### Testes

| Teste | O que confere |
| --- | --- |
| `test_explores_every_simulator_map` | Roda uma vez para cada um dos 11 labirintos. Confere que o alvo foi achado no lugar certo, que o mapa construído não tem nenhuma diferença do real, que as células visitadas são exatamente as alcançáveis, que `go_home` volta à partida, que a rota no mapa construído é **idêntica** à BFS no mapa real (a rota da parte 1) e que segui-la chega ao alvo. É pulado se o submódulo não estiver presente. |
| `test_ignores_pocket_seen_only_diagonally` | Uma célula livre vista só na diagonal e cercada de paredes: é anotada no mapa, mas não visitada, e a exploração termina. |
| `test_to_grid_has_get_map_format` | A matriz gerada tem `r` na partida, `t` no alvo e a linha explorada sem `?`. |
| `test_conflicting_reading_raises` | Um "sensor quebrado" que dá sempre a mesma leitura faz o `Explorer` lançar `MapConflict` no segundo passo. |

## Resultados

### Nos 11 labirintos do simulador (testes, robô de mentira)

Números de `test_explores_every_simulator_map`, que usa exatamente a mesma
lógica do nó (a ordem das decisões é determinística, então o simulador de
verdade faz os mesmos movimentos):

| Labirinto | Células alcançáveis | Visitadas | Movimentos de exploração | Volta à partida | Rota (parte 2) | Rota (parte 1) |
| --- | --- | --- | --- | --- | --- | --- |
| `1.csv` | 391 | 391 | 678 | 102 | 102 | 102 |
| `2.csv` | 391 | 391 | 750 | 30 | 102 | 102 |
| `3.csv` | 391 | 391 | 718 | 62 | 192 | 192 |
| `4.csv` | 391 | 391 | 732 | 48 | 114 | 114 |
| `5.csv` | 391 | 391 | 686 | 94 | 204 | 204 |
| `6.csv` | 391 | 391 | 640 | 140 | 110 | 110 |
| `7.csv` | 391 | 391 | 740 | 40 | 96 | 96 |
| `8.csv` | 391 | 391 | 694 | 86 | 164 | 164 |
| `9.csv` | 391 | 391 | 750 | 30 | 190 | 190 |
| `10.csv` | 391 | 391 | 682 | 98 | 48 | 48 |
| `test.csv` | 728 | 728 | 752 | 26 | 25 | 25 |

"Células alcançáveis" não conta o alvo. Em todos os labirintos, o robô
visitou todas as células alcançáveis, o mapa construído não teve nenhuma
diferença do real e a rota da parte 2 foi idêntica à da parte 1.

Os labirintos `1` a `10` são perfeitos, com 391 células alcançáveis. Uma DFS
completa, que voltasse pela pilha até a partida, faria 2 × 390 = 780
movimentos. Como a exploração para quando a fronteira esvazia e volta pelo
caminho mais curto, gasta de 640 a 750 movimentos explorando.

### No simulador de verdade

`ros2 run nav_auto part2 --ros-args -p route_step_delay:=0.0`, com o jogo
aberto com janela, em 26/11/2026:

| Labirinto | Visitadas | Movimentos de exploração | Tempo de exploração | Rota (parte 2) | Rota (parte 1) | Diferenças no mapa | Conferência |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `1.csv` | 391 | 678 | 8,7 s | 102 | 102 (idêntica) | 0 | OK |
| `3.csv` ¹ | 391 | 718 | 8,8 s | 192 | 192 (idêntica) | 0 | OK |
| `5.csv` | 391 | 686 | 8,7 s | 204 | 204 (idêntica) | 0 | OK |
| `7.csv` | 391 | 740 | 9,0 s | 96 | 96 (idêntica) | 0 | OK |
| `10.csv` | 391 | 682 | 8,6 s | 48 | 48 (idêntica) | 0 | OK |
| `test.csv` | 728 | 752 | 14,7 s | 25 | 25 (idêntica) | 0 | OK |
| `--generate` | 391 | 680 | 8,7 s | 48 | 48 (idêntica) | 0 | OK |

¹ No `3.csv`, a parte 1 rodou antes no mesmo jogo e venceu. A parte 2
reiniciou o labirinto sozinha e funcionou, duas vezes seguidas.

Os movimentos de exploração são exatamente os previstos pelos testes. O
tempo é quase todo espera pelos sensores: duas mensagens a 100 por segundo,
em cada uma das 391 células novas. No `test.csv`, que tem quase o dobro de
células livres, a exploração leva proporcionalmente mais tempo.

Saída do `1.csv` (os dois mapas impressos foram omitidos aqui; o segundo é
igual ao da [parte 1](part1.md#exemplo-de-saída-1csv)):

```text
[INFO] [part2_mapper]: Labirinto reiniciado: 1.csv
[INFO] [part2_mapper]: Exploração concluída: 391 células visitadas, 678 movimentos, 8.7 s
[INFO] [part2_mapper]: Alvo visto em (13, 13) (relativo à partida)
[INFO] [part2_mapper]: De volta à partida com 102 movimentos
[INFO] [part2_mapper]: Rota ótima no mapa construído (BFS): 102 movimentos
[INFO] [part2_mapper]: Alvo alcançado com 102 movimentos!
[INFO] [part2_mapper]: Mapa: 841 células conhecidas, 0 diferentes do mapa real
[INFO] [part2_mapper]: Células transitáveis do labirinto real conhecidas: 392/392
[INFO] [part2_mapper]: Alvo em (14, 14) no simulador; jogo vencido lá: sim
[INFO] [part2_mapper]: Rota da parte 1 (mapa real): 102 movimentos; rota da parte 2 (mapa construído): 102 movimentos; idênticas
[INFO] [part2_mapper]: Conferência OK: o mapa construído reproduz a rota ótima da parte 1.
```

O alvo aparece em (13, 13) relativo à partida porque a partida está em
(1, 1) no simulador: (1, 1) + (13, 13) = (14, 14). As 392 células
transitáveis são as 391 visitadas mais o alvo.

## Testes unitários

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test
```

Resultado: `19 passed` (5 da parte 1, 11 labirintos e mais 3 casos da parte
2).

## Limitações

- **O tópico dos sensores depende do relógio do sistema.** Se o relógio do
  WSL pular para trás, o simulador para de publicar os sensores e o nó
  desiste depois de `sensor_timeout` segundos com "Nenhuma leitura válida dos
  sensores". A solução está no
  [Passo 7 do setup](setup.md#passo-7-evitar-saltos-de-relógio-no-wsl).
- **O reset é esperado por tempo, não por um sinal.** O simulador não avisa
  quando terminou de aplicar um *reset*, então o nó espera `reset_wait`
  segundos. Numa máquina muito mais lenta pode ser preciso aumentar esse
  valor; a conferência da odometria avisa quando isso acontece.
- **Sem janela funciona, mas é bem mais lento.** Com o jogo rodando sem
  janela (`SDL_VIDEODRIVER=dummy`), o simulador publica os sensores com muito
  menos frequência: a exploração do `1.csv` levou 124 s, contra 8,7 s com
  janela, com o mesmo resultado (conferência OK).
- **Exploração completa.** O robô explora todo o labirinto alcançável mesmo
  depois de ver o alvo. Isso é o que garante a rota ótima em qualquer
  labirinto, mas custa mais movimentos que parar cedo.
