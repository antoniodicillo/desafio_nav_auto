# 02 — Parte 1: Navegação com mapa

**Objetivo:** com acesso ao mapa completo do labirinto, encontrar a rota
otimizada (com o menor número de movimentos) do robô até o alvo e executá-la.

**Código:**

- [`src/nav_auto/nav_auto/grid.py`](../src/nav_auto/nav_auto/grid.py): o
  algoritmo (BFS) e funções do grid. É Python puro, sem ROS, e será reutilizado
  na parte 2.
- [`src/nav_auto/nav_auto/part1_node.py`](../src/nav_auto/nav_auto/part1_node.py):
  o nó ROS 2 que conversa com o simulador.

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

O parâmetro `step_delay` (padrão `0.05` s) controla a pausa entre movimentos,
para dar tempo de ver o robô andando. Para executar sem pausa:

```bash
ros2 run nav_auto part1 --ros-args -p step_delay:=0.0
```

> Se o jogo já tiver sido vencido, o alvo some do mapa e o nó avisa. Reinicie
> o labirinto com
> `ros2 service call /reset cg_interfaces/srv/Reset "{is_random: false}"`.

## Modelando o labirinto como um grafo

O labirinto é uma matriz de células. Para aplicar um algoritmo de grafos:

- **Vértices:** cada célula transitável (livre `f`, robô `r` ou alvo `t`).
- **Arestas:** ligam duas células transitáveis vizinhas na horizontal ou na
  vertical (o robô só anda para `up`, `down`, `left` e `right`; diagonais não
  são movimentos).
- **Pesos:** todo movimento custa 1. O grafo é **não ponderado**.

Paredes (`b`) não são vértices, então nenhum caminho passa por elas.

O grafo não é montado explicitamente em memória: os vizinhos de uma célula são
calculados na hora pela função `neighbors()`, olhando as 4 células ao redor.

## Por que BFS?

A **busca em largura** (BFS) explora o grafo em "camadas": primeiro todas as
células a 1 movimento do início, depois todas a 2 movimentos, e assim por
diante. Ela usa uma **fila** (FIFO) para garantir essa ordem.

Consequência: a primeira vez que a BFS chega ao alvo, ela chegou pelo menor
número possível de movimentos. Em um grafo não ponderado, **a BFS sempre
encontra o caminho mais curto**. Isso é exatamente a "rota otimizada" pedida.

### O algoritmo, passo a passo

```text
fila      ← [início]
veio_de   ← {início: nenhum}          # também serve como "visitados"

enquanto a fila não estiver vazia:
    célula ← remove o primeiro da fila
    se célula é o alvo: reconstrói o caminho e termina
    para cada vizinho transitável da célula:
        se vizinho ainda não foi visitado:
            veio_de[vizinho] ← célula
            coloca vizinho no fim da fila

se a fila esvaziar sem achar o alvo: não existe caminho
```

**Reconstrução do caminho:** o dicionário `veio_de` guarda, para cada célula,
de qual célula a BFS chegou nela. Partindo do alvo e seguindo `veio_de` até o
início, obtemos o caminho de trás para frente; basta invertê-lo.

**Do caminho aos comandos:** para cada par de células consecutivas, a diferença
de posição vira um comando: `(-1, 0)` → `up`, `(1, 0)` → `down`,
`(0, -1)` → `left`, `(0, 1)` → `right` (função `path_to_moves()`).

### Complexidade

- **Tempo:** O(V + E). Cada célula entra na fila no máximo uma vez, e cada
  ligação entre células é examinada no máximo duas vezes. No grid, cada
  célula tem no máximo 4 vizinhos, então E ≤ 4V, e o tempo é O(V).
- **Memória:** O(V), para a fila e o dicionário `veio_de`.

Um labirinto 29 × 29 tem 841 células; a BFS roda em menos de 1 ms.

### Comparação com outros algoritmos

| Algoritmo | Encontra o caminho mais curto aqui? | Observação |
| --- | --- | --- |
| **BFS** | Sim | Ideal para grafos não ponderados. Escolhido. |
| DFS (busca em profundidade) | Não | Encontra *um* caminho, não necessariamente o mais curto. É útil para **explorar** (parte 2). |
| Dijkstra | Sim | Generaliza a BFS para arestas com pesos diferentes. Com todos os pesos iguais a 1, faz o mesmo que a BFS, com o custo extra de uma fila de prioridade. |
| A* | Sim, com heurística admissível (ex.: distância de Manhattan) | Usa uma estimativa da distância até o alvo para examinar menos células. Mesmo resultado, mais complexo de implementar; a diferença é irrelevante em um grid deste tamanho. |

## Como o nó funciona

```text
Part1Navigator                         Simulador (cg maze)
      │                                       │
      │ ── /get_map ─────────────────────────►│
      │ ◄─ grid achatado + dimensões ─────────│
      │                                       │
      │  reconstrói a matriz (grid_from_flat)
      │  acha robô 'r' e alvo 't' (find_cell)
      │  BFS → caminho → comandos
      │                                       │
      │ ── /move_command {direction} ────────►│   (uma vez por passo)
      │ ◄─ success, robot_pos, target_pos ────│
      │  confere se robot_pos é a célula esperada
```

A cada passo, o nó confere se o robô chegou na célula prevista pelo caminho.
Se um movimento falhar, o nó para e avisa, em vez de seguir com um plano
errado.

## Resultados

Testado em 24/11/2026 no ambiente descrito em
[01-setup-ambiente.md](01-setup-ambiente.md):

| Labirinto | Tamanho | Início → Alvo | Movimentos da rota | Tempo da BFS | Resultado |
| --- | --- | --- | --- | --- | --- |
| `test.csv` (sala aberta) | 29 × 29 | (1, 1) → (13, 14) | 25 | < 1 ms | Alvo alcançado |
| `1.csv` | 29 × 29 | (1, 1) → (14, 14) | 102 | 0,20 ms | Alvo alcançado |
| `7.csv` | 29 × 29 | (1, 1) → (14, 14) | 96 | 0,20 ms | Alvo alcançado |
| `--generate` (aleatório) | 29 × 29 | (1, 1) → (14, 14) | 36 | 0,14 ms | Alvo alcançado |

No `test.csv`, que não tem paredes internas, a rota de 25 movimentos é igual à
distância de Manhattan entre início e alvo (12 linhas + 13 colunas), o menor
valor possível. Isso confirma, em um caso fácil de verificar à mão, que a rota
encontrada é ótima.

### Exemplo de saída (`1.csv`)

O nó imprime o labirinto com a rota planejada (`█` parede, `R` robô, `T` alvo,
`·` rota):

```text
[INFO] [part1_navigator]: Mapa recebido: 29x29, robô em (1, 1), alvo em (14, 14)
[INFO] [part1_navigator]: Rota ótima (BFS): 102 movimentos, calculada em 0.20 ms
█████████████████████████████
█R█           █         █   █
█·█ ███████ █ █████ ███ █ █ █
█·█   █   █ █ █     █ █ █ █ █
█·███ █ ███ █ █ █████ █ █ █ █
█·█ █ █   █ █   █       █ █ █
█·█ █ ███ █ █████ █████████ █
█·█     █ █ █   █ █       █ █
█·█████ █ █ █ ███ █ █████ █ █
█···█     █ █   █ █ █   █ █ █
███·███████ ███ █ █ █ █ █ █ █
█ █·█·····█   █   █ █ █ █   █
█ █·█·███·███ ███ █ ███ ███ █
█···█·█ █···█   █   █     █ █
█·███·█ ███·█ T·█████ █████ █
█·····█···█·█ █·█   █       █
███████·█·█·███·█ █ █ ███████
█     █·█···█···█ █   █     █
█ █████·█████·███ █████ █ ███
█·····█·█    ·█ █   █   █   █
█·███·█·█ ███·█ ███ █ █████ █
█···█·█·█ █···█   █ █     █ █
███·█·█·█ █·███ ███ █ █████ █
█ █·█···█ █·█   █   █ █   █ █
█ █·███████·███ █ ███ █ █ █ █
█ █·█  ···█·█   █ █   █ █   █
█ █·███·█·█·█ ███ █████ ███ █
█  ·····█···█           █   █
█████████████████████████████
[INFO] [part1_navigator]: Alvo alcançado em (14, 14) com 102 movimentos!
```

## Testes unitários

[`src/nav_auto/test/test_grid.py`](../src/nav_auto/test/test_grid.py) testa
a BFS sem precisar do simulador:

- reconstrução da matriz a partir da lista achatada;
- caminho mais curto em um labirinto pequeno com duas rotas de mesmo tamanho;
- conversão de caminho em comandos;
- alvo inalcançável retorna `None`;
- o caminho nunca passa por paredes.

```bash
PYTHONPATH=src/nav_auto python3 -m pytest src/nav_auto/test
```

Resultado: `5 passed`.
