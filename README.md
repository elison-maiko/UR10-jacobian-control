# TS1: Engenharia de Controle e Segurança via Jacobiano no UR10

**Disciplina:** EMB5615-09605 (20261) - Robótica e Sistemas Mecatrônicos
**Instituição:** Universidade Federal de Santa Catarina (UFSC)

Este repositório contém o desenvolvimento e a implementação de soluções de controle e segurança para o manipulador industrial Universal Robots UR10, utilizando o simulador URSim. O projeto baseia-se na exploração das propriedades matemáticas e físicas do **Jacobiano Geométrico**, transformando matrizes e derivadas em algoritmos práticos de proteção e monitoramento dinâmico.

## 1. Fundamentação Teórica: O Jacobiano do UR10 e suas Propriedades Físicas

Na modelagem de manipuladores robóticos como o UR10, a cinemática direta estabelece a relação não-linear entre o espaço das juntas (ângulo dos motores) e o espaço cartesiano (posição e orientação da ferramenta). No entanto, para o controle dinâmico, monitoramento de segurança e interação com o ambiente, é fundamental analisar as taxas de variação dessas grandezas. É neste cenário que o **Jacobiano Geométrico (**$J$**)** atua como a ferramenta matemática central.

O Jacobiano é uma matriz de derivadas parciais que opera como um elemento de transformação dual. Para o UR10 (6 graus de liberdade), $J$ é uma matriz $6 \times 6$ dependente da configuração atual das juntas ($q$ ou $\theta$), que mapeia as velocidades articulares para o espaço cartesiano:

$$\dot{p} = J(q) \dot{q}$$

Onde:
* $\dot{p}$: É o vetor de velocidades cartesianas (lineares e angulares) do efetor final (TCP).
* $J(q)$: É a Matriz Jacobiana (que depende da posição atual $q$ das juntas).
* $\dot{q}$: É o vetor de velocidades das articulações (motores).

Além da cinemática de velocidades, o Jacobiano dita o comportamento físico e a estabilidade estrutural do robô por meio de suas propriedades algébricas. A compreensão destas propriedades fundamenta as soluções de engenharia implementadas neste trabalho:

### 1.1. Determinante ($\det(J)$): O Indicador de Singularidade

* **Significado Físico:** O determinante avalia o volume do *elipsoide de manipulabilidade* do robô. Quando o robô estica completamente o braço (limite de espaço) ou alinha determinados eixos de rotação (singularidade de punho), esse volume entra em colapso, resultando em $\det(J) \to 0$. Nessas zonas de instabilidade matemática (Singularidades), o efetor final perde graus de liberdade e a cinemática inversa exige velocidades articulares infinitas, causando paradas bruscas e erros críticos no hardware.
* **Aplicação Prática (Neste Projeto):** Utilizamos a medida de manipulabilidade de Yoshikawa, monitorando este valor em tempo real para criar um sistema de **Proteção Ativa**, freando o robô preventivamente antes que o controlador original desarme. Essa medida é definida por:

$$w = \sqrt{\det(J \cdot J^T)}$$

### 1.2. Transposição ($J^T$): A Dualidade Força-Torque

* **Significado Físico:** Baseado no Princípio do Trabalho Virtual, o mesmo Jacobiano que mapeia velocidades atua no domínio da estática. Ele converte as forças e momentos exercidos no efetor final ($F$) para os torques internos requeridos em cada motor ($\tau$). Esta relação não requer a inversão da matriz, apenas a sua transposição:

$$\tau = J^T F$$

* **Aplicação Prática (Neste Projeto):** Exploramos essa propriedade para criar um algoritmo de **Mapeamento Dinâmico de Torque**. Simulamos a manipulação de uma carga pesada e calculamos, em tempo real, a distribuição de esforço mecânico exigida por cada junta durante a trajetória.

### 1.3. Posto (Rank da Matriz): Graus de Liberdade Efetivos

* **Significado Físico:** O posto da matriz Jacobiana indica o número de direções cartesianas linearmente independentes nas quais o efetor final pode se mover instantaneamente. Um manipulador como o UR10 possui posto máximo 6 em operações normais. Quando o posto cai (ex: para 5), o robô perde a capacidade física de transladar ou rotacionar em eixos específicos do espaço.
* **Aplicação Prática (Neste Projeto):** A garantia de que o posto da matriz se mantém máximo é o que valida matematicamente a capacidade do robô de operar sem travar. O nosso sistema de proteção atua justamente para impedir que o robô atinja posturas onde a matriz perca o seu posto completo.

### 1.4. Número de Condição ($\kappa(J)$): Índice de Esforço e Isotrópia

* **Significado Físico:** Calculado através da razão entre o maior e o menor valor singular da matriz, o número de condição ($\kappa(J)$) mede o grau de isotrópia cinemática. Valores muito altos indicam mau condicionamento mecânico: o robô terá extrema facilidade de se mover em uma direção, mas exigirá velocidades ou torques perigosamente altos nos motores para atuar em outra direção.
* **Aplicação Prática (Neste Projeto):** O número de condição justifica a necessidade do mapeamento dinâmico de torque. Ele fundamenta teoricamente a observação de que levantar os mesmos 10 kg com o braço esticado (mau condicionamento, $\kappa$ elevado) ou com o braço contraído (bom condicionamento, $\kappa$ próximo de 1) gera respostas de estresse mecânico completamente diferentes nas articulações do UR10.

## 2. Arquitetura da Solução e Scripts Desenvolvidos

A comunicação com o simulador URSim foi estabelecida via rede local utilizando a biblioteca `ur_rtde` (Real-Time Data Exchange), operando a uma frequência estável para garantir segurança e precisão. A modelagem matemática foi extraída via `roboticstoolbox-python`.

Foram desenvolvidos dois scripts principais abordando eixos temáticos distintos:

### A. Monitor de Singularidade Ativo (`singularity_test.py`)

**Eixo Temático:** Sistemas de Proteção e Monitoramento Ativo.

Este script atua como um supervisor de segurança. Ele força propositalmente o robô a se mover em direção a uma **Singularidade de Limite de Espaço** (esticamento total do cotovelo) e intercepta o movimento momentos antes do colapso matemático.

* **Passo a Passo da Execução:**
  1. O script se conecta ao URSim via portas RTDE (30003 e 30004).
  2. Envia comandos de movimento (`moveJ`) para posicionar o robô em uma postura segura (braço em "L").
  3. Inicia um movimento assíncrono esticando o Cotovelo (Junta 3) em direção a 0 graus.
  4. Em um loop de alta frequência, lê a posição real das juntas (`getActualQ`).
  5. Computa a matriz Jacobiana correspondente àquele milissegundo e extrai a **Manipulabilidade de Yoshikawa**.
  6. **Ação de Proteção:** Quando o índice cai abaixo do limite crítico seguro ($w < 0.03$), o script substitui o movimento atual por um comando de parada de emergência (`stopJ`), preservando o robô e reportando o alerta no terminal.

### B. Mapeamento Dinâmico de Torque (`pick_and_place_torque.py`)

**Eixo Temático:** Mapeamento e Controle Baseado em Força/Torque.

Este script simula uma operação industrial clássica de *Pick and Place*, evidenciando o comportamento elástico da matriz transposta sob carga variável.

* **Passo a Passo da Execução:**
  1. Define dois vetores de Força Cartesiana ($F$): Garra vazia ($0$ N) e Garra com Carga simulando $10$ kg ($-100$ N no eixo Z).
  2. O robô inicia um ciclo completo: Vai até a posição A, abaixa, "pega" o peso, levanta, rotaciona a base, vai até a posição B e "solta" o peso.
  3. Durante toda a trajetória, o script lê as posições articulares e calcula $J^T$.
  4. A equação $\tau = J^T F$ é resolvida dinamicamente baseada no status atual da garra.
  5. O terminal exibe, em tempo real, o torque teórico exigido em **cada um dos 6 motores**.
  6. **Análise Evidenciada:** É possível observar que, com a garra vazia, o torque induzido é nulo. Ao levantar a carga, as juntas 2 e 3 (Ombro e Cotovelo) assumem instantaneamente valores elevados, que flutuam dependendo do braço de alavanca (postura momentânea) enquanto o robô transita pelo espaço.

## 3. Pré-requisitos e Execução

### Ambiente Necessário

* Docker com imagem oficial do **URSim** em execução (portas `30004`, `30003` e `29999` expostas).
* Python 3.8+ instalado localmente.

### Bibliotecas Python

Instale as dependências via gerenciador de pacotes:

```bash
pip install ur_rtde roboticstoolbox-python numpy
