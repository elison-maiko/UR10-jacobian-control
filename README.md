# UR10-jacobian-control

### 1. Fundamentação Teórica: O Jacobiano do UR10 e suas Propriedades Físicas

Na modelagem de manipuladores robóticos como o UR10, a cinemática direta estabelece a relação não-linear entre o espaço das juntas e o espaço cartesiano. No entanto, para fins de controle dinâmico, monitoramento de segurança e interação com o ambiente, é necessário analisar as taxas de variação dessas grandezas. É nesse contexto que o Jacobiano Geométrico ($J$) atua como a ferramenta matemática central[cite: 1].

O Jacobiano é uma matriz de derivadas parciais que atua como um elemento de transformação dual[cite: 1]. Para um manipulador de 6 graus de liberdade como o UR10, $J$ é uma matriz quadrada $6 \times 6$, dependente da configuração atual das juntas ($\theta$), que mapeia as velocidades das juntas para o espaço cartesiano[cite: 1]:

$$\dot{p}=J\dot{\theta}$$

Onde $\dot{p}$ representa o vetor de velocidades cartesianas (lineares e angulares) do efetor final, e $\dot{\theta}$ é o vetor de velocidades das articulações. 

Além da cinemática de velocidades, o Jacobiano dita o comportamento físico e a estabilidade estrutural do robô por meio de suas propriedades algébricas[cite: 1]. A compreensão destas propriedades é o que fundamenta as soluções de engenharia implementadas neste projeto.

#### Propriedades da Matriz Jacobiana e seus Significados Físicos

* **Determinante ($\det(J)$): O Indicador de Singularidade**
    * **Significado Físico:** O valor absoluto do determinante mede o "volume" do elipsóide de manipulabilidade de velocidades do robô. Quando o robô estica completamente o braço ou alinha determinados eixos de rotação, esse volume entra em colapso. Matematicamente, quando $\det(J) \to 0$, ocorrem as chamadas zonas de instabilidade matemática (singularidades)[cite: 1]. Fisicamente, o efetor final perde a capacidade de se mover em pelo menos uma direção cartesiana, podendo provocar falhas críticas, desligamentos de proteção ou danos estruturais[cite: 1].
    * **Fundamentação da Solução:** A Aplicação de Proteção e Monitoramento Ativo utiliza a avaliação das condições geométricas em tempo real[cite: 1]. Ao estabelecer um limite de tolerância próximo a zero para o determinante, o algoritmo atua na interrupção de movimentos perigosos antes que o hardware desarme[cite: 1].

* **Posto (Rank) da Matriz: Graus de Liberdade Efetivos**
    * **Significado Físico:** O posto indica o número de direções cartesianas independentes nas quais o robô pode gerar velocidade ou aplicar força. Um UR10 operando em condições normais possui posto 6. Se o posto cai para 5 ou menos, ocorre uma singularidade cinemática. 
    * **Fundamentação da Solução:** O cálculo do posto valida se a matriz é inversível naquele instante específico, o que é um pré-requisito obrigatório para explorar as propriedades matemáticas do manipulador de forma segura[cite: 1].

* **Número de Condição ($\kappa(J)$): Índice de Esforço e Isotrópia**
    * **Significado Físico:** Calculado pela razão entre o maior e o menor valor singular da matriz, o número de condição avalia quão bem "condicionada" está a postura do robô. Um $\kappa(J)$ muito alto significa que movimentos ao longo de uma direção específica exigirão velocidades articulares perigosamente altas, gerando estresse mecânico.
    * **Fundamentação da Solução:** O condicionamento da matriz serve como guia para o comportamento do robô, protegendo os motores contra sobrecargas induzidas por posturas desvantajosas[cite: 1].

* **Transposição ($J^T$): A Dualidade Força-Torque**
    * **Significado Físico:** Pelo Princípio do Trabalho Virtual, o Jacobiano que mapeia velocidades também atua no domínio estático mapeando as forças exercidas no efetor final para os torques internos dos motores[cite: 1]. Esta relação não requer a inversão da matriz, apenas a sua transposição:
    $$\tau=J^TF$$
    * **Fundamentação da Solução:** A Aplicação de Mapeamento e Controle Baseado em Força/Torque baseia-se diretamente nesta dualidade do Jacobiano para estimar esforços[cite: 1]. Isso permite gerenciar a conformidade mecânica do braço ao interagir com o ambiente[cite: 1].
