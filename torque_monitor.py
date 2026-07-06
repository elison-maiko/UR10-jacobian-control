### Autores:
### Elison Maiko Oliveira de Souza (22102900)
### Italo Miranda Kusmin Alves (22101930)

import time
import math
import numpy as np
import roboticstoolbox as rtb
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


# 1. Configuração Inicial
IP_URSIM = "127.0.0.1"
print(f"Conectando ao URSim no IP {IP_URSIM}...")

rtde_c = RTDEControlInterface(IP_URSIM)
rtde_r = RTDEReceiveInterface(IP_URSIM)
ur10 = rtb.models.UR10()

# 2. Definição das Forças
# Vetor de força: [Fx, Fy, Fz, Mx, My, Mz]
F_vazia = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Garra vazia (ignorando o peso da própria garra)
F_carga = np.array([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]) # Carga de 10kg puxando para baixo (Eixo Z)

# 3. Definição das Posições (Ângulos em radianos)
base_A = math.radians(131.08)
base_B = math.radians(70.0) # Rotaciona a base para colocar a peça em outro lugar

# Ponto A (Lado de Pegar)
pos_alto_A  = [base_A, math.radians(-90), math.radians(90), math.radians(-90), math.radians(-90), 0]
pos_baixo_A = [base_A, math.radians(-60), math.radians(60), math.radians(-90), math.radians(-90), 0]

# Ponto B (Lado de Soltar)
pos_alto_B  = [base_B, math.radians(-90), math.radians(90), math.radians(-90), math.radians(-90), 0]
pos_baixo_B = [base_B, math.radians(-60), math.radians(60), math.radians(-90), math.radians(-90), 0]

# 4. Função Mágica: Move e calcula torque em tempo real
def mover_e_monitorar(pos_alvo, F_externa, nome_movimento):
    print(f"\n--- {nome_movimento} ---")
    
    # Inicia o movimento sem travar o código
    rtde_c.moveJ(pos_alvo, speed=0.5, acceleration=0.5, asynchronous=True)
    
    while True:
        pos_atual = rtde_r.getActualQ()
        
        # Verifica se chegou
        distancia = sum(abs(atual - alvo) for atual, alvo in zip(pos_atual, pos_alvo))
        if distancia < 0.05:
            print(f"\n[✓] Destino alcançado!")
            break
            
        # Calcula o Jacobiano Transposto e o Torque
        J = ur10.jacob0(pos_atual)
        torques = J.T @ F_externa
        
        # Formata a string para o terminal e imprime na mesma linha
        t_str = " | ".join([f"J{i+1}: {torques[i]:>6.2f}" for i in range(6)])
        print(f"\rTorques (Nm): [ {t_str} ]", end="")
        
        time.sleep(0.05) # 20Hz

# 5. Execução do Cenário
try:
    print("\nIniciando Ciclo Pick and Place...")
    
    # ETAPA 1: Vai buscar (Vazio)
    mover_e_monitorar(pos_alto_A, F_vazia, "1. Indo para Posição de Espera (Vazio)")
    mover_e_monitorar(pos_baixo_A, F_vazia, "2. Abaixando para pegar peça (Vazio)")
    
    # SIMULAÇÃO DE PEGAR A PEÇA (Pausa rápida)
    print("\n>>> PEGANDO A PEÇA (10 KG) <<<")
    time.sleep(1)
    
    # ETAPA 2: Transporta (Cheio)
    mover_e_monitorar(pos_alto_A, F_carga, "3. Levantando a peça (Carga Máxima!)")
    mover_e_monitorar(pos_alto_B, F_carga, "4. Movendo para o outro lado (Carga Máxima!)")
    mover_e_monitorar(pos_baixo_B, F_carga, "5. Abaixando para soltar (Carga Máxima!)")
    
    # SIMULAÇÃO DE SOLTAR A PEÇA
    print("\n>>> SOLTANDO A PEÇA <<<")
    time.sleep(1)
    
    # ETAPA 3: Retorna (Vazio)
    mover_e_monitorar(pos_alto_B, F_vazia, "6. Levantando garra (Vazio)")
    mover_e_monitorar(pos_alto_A, F_vazia, "7. Retornando ao Início (Vazio)")
    
    print("\nCiclo finalizado com sucesso!")

except KeyboardInterrupt:
    print("\n\nOperação interrompida.")
    rtde_c.stopJ(2.0)

finally:
    rtde_c.disconnect()
    rtde_r.disconnect()