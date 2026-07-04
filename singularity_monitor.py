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

# Carrega o modelo cinemático do UR10
ur10 = rtb.models.UR10()
LIMITE_SINGULARIDADE = 0.0025 # Limite crítico para o teste

# 2. Definindo os pontos (Ângulos em radianos)
base_angle = math.radians(131.08) # Ângulo especifico para melhor vizualização na simulação

# Posição segura de repouso (Braço dobrado em L)
# Base: 131, Ombro: -90, Cotovelo: 90, Punho 1: -90, Punho 2: -90, Punho 3: 0
pos_safety = [base_angle, math.radians(-90), math.radians(90), math.radians(-90), math.radians(-90), 0]


# Posição de Singularidade de Limite de Espaço (Braço totalmente esticado)
# Cotovelo vai para 0 graus. O braço vira um "bastão" reto.
pos_singularidade = [base_angle, math.radians(-90), math.radians(0), math.radians(-90), math.radians(-90), 0]

try:
    print("\n--- PASSO 1: Indo para a posição inicial segura ---")
    rtde_c.moveJ(pos_safety, speed=0.5, acceleration=1.0)
    time.sleep(2)

    print("\n--- PASSO 2: Esticando o braço em direção à singularidade ---")
    # asynchronous=True faz o código Python continuar na linha de baixo imediatamente
    rtde_c.moveJ(pos_singularidade, speed=0.3, acceleration=0.3, asynchronous=True)

    # 3. Loop de Monitoramento
    while True:
        
        # Lê a posição em tempo real
        pos_actual = rtde_r.getActualQ()
        
        # Condição de parada caso ele chegue no destino
        distancia_para_alvo = sum(abs(atual - alvo) for atual, alvo in zip(pos_actual, pos_singularidade))
        if distancia_para_alvo < 0.05:
            print("Robô chegou no alvo antes de acionar a proteção.")
            break
        
        # Calcula o Jacobiano e a manipulabilidade
        J = ur10.jacob0(pos_actual)
        manipulabilidade = np.sqrt(max(0, np.linalg.det(J @ J.T)))
        
        # Converte o ângulo do Cotovelo (Junta 3) para graus para mostrar na tela
        cotovelo_graus = math.degrees(pos_actual[2])
        
        print(f"Cotovelo: {cotovelo_graus:>6.2f} graus | Manipulabilidade: {manipulabilidade:.4f}")
        
        # Atuação do Monitor de Singularidade
        if manipulabilidade < LIMITE_SINGULARIDADE:
            print("\n" + "="*50)
            print("🚨 ALERTA CRÍTICO: LIMITE DE SINGULARIDADE (BRAÇO ESTICADO) 🚨")
            print("="*50)
            
            # Comando para frear o robô imediatamente
            rtde_c.stopJ(2.0)
            print("Robô freado com sucesso antes da singularidade fatal!")
            break 
            
        time.sleep(0.02) # Roda a 50Hz para não sobrecarregar a rede

    print("\nTeste finalizado.")

except KeyboardInterrupt:
    print("\nTeste interrompido pelo usuário.")
    rtde_c.stopJ(2.0)

finally:
    # Sempre desconectar limpo
    rtde_c.disconnect()
    rtde_r.disconnect()