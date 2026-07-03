# Instale as dependências:
# pip install ur_rtde roboticstoolbox-python numpy

from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import roboticstoolbox as rtb
import numpy as np
import time

# 1. Configuração do RTDE (Use o IP do seu Docker URSim)
# O padrão no docker local geralmente é 127.0.0.1
IP_URSIM = "127.0.0.1" 
rtde_r = RTDEReceiveInterface(IP_URSIM)
rtde_c = RTDEControlInterface(IP_URSIM)

# 2. Carregar o modelo cinemático exato do UR10
ur10 = rtb.models.UR10()

# Limite de segurança para a singularidade (quanto mais próximo de 0, mais perigoso)
LIMITE_SINGULARIDADE = 0.05 

try:
    print("Monitoramento de Singularidade Iniciado...")
    while True:
        # 3. Ler a posição atual das 6 juntas em radianos
        q_atual = rtde_r.getActualQ()
        
        # 4. Obter a Matriz Jacobiana (6x6) no estado atual
        J = ur10.jacob0(q_atual)
        
        # 5. Calcular a Medida de Manipulabilidade de Yoshikawa
        # w = sqrt(det(J * J_transposta)). Se w se aproxima de 0, é singularidade.
        manipulabilidade = np.sqrt(max(0, np.linalg.det(J @ J.T)))
        
        # 6. Lógica de Proteção
        if manipulabilidade < LIMITE_SINGULARIDADE:
            print(f"ALERTA! Risco de Singularidade. Manipulabilidade: {manipulabilidade:.4f}")
            
            # Exemplo de ação: Parar o movimento suavemente
            # rtde_c.stopL(0.5) # Descomente para ativar a parada
            
        else:
            # Apenas imprime o status seguro
            print(f"Status Normal - Manipulabilidade: {manipulabilidade:.4f}", end='\r')
            
        time.sleep(0.01) # Ciclo de 100Hz

except KeyboardInterrupt:
    print("\nMonitoramento encerrado.")
finally:
    rtde_r.disconnect()
    rtde_c.disconnect()
