# DeltaV - TuningTool

O **DeltaV - TuningTool** é uma aplicação de interface gráfica (GUI) desenvolvida em Python para monitorização de telemetria e ajuste (tuning) de parâmetros PID de drones em tempo real, utilizando o protocolo MAVLink (via DroneKit).

A ferramenta permite visualizar dados de atitude e canais RC, bem como ajustar os ganhos dos controladores de Roll, Pitch e Yaw de forma interativa.

## 🚀 Funcionalidades

* **Monitorização de Telemetria:**
    * Visualização gráfica em tempo real de **Roll, Pitch e Yaw**.
    * Monitorização de entradas (RCIN) e saídas (RCOUT) dos canais RC.
    * Interface de gráficos interativa com zoom, pause e seleção de dados visíveis.
* **Ajuste de PID (Tuning):**
    * Leitura e escrita de parâmetros PID do drone (P, I, D e IMAX).
    * Seleção de eixos para ajuste individual (Pitch, Roll, Yaw).
    * **Modo Link:** Possibilidade de vincular e ajustar Roll e Pitch simultaneamente.
    * Multiplicador ajustável para incrementos finos ou grosseiros nos valores.
* **Controlo Básico:**
    * Funções para **Armar** e **Pousar (Land)** o veículo diretamente pela interface.
* **Interface Moderna:**
    * Baseada em `customtkinter` com tema escuro (Dark Mode).

## 🛠️ Pré-requisitos

Para executar este projeto, necessita de ter o **Python 3** instalado e as seguintes bibliotecas:

* `customtkinter`
* `matplotlib`
* `dronekit`

Pode instalar as dependências através do comando:

```bash
pip install customtkinter matplotlib dronekit
```

## ⚙️ Configuração de Conexão

Por defeito, o script tenta conectar-se a uma simulação local ou a uma porta série específica. Verifique o ficheiro `main.py` para ajustar a string de conexão conforme o seu ambiente:

* **Simulação (SITL):** `udp:127.0.0.1:14550`
* **Hardware (Rádio/USB):** `/dev/ttyUSB0` (ajuste a porta COM/TTY conforme necessário).

Pode alterar a variável `simulado` no início do ficheiro `main.py` para alternar entre estes modos:

```python
simulado = True  # Define se usa UDP local ou Serial
```

## ▶️ Como Utilizar

1.  Conecte o seu drone ou inicie a simulação (SITL).
2.  Execute o ficheiro principal:
    ```bash
    python main.py
    ```
3.  **Na Interface:**
    * **Conexão:** O status mudará para "CONECTADO (MAVLink)" assim que o veículo for detetado.
    * **Ler Parâmetros:** Clique em "LER PARÂMETROS" para carregar os PIDs atuais do drone.
    * **Ajustes:** Selecione o eixo (Roll/Pitch/Yaw) e utilize os botões `+` e `-` para alterar os valores de P, I, D ou IMAX.
    * **Enviar:** Clique em "ENVIAR PARÂMETROS" para gravar as alterações no drone.
    * **Telemetria:** Clique em "ABRIR TELEMETRIA" para ver os gráficos de resposta em tempo real.

## 📂 Estrutura do Projeto

* `main.py`: Código principal da aplicação, interface gráfica e lógica MAVLink.
* `plot_async.py`: Módulo auxiliar para gestão de filas de dados e plotagem assíncrona.
* `icon.png` / `icon.ico`: Ícones da aplicação.

## ⚠️ Notas Importantes

* Esta ferramenta altera parâmetros críticos de voo (`ATC_RAT_RLL`, `ATC_RAT_PIT`, `ATC_RAT_YAW`). Utilize com cautela, especialmente em voos reais.
* Se a conexão falhar, a ferramenta entra em modo de "Simulação" de interface para testes de UI, mas não enviará comandos.