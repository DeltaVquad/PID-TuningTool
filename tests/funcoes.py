from dronekit import VehicleMode
from pymavlink import mavutil
# import utm
import time
from typing import Tuple
import math
from simple_pid import PID

def gps_global_origin(logan):
    latitude = logan.location.global_frame.lat
    longitude = logan.location.global_frame.lon
    altitude = logan.location.global_relative_frame.alt

    msg = logan.message_factory.set_gps_global_origin_encode(
        0,
        int(latitude*1e7),
        int(longitude*1e7),
        int(altitude*1000),
        0
    )
    logan.send_mavlink(msg)


def calcular_distancia(logan, pos2_x, pos2_y, pos2_z):

    pos1_x, pos1_y, pos1_z, _, _, _, _ = estado_drone(logan)  # posicao inicial

    dist_x = abs(pos2_x) - abs(pos1_x)
    dist_y = abs(pos2_y) - abs(pos1_y)
    dist_z = abs(pos2_z) - abs(pos1_z)

    return dist_x, dist_y, dist_z


def pixel2distancia(info_alvo, h_fov_deg: float, h_pixels: int) -> Tuple[float, float]:
    # 1. Converte o erro de pixels para metros
    distancia_m = info_alvo.distancia_m
    offset_x_px, offset_y_px = info_alvo.offset

    if distancia_m <= 0 or h_pixels <= 0:
        return 0.0, 0.0, distancia_m # Evita divisão por zero se os dados estiverem inválidos

    # Calcula a largura da visão da câmera (em metros) na distância do alvo
    largura_visao_m = 2 * distancia_m * math.tan(math.radians(h_fov_deg / 2))
    metros_por_pixel = largura_visao_m / h_pixels

    erro_x_m = offset_x_px * metros_por_pixel
    erro_y_m = offset_y_px * metros_por_pixel

    return erro_x_m, erro_y_m, distancia_m


def armar(logan):
    print("Armando Logan...")
    #x, y, _, _, _, _, _ = estado_drone(logan)
    vehicle_mode = VehicleMode("GUIDED")
    #vehicle_mode = VehicleMode("GUIDED_NOGPS")
    
    while logan.mode != vehicle_mode or logan.armed == False:
        logan.mode = vehicle_mode
        logan.armed = True
        time.sleep(1)
        
    print("Modo setado como GUIADO e logan ARMADO!")
    
    # --- CORREÇÃO AQUI ---
    # Verifica se x e y não são None antes de formatar
    #x_str = f"{x:.2f}" if x is not None else "N/A"
    #y_str = f"{y:.2f}" if y is not None else "N/A"
    
    #print(f"Posição inicial (X: {x_str}m, Y: {y_str}m).")


def desarmar(logan):
    print("Desarmando Logan...")
    while logan.armed:
        logan.armed = True
        time.sleep(1)
    print('Logan desarmando')

#------------------------------#

def decolar(logan, altitude):
    print(F"Decolando até {altitude}m...")
    logan.simple_takeoff(altitude)

    while True:
        #print(F"Altitude = {logan.location.global_relative_frame.alt}m")

        while logan.location.global_relative_frame.alt < altitude:
            _, _, altitude_atual, _,_,_,_ = estado_drone(logan)
            print(f"Altitude atual: {altitude_atual}m")
            time.sleep(1)
            if logan.location.global_relative_frame.alt >= altitude*0.95:
                print("Atingiu a altitude declarada.")
                break
            time.sleep(1)

        time.sleep(1)
        break

#------------------------------#

def mudar_param(logan, nome_param, valor): #se ligar na unidade de medida do valor do param!!!!!!!!
    print(f"Mudando o parametro {nome_param} para {valor}")
    logan.parameters[nome_param] = valor
    time.sleep(0.5)
    leitura_de_valor = logan.parameters[nome_param]
    if math.isclose(leitura_de_valor, valor):
        print(f"Parametro {nome_param} mudado com SUCESSO para {leitura_de_valor}")
        return True

    else:
        print(f"Parametro {nome_param} FALHOU na mudanca de valor.")
        return False

#------------------------------#

def pousar(logan):
    vehicle_mode = VehicleMode("LAND")
    x, y, z, _, _, _, _ = estado_drone(logan)
    # esperar modo LAND
    while logan.mode != vehicle_mode:
        logan.mode = vehicle_mode
        time.sleep(1)
    print("Inicializando pouso...")
    # esperar altitude
    while z >= 0.02:
        _, _, z, _, _, _, _ = estado_drone(logan)
        time.sleep(1)
    print("Logan pousado!")

#------------------------------#

def rtl(logan):
    logan.mode = VehicleMode("RTL")

    if logan.mode.name != "RTL":
        logan.mode = VehicleMode("RTL")

    while not logan.mode.name == "RTL":
        time.sleep(1)

    print("Logan está no modo RTL.")
    while logan.mode.name == "RTL":
        time.sleep(1)

    while logan.location.global_relative.alt > 0.02:
        _,_, altitude_atual,_,_,_,_ = estado_drone(logan)
        print(f"Altitude atual enquanto executan LAND: {altitude_atual}")

    if logan.location.global_relative_frame.alt <= 0.02:
        logan.mode = VehicleMode("LAND")
        print("Altura baixa o suficiente para realizar LAND...")
        logan.armed = False
        print("Logan pousado e desarmado.")
#------------------------------#

def servo(logan, channel, pwm):
    logan._master.mav.command_long_send(
        logan._master.target_system,
        logan._master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,    # Canal servo (7 ou 8)
        pwm,        # PWM value (ex: 1000-2000)
        0, 0, 0, 0, 0
    )

#------------------------------#


def liberar_gancho(logan):
    pwm_open = 1900
    pwm_close = 1100

    print("Liberando gancho...")
    servo(logan, 7, pwm_open) #ver os pinos
    time.sleep(0.2)
    servo(logan, 8, pwm_open) #ver os pinos
    time.sleep(0.2)

#------------------------------#

def estado_drone(logan):
    # print("ESTADO DO LOGAN:")
    altitude_atual = logan.location.global_relative_frame.alt
    x = logan.location.local_frame.north # latitude para m
    y = logan.location.local_frame.east # longitude para m

    pitch_graus = math.degrees(logan.attitude.pitch)
    roll_graus = math.degrees(logan.attitude.roll)
    yaw_graus = math.degrees(logan.attitude.yaw)
    nivel_bateria = logan.battery.level

    '''
    -> MINIMO DE VOLTAGEM PARA COMPETICAO!! retornar isso no script das missoes.

    if logan.battery.voltage <= 15.95:
        print(f"Bateria abaixo do determinado pela SAE..")
        print("\nAcionando LAND...")
        logan.mode = VehicleMode("LAND")

        while logan.mode.name != "LAND":
            time. sleep(0.5)
        print("MOdo LAND ativado. Drone pousando...")
        if logan.location.global_relative_frame.alt <= 0.02:
            logan.armed = False
            print("Drone desarmado.")
    '''
    return x, y, altitude_atual, pitch_graus, roll_graus, yaw_graus, nivel_bateria

def velocidade(logan, veloc_x=0, veloc_y=0, veloc_z=0, yaw_rate=0, duracao=0.5):

    msg = logan.message_factory.set_position_target_local_ned_encode(
        0,  #time_boot_ms(nao usado)
        0, 0, #target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        # 0b0000111111000111, # type_mask (only speeds enabled)
        0b0000010111000111, # type_mask (only speeds + yawrate enabled)
        0,0,0,  # x, y, z positions (nao usado)
        veloc_x, veloc_y, veloc_z, #m/s
        0,0,0, # x,y, z acceleration (n suportada no GCS mavlink**)
        0,
        yaw_rate)

    end_time = time.time() + duracao
    while time.time() < end_time:
        logan.send_mavlink(msg)
        time.sleep(0.1)

def manter_posicao(logan, duracao):
    print(f"Mantendo a posição do Logan durante {duracao}s...")
    x,y,altitude_atual,_,_,_,_ = estado_drone(logan)
    print(f"Posicao atual do Longa: em x: {x:.2f}, y: {y:.2f} e altitude: {altitude_atual}")
    velocidade(logan, 0, 0, 0, 0, duracao)
    print(f"Fim do tempo em manter o Logan parado.")

def mover_frente(logan, veloc_x, duracao):
    print(f"Exexcutando velocidade para FRENTE a {veloc_x}m/s, durante {duracao}s")
    x,y,_,_,_,_,_ = estado_drone(logan)
    print(F"Posicao em x: {x:.2f} e Posicao em y: {y:.2f}m")
    velocidade(logan,veloc_x, 0, 0, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def mover_tras(logan, veloc_x, duracao):
    print(f"Exexcutando velocidade para TRÁS a {veloc_x}m/s, durante {duracao}s")
    x,y,_,_,_,_,_ = estado_drone(logan)
    print(F"Posicao em x: {x:.2f} e Posicao em y: {y:.2f}m")

    veloc_x = -veloc_x
    velocidade(logan, veloc_x, 0, 0, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def mover_direita(logan, veloc_y, duracao): #direita = +y, esquera = -y
    print(f"Exexcutando velocidade para DIREITA a {veloc_y}m/s, durante {duracao}s")
    x,y,_,_,_,_,_ = estado_drone(logan)
    print(F"Posicao em x: {x:.2f} e Posicao em y: {y:.2f}m")
    velocidade(logan, 0, veloc_y, 0, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def mover_esquerda(logan, veloc_y, duracao):
    print(f"Exexcutando velocidade para ESQUERDA a {veloc_y}m/s, durante {duracao}s")
    x,y,_,_,_,_,_ = estado_drone(logan)
    print(F"Posicao em x: {x:.2f} e Posicao em y: {y:.2f}m")
    veloc_y = -veloc_y
    velocidade(logan, 0, veloc_y, 0, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def mover_descer(logan, veloc_z,duracao): # down = +z, up = -z
    print(f"Exexcutando velocidade para DESCER a {veloc_z}m/s, durante {duracao}s")
    _,_,altitude_atual,_,_,_,_ = estado_drone(logan)
    print(f"Altitude(velocidade z) atual: {altitude_atual}")
    velocidade(logan, 0, 0, veloc_z, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def mover_subir(logan, veloc_z,duracao): # down = +z, up = -z
    print(f"Exexcutando velocidade para DESCER a {veloc_z}m/s, durante {duracao}s")
    _,_,altitude_atual,_,_,_,_ = estado_drone(logan)
    print(f"Altitude(velocidade z) atual: {altitude_atual}")
    veloc_z = -veloc_z
    velocidade(logan, 0, 0, veloc_z, 0, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def girar_direita(logan, yaw_rate, duracao):
    print(f"Exexcutando velocidade para GIRAR a {yaw_rate}rad/s, durante {duracao}s")
    _,_,altitude_atual,_,_,_,_ = estado_drone(logan)
    print(f"Altitude(velocidade z) atual: {altitude_atual}")
    velocidade(logan, 0, 0, 0, yaw_rate, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

def girar_esquerda(logan, yaw_rate, duracao):
    print(f"Exexcutando velocidade para GIRAR a {yaw_rate}rad/s, durante {duracao}s")
    _,_,altitude_atual,_,_,_,_ = estado_drone(logan)
    print(f"Altitude(velocidade z) atual: {altitude_atual}")
    velocidade(logan, 0, 0, 0, -yaw_rate, duracao)
    print(f"Fim do movimento setado.")
    time.sleep(1)

