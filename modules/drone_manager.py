import time
import math
from decimal import Decimal
import collections
from ardupilot_log_reader import Ardupilot
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
from pymavlink import mavutil
try:
    from scipy.spatial.transform import Rotation as R
    scipy_disponivel = True
except ImportError:
    scipy_disponivel = False

from .telemetry_buffer import submit_data

def criar_conexao(connection_string, baud):
    return connect(connection_string, baud=baud)

class DroneManager:
    def __init__(self, connection_string, baud):
        self.connection_string = connection_string
        self.baud = baud
        self.vehicle = None
        self.connected = False
        self.log_callback = print
        
        self.curr_rpy = (0.0, 0.0, 0.0)
        self.target_rpy = (0.0, 0.0, 0.0)
        self.baud = baud
        self.vehicle = None
        self.connected = False
        self.log_callback = print

    def set_logger(self, callback):
        self.log_callback = callback

    def log(self, msg):
        self.log_callback(msg)

    def conectar(self):
        try:
            self.log(f"Conectando em {self.connection_string}...")
            self.vehicle = connect(self.connection_string, baud=self.baud, timeout=60)
            self.connected = True
            self.log("Drone Conectado!")
            self._configurar_listeners()
            self._request_mavlink_streams()
            return True
        except Exception as e:
            self.log(f"Erro conexão: {e}")
            return False

    def fechar(self):
        if self.vehicle:
            self.vehicle.close()

    def armar(self):
        if not self.vehicle: return
        try:
            self.vehicle.mode = VehicleMode("STABILIZE")
            self.vehicle.armed = True
            self.log("Comando ARMAR enviado.")
        except Exception as e:
            self.log(f"Erro Arm: {e}")

    def desarmar(self):
        if not self.vehicle: return
        try:
            self.vehicle.armed = False
            self.log("Comando DESARMAR enviado.")
        except Exception as e:
            self.log(f"Erro Disarm: {e}")

    def pousar(self):
        if not self.vehicle: return
        try:
            self.vehicle.mode = VehicleMode("LAND")
            self.log("Modo LAND enviado!")
        except Exception as e:
            self.log(f"Erro LAND: {e}")

    # --- Gerenciamento de PIDs ---
    def ler_pids(self):
        if not self.vehicle: return {}
        # Lista simplificada para leitura. Adicione mais se precisar.
        keys = [
            'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_RAT_RLL_IMAX',
            'ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D', 'ATC_RAT_PIT_IMAX',
            'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D', 'ATC_RAT_YAW_IMAX'
        ]
        pids = {}
        for k in keys:
            val = self.vehicle.parameters.get(k, 0)
            pids[k] = Decimal(str(val)).quantize(Decimal("0.000001"))
        return pids

    def enviar_pids(self, pids_dict):
        if not self.vehicle: return
        self.log("Enviando PIDs...")
        try:
            for param, valor in pids_dict.items():
                self.vehicle.parameters[param] = float(valor)
            self.log("PIDs enviados com sucesso.")
        except Exception as e:
            self.log(f"Erro ao enviar PIDs: {e}")

    def processar_log_posicao(self, caminho_arquivo):
        """
        Lê o arquivo .bin e extrai dados de posição (North, East, Down).
        """
        try:
            self.log(f"Processando log: {caminho_arquivo}")
            log_data = Ardupilot.parse(caminho_arquivo, types=['PSCN', 'PSCE', 'PSCD'])
            dados_posicao = {
                'north': log_data.PSCN, # DataFrame com campos TP (Target) e P (Actual)
                'east':  log_data.PSCE,
                'down':  log_data.PSCD
            }
            
            self.log("Log de posição processado com sucesso.")
            return dados_posicao
        except Exception as e:
            self.log(f"Erro ao ler log: {e}")
            return None

    # --- Callbacks e Telemetria ---
    def _configurar_listeners(self):
        self.vehicle.add_attribute_listener('attitude', self._cb_attitude)
        self.vehicle.add_message_listener('ATTITUDE_TARGET', self._cb_att_target)
        self.vehicle.add_message_listener('RC_CHANNELS', self._cb_rc)
        self.vehicle.add_message_listener('SERVO_OUTPUT_RAW', self._cb_servo)

    def _request_mavlink_streams(self):
        # IDs das mensagens MAVLink:
        # 30: ATTITUDE (Roll, Pitch, Yaw reais)
        # 83: ATTITUDE_TARGET (Desejados)
        # 65: RC_CHANNELS (Inputs do rádio)
        # 36: SERVO_OUTPUT_RAW (Saídas para motores)
        # Intervalo em microssegundos (20000us = 50Hz)
        interval_us = 10000 
        
        ids_to_request = [30, 83, 65, 36]

        for msg_id in ids_to_request:
            try:
                # Comando MAV_CMD_SET_MESSAGE_INTERVAL (511)
                msg = self.vehicle.message_factory.command_long_encode(
                    0, 0, 511, 0, msg_id, interval_us, 0, 0, 0, 0, 0
                )
                self.vehicle.send_mavlink(msg)
            except Exception as e:
                self.log(f"Erro ao pedir stream {msg_id}: {e}")

    def _calcular_erro_angular(self, target, current):
        diff = target - current
        # Normaliza para -180 a 180
        diff = (diff + 180) % 360 - 180
        return diff

    # --- Callbacks Atualizados ---
    def _cb_attitude(self, _, attr_name, value):
        r = math.degrees(value.roll)
        p = math.degrees(value.pitch)
        y = math.degrees(value.yaw)
        
        self.curr_rpy = (r, p, y)
        
        submit_data('roll', r)
        submit_data('pitch', p)
        submit_data('yaw', y)
        
        # CÁLCULO DE ERRO CENTRALIZADO AQUI
        tr, tp, ty = self.target_rpy
        submit_data('err_roll', self._calcular_erro_angular(tr, r))
        submit_data('err_pitch', self._calcular_erro_angular(tp, p))
        submit_data('err_yaw', self._calcular_erro_angular(ty, y))

    def _cb_att_target(self, _, name, msg):
        if hasattr(msg, 'q'):
            w, x, y, z = msg.q
            if scipy_disponivel:
                # CORREÇÃO 1: Ordem ZYX (Padrão Aeroespacial)
                rot = R.from_quat([x, y, z, w])
                ya_temp, p_temp, r_temp = rot.as_euler('zyx', degrees=True)
                r, p, ya = r_temp, p_temp, ya_temp
            else:
                r, p, ya = self._quaternion_to_euler_manual(w, x, y, z)
            
            self.target_rpy = (r, p, ya)
            
            submit_data('des_roll', r)
            submit_data('des_pitch', p)
            submit_data('des_yaw', ya)

            # CORREÇÃO 2: Removido o cálculo de erro duplicado daqui para evitar ruído no gráfico

    def _cb_rc(self, _, name, msg):
        submit_data('rcin1', msg.chan1_raw)
        submit_data('rcin2', msg.chan2_raw)
        submit_data('rcin3', msg.chan3_raw)
        submit_data('rcin4', msg.chan4_raw)

    def _cb_servo(self, _, name, msg):
        submit_data('rcout1', msg.servo1_raw)
        submit_data('rcout2', msg.servo2_raw)
        submit_data('rcout3', msg.servo3_raw)
        submit_data('rcout4', msg.servo4_raw)

    def _quaternion_to_euler_manual(self, w, x, y, z):
        # Backup manual caso Scipy falhe
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)