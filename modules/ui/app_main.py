import sys
import customtkinter as ctk
import time
import os
import threading
import math
import csv
from datetime import datetime
from decimal import Decimal
from tkinter import filedialog 
from pymavlink import mavutil 

from ..drone_manager import DroneManager
from .. import telemetry_buffer
from .window_plot import JanelaTelemetria
from .window_test import JanelaCatalogo

# --- CLASSE AUXILIAR PARA REDIRECIONAR PRINT PARA O LOG ---
class StdoutRedirector:
    def __init__(self, log_func):
        self.log_func = log_func
        self.buffer = ""
        
    def write(self, msg):
        self.buffer += msg
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if line: 
                self.log_func(line)
            
    def flush(self):
        if self.buffer:
            self.log_func(self.buffer)
            self.buffer = ""

class MainApp(ctk.CTk):
    def __init__(self, connection_string, baud_rate):
        super().__init__()
        self.title("DeltaV TuningTool - Modular")
        self.geometry("1100x750")
        
        self.log_history = []
        
        self.drone = DroneManager(connection_string, baud_rate)
        self.drone.set_logger(self.log)
        
        self.axis_mode = "Pitch"
        self.multiplier = Decimal("0.01")
        self.teste_selecionado = None
        self.viewing_log = False
        self.t_inicio_calculo = None
        
        self.pid_vals = {
            'Pitch': {'P':0, 'I':0, 'D':0, 'IMAX':0},
            'Roll': {'P':0, 'I':0, 'D':0, 'IMAX':0},
            'Yaw': {'P':0, 'I':0, 'D':0, 'IMAX':0}
        }
        
        self.grid_columnconfigure(0, weight=1, minsize=400) 
        self.grid_columnconfigure(1, weight=2) 
        self.grid_rowconfigure(0, weight=1)

        self._setup_left_panel()
        self._setup_right_panel()
        
        self.vcmd = self.register(self.validar_multiplier)

        self.after(100, self.update_loop)
        self.after(500, self._conectar_e_inicializar)
        self.after(200, self._atualizar_visual_link)
        
        self.protocol("WM_DELETE_WINDOW", self.mostrar_dialogo_saida)

    def carregar_log_com_ardupilot_reader(self, filename):
        """
        Lê o ficheiro .bin usando a biblioteca ardupilot_log_reader 
        e envia os dados de velocidade (Copter) e seus erros para o telemetry_buffer.
        """
        self.log(f"Processando Log com ArdupilotLogReader: {os.path.basename(filename)}...")
        
        try:
            from ardupilot_log_reader import Ardupilot
            log_data = Ardupilot.parse(filename, types=['PSCN', 'PSCE'])
            
            if hasattr(log_data, 'PSCN') and not log_data.PSCN.empty:
                df_n = log_data.PSCN
                colunas_n = list(df_n.columns)
                
                col_v = next((c for c in ['V', 'VN', 'Vel', 'Velocity'] if c in colunas_n), None)
                col_tv = next((c for c in ['TV', 'TVN', 'TVel', 'DesVel', 'TargetVel'] if c in colunas_n), None)
                
                if col_v and col_tv:
                    t0_n = df_n['TimeUS'].iloc[0] / 1e6
                    for t_us, v, tv in zip(df_n['TimeUS'], df_n[col_v], df_n[col_tv]):
                        t_sec = (t_us / 1e6) - t0_n
                        telemetry_buffer.insert_manual('vn', t_sec, v)
                        telemetry_buffer.insert_manual('dvn', t_sec, tv)
                        
                        telemetry_buffer.insert_manual('err_vn', t_sec, tv - v)
                else:
                    self.log(f"AVISO: Colunas não encontradas no PSCN! O log contém apenas: {colunas_n}")

            if hasattr(log_data, 'PSCE') and not log_data.PSCE.empty:
                df_e = log_data.PSCE
                colunas_e = list(df_e.columns)
                
                col_v = next((c for c in ['V', 'VE', 'Vel', 'Velocity'] if c in colunas_e), None)
                col_tv = next((c for c in ['TV', 'TVE', 'TVel', 'DesVel', 'TargetVel'] if c in colunas_e), None)
                
                if col_v and col_tv:
                    t0_e = df_e['TimeUS'].iloc[0] / 1e6
                    for t_us, v, tv in zip(df_e['TimeUS'], df_e[col_v], df_e[col_tv]):
                        t_sec = (t_us / 1e6) - t0_e
                        telemetry_buffer.insert_manual('ve', t_sec, v)
                        telemetry_buffer.insert_manual('dve', t_sec, tv)
                        
                        telemetry_buffer.insert_manual('err_ve', t_sec, tv - v)

                col_p = next((c for c in ['PN', 'Pos', 'Position'] if c in colunas_n), None)
                col_tp = next((c for c in ['TPN', 'TPos', 'DesPos', 'TargetPos'] if c in colunas_n), None)

                if col_p and col_tp:
                    for t_us, p, tp in zip(df_n['TimeUS'], df_n[col_p], df_n[col_tp]):
                        t_sec = (t_us / 1e6) - t0_n
                        telemetry_buffer.insert_manual('pn', t_sec, p)
                        telemetry_buffer.insert_manual('dpn', t_sec, tp)
                        telemetry_buffer.insert_manual('err_pn', t_sec, tp - p)

                col_p = next((c for c in ['PE', 'Pos', 'Position'] if c in colunas_e), None)
                col_tp = next((c for c in ['TPE', 'TPos', 'DesPos', 'TargetPos'] if c in colunas_e), None)

                if col_p and col_tp:
                    for t_us, p, tp in zip(df_e['TimeUS'], df_e[col_p], df_e[col_tp]):
                        t_sec = (t_us / 1e6) - t0_e
                        telemetry_buffer.insert_manual('pe', t_sec, p)
                        telemetry_buffer.insert_manual('dpe', t_sec, tp)
                        telemetry_buffer.insert_manual('err_pe', t_sec, tp - p)
                else:
                    self.log(f"AVISO: Colunas não encontradas no PSCE! O log contém apenas: {colunas_e}")

            self.log("Leitura de velocidades (PSCN/PSCE) e erros concluída!")
            
        except ImportError:
            self.log("Erro: Biblioteca 'ardupilot_log_reader' não instalada.")
        except Exception as e:
            self.log(f"Erro crítico ao ler PSCN/PSCE: {e}")
            
    # --- Método Auxiliar para Abrir Janela e Forçar Foco ---
    def _abrir_janela_telemetria_forcada(self):
        janela = JanelaTelemetria(self)
        
        # 1. Define como transiente (filha da principal -> sempre acima dela)
        # janela.transient(self)  # <--- LINHA COMENTADA PARA PERMITIR MAXIMIZAR/REDIMENSIONAR
        
        # 2. Tenta levantar imediatamente
        janela.lift()
        
        # 3. Agenda o foco forçado para depois que a janela renderizar (Hack de tempo)
        janela.after(100, lambda: janela.focus_force())
        janela.after(100, lambda: janela.lift()) 
        
        return janela

    def carregar_log_externo(self):
        filename = filedialog.askopenfilename(title="Selecione o Log", 
                                              filetypes=[("Log Files", "*.csv *.bin *.tlog *.log"), ("All Files", "*.*")])
        if not filename:
            return

        try:
            self.viewing_log = True
            telemetry_buffer.clear_buffer()
            
            ext = os.path.splitext(filename)[1].lower()
            
            if ext == '.csv':
                self._parse_csv_log(filename) # Mantém a leitura do CSV que ajustamos antes
            else:
                # Agora processamos TODO o log MAVLink apenas por esta função
                self._parse_mavlink_log(filename)
            
            self._abrir_janela_telemetria_forcada()
            
        except Exception as e:
            self.log(f"Erro ao ler log: {e}")
            import traceback
            traceback.print_exc()
            self.viewing_log = False

    def _parse_csv_log(self, filename):
        count = 0
        t0 = None 
        
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            
            reader.fieldnames = [name.strip() for name in reader.fieldnames]
            
            for row in reader:
                if not row or 'timestamp' not in row:
                    continue
                
                try:
                    abs_t = float(row['timestamp'])
                    if t0 is None:
                        t0 = abs_t 
                    
                    rel_t = abs_t - t0
                    
                    row_type = row.get('type', '').strip().lower()
                    
                    if row_type == 'current':
                        telemetry_buffer.insert_manual('vn', rel_t, float(row.get('vel_x', 0)))
                        telemetry_buffer.insert_manual('ve', rel_t, float(row.get('vel_y', 0)))
                        telemetry_buffer.insert_manual('roll', rel_t, float(row.get('roll_deg', 0)))
                        telemetry_buffer.insert_manual('pitch', rel_t, float(row.get('pitch_deg', 0)))
                        telemetry_buffer.insert_manual('yaw', rel_t, float(row.get('yaw_deg', 0)))
                        
                        telemetry_buffer.insert_manual('pn', rel_t, float(row.get('pos_x', 0)))
                        telemetry_buffer.insert_manual('pe', rel_t, float(row.get('pos_y', 0)))
                        
                    elif row_type == 'target':
                        telemetry_buffer.insert_manual('dvn', rel_t, float(row.get('vel_x', 0)))
                        telemetry_buffer.insert_manual('dve', rel_t, float(row.get('vel_y', 0)))
                        telemetry_buffer.insert_manual('des_roll', rel_t, float(row.get('roll_deg', 0)))
                        telemetry_buffer.insert_manual('des_pitch', rel_t, float(row.get('pitch_deg', 0)))
                        telemetry_buffer.insert_manual('des_yaw', rel_t, float(row.get('yaw_deg', 0)))
                        
                        telemetry_buffer.insert_manual('dpn', rel_t, float(row.get('pos_x', 0)))
                        telemetry_buffer.insert_manual('dpe', rel_t, float(row.get('pos_y', 0)))
                    
                    count += 1
                except ValueError:
                    # Ignora a linha caso algum valor falhe na conversão para float
                    pass 
                    
        self.log(f"CSV de testes carregado: {os.path.basename(filename)} ({count} pontos mapeados).")

    def _parse_mavlink_log(self, filename):
        self.log(f"Processando Log MAVLink: {os.path.basename(filename)}...")
        mlog = mavutil.mavlink_connection(filename)
        
        count = 0
        t_start = None
        
        # Estas são as mensagens exatas que a sua API (MavlinkAPI/DeltaVehicle) utiliza
        msgs_to_process = [
            'ATTITUDE', 'ATTITUDE_TARGET', 'LOCAL_POSITION_NED', 
            'POSITION_TARGET_LOCAL_NED', 'RC_CHANNELS', 'SERVO_OUTPUT_RAW',
            'PSCN', 'PSCE' # <-- ADICIONE AQUI
        ]

        while True:
            msg = mlog.recv_match(type=msgs_to_process, blocking=False)
            if not msg:
                break
            
            # Padronização do tempo: tenta ms de boot (MAVLink padrão) ou timestamp fallback
            if hasattr(msg, 'time_boot_ms'):
                t_sec = msg.time_boot_ms / 1000.0
            else:
                t_sec = getattr(msg, '_timestamp', 0)
                
            if t_start is None and t_sec > 0: 
                t_start = t_sec
                
            rel_time = t_sec - t_start if t_start else t_sec
            mtype = msg.get_type()
            
            # --- ATITUDE ATUAL ---
            if mtype == 'ATTITUDE':
                telemetry_buffer.insert_manual('roll', rel_time, math.degrees(msg.roll))
                telemetry_buffer.insert_manual('pitch', rel_time, math.degrees(msg.pitch))
                telemetry_buffer.insert_manual('yaw', rel_time, math.degrees(msg.yaw))
                
            # --- ATITUDE DESEJADA ---
            elif mtype == 'ATTITUDE_TARGET':
                # MAVLink envia a atitude desejada em Quatérnios (q)
                w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
                
                # Conversão manual Quatérnio -> Euler (padrão ZYX)
                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                roll = math.atan2(sinr_cosp, cosr_cosp)
                
                sinp = 2 * (w * y - z * x)
                pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
                
                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                telemetry_buffer.insert_manual('des_roll', rel_time, math.degrees(roll))
                telemetry_buffer.insert_manual('des_pitch', rel_time, math.degrees(pitch))
                telemetry_buffer.insert_manual('des_yaw', rel_time, math.degrees(yaw))
            
            # --- DATALOGS INTERNOS (DataFlash PIDs) ---
            elif mtype == 'PSCN':
                # Busca 'TVN' (novo firmware) ou 'TV' (antigo). Retorna None se não achar nenhum.
                tv = getattr(msg, 'TVN', getattr(msg, 'TV', None))
                v = getattr(msg, 'VN', getattr(msg, 'V', None))
                
                if tv is not None and v is not None:
                    telemetry_buffer.insert_manual('dvn', rel_time, tv)
                    telemetry_buffer.insert_manual('vn', rel_time, v)
                    telemetry_buffer.insert_manual('err_vn', rel_time, tv - v)
                
            elif mtype == 'PSCE':
                # Busca 'TVE' (novo firmware) ou 'TV' (antigo).
                tv = getattr(msg, 'TVE', getattr(msg, 'TV', None))
                v = getattr(msg, 'VE', getattr(msg, 'V', None))
                
                if tv is not None and v is not None:
                    telemetry_buffer.insert_manual('dve', rel_time, tv)
                    telemetry_buffer.insert_manual('ve', rel_time, v)
                    telemetry_buffer.insert_manual('err_ve', rel_time, tv - v)

            # --- VELOCIDADE E POSIÇÃO ATUAL (VN, VE, PN, PE) ---
            elif mtype == 'LOCAL_POSITION_NED':
                telemetry_buffer.insert_manual('vn', rel_time, msg.vx)
                telemetry_buffer.insert_manual('ve', rel_time, msg.vy)
                telemetry_buffer.insert_manual('pn', rel_time, msg.x) # Posição X Atual
                telemetry_buffer.insert_manual('pe', rel_time, msg.y) # Posição Y Atual

# --- VELOCIDADE E POSIÇÃO DESEJADA (DVN, DVE, DPN, DPE) ---
            elif mtype == 'POSITION_TARGET_LOCAL_NED':
                telemetry_buffer.insert_manual('dvn', rel_time, msg.vx)
                telemetry_buffer.insert_manual('dve', rel_time, msg.vy)
                telemetry_buffer.insert_manual('dpn', rel_time, msg.x) # Posição X Desejada
                telemetry_buffer.insert_manual('dpe', rel_time, msg.y) # Posição Y Desejada
                
            # --- RÁDIO CONTROLE E MOTORES ---
            elif mtype == 'RC_CHANNELS':
                telemetry_buffer.insert_manual('rcin1', rel_time, msg.chan1_raw)
                telemetry_buffer.insert_manual('rcin2', rel_time, msg.chan2_raw)
                telemetry_buffer.insert_manual('rcin3', rel_time, msg.chan3_raw)
                telemetry_buffer.insert_manual('rcin4', rel_time, msg.chan4_raw)

            elif mtype == 'SERVO_OUTPUT_RAW':
                telemetry_buffer.insert_manual('rcout1', rel_time, msg.servo1_raw)
                telemetry_buffer.insert_manual('rcout2', rel_time, msg.servo2_raw)
                telemetry_buffer.insert_manual('rcout3', rel_time, msg.servo3_raw)
                telemetry_buffer.insert_manual('rcout4', rel_time, msg.servo4_raw)
            
            count += 1
            if count % 10000 == 0: 
                self.update()
        
        self.log(f"Log MAVLink carregado! {count} mensagens extraídas e processadas.")

    def voltar_para_live(self):
        if self.viewing_log:
            telemetry_buffer.clear_buffer()
            telemetry_buffer.reset_timer()
            self.viewing_log = False
            self.log("Modo LIVE reiniciado (Buffer limpo).")
        else:
            self.log("Abrindo janela LIVE (Buffer preservado).")

        self._abrir_janela_telemetria_forcada()

    def mostrar_dialogo_saida(self):
        if self.viewing_log:
            self._encerrar_tudo()
            return

        dialog = ctk.CTkToplevel(self)
        dialog.title("Confirmar Saída")
        dialog.geometry("350x180")
        dialog.resizable(False, False)
        dialog.transient(self)
        dialog.grab_set()
        
        dialog.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() // 2) - (350 // 2)
        y = self.winfo_y() + (self.winfo_height() // 2) - (180 // 2)
        dialog.geometry(f"+{x}+{y}")

        f_conteudo = ctk.CTkFrame(dialog, fg_color="transparent")
        f_conteudo.pack(expand=True, fill="both", padx=20, pady=20)

        ctk.CTkLabel(f_conteudo, text="Deseja salvar o log de voo\nantes de encerrar?", 
                     font=("Segoe UI", 16, "bold"), text_color="#ffffff").pack(pady=(10, 20))
        
        f_btns = ctk.CTkFrame(f_conteudo, fg_color="transparent")
        f_btns.pack(fill="x")
        
        btn_nao = ctk.CTkButton(f_btns, text="NÃO SALVAR", fg_color="#bd2828", hover_color="#8a1c1c",
                                width=120, height=35, font=("Segoe UI", 12, "bold"),
                                command=self._encerrar_tudo)
        btn_nao.pack(side="left", padx=(0, 10), expand=True)

        btn_sim = ctk.CTkButton(f_btns, text="SIM, SALVAR", fg_color="#009933", hover_color="#006622",
                                width=120, height=35, font=("Segoe UI", 12, "bold"),
                                command=lambda: [dialog.destroy(), self.abrir_janela_comentario()])
        btn_sim.pack(side="right", padx=(10, 0), expand=True)

    def abrir_janela_comentario(self):
        dialogo = ctk.CTkToplevel(self)
        dialogo.title("Comentário do Log")
        dialogo.geometry("400x320")
        dialogo.transient(self)
        dialogo.grab_set()
        
        dialogo.update_idletasks()
        x = self.winfo_x() + (self.winfo_width() // 2) - (200)
        y = self.winfo_y() + (self.winfo_height() // 2) - (160)
        dialogo.geometry(f"+{x}+{y}")
        
        ctk.CTkLabel(dialogo, text="Notas do Voo (Opcional)", font=("Segoe UI", 16, "bold")).pack(pady=(20, 10))
        
        txt_comentario = ctk.CTkTextbox(dialogo, height=150, corner_radius=10, border_width=1, border_color="#444")
        txt_comentario.pack(padx=20, pady=5, fill="both", expand=True)
        txt_comentario.focus()
        
        def confirmar_salvamento():
            comentario = txt_comentario.get("1.0", "end").strip()
            self.salvar_log_em_disco(comentario)
            dialogo.destroy()
            self._encerrar_tudo()
            
        ctk.CTkButton(dialogo, text="SALVAR E FECHAR", fg_color="#009933", hover_color="#006622", height=40,
                      font=("Segoe UI", 13, "bold"), command=confirmar_salvamento).pack(pady=20, padx=20, fill="x")

    def salvar_log_em_disco(self, comentario):
        if not os.path.exists("logs"):
            try: os.makedirs("logs")
            except: pass

        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"logs/log_{timestamp_str}.csv"
        
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                f.write(f"# TuningTool Log\n")
                f.write(f"# Data: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}\n")
                if self.log_history:
                    f.write("# --- System Log (Console Output) ---\n")
                    for line in self.log_history:
                        clean_line = line.replace('\n', ' ')
                        f.write(f"# {clean_line}\n")
                    f.write("# -----------------------------------\n")
                
                if comentario:
                    f.write("# Notas do Usuário:\n")
                    for linha in comentario.split('\n'):
                        f.write(f"#   {linha}\n")
                f.write("# \n")
                f.write("Timestamp,Key,Value\n")
                
                writer = csv.writer(f)
                dados_combinados = []
                if hasattr(telemetry_buffer, 'stored_values') and hasattr(telemetry_buffer, 'timestamps'):
                    vals = telemetry_buffer.stored_values
                    times = telemetry_buffer.timestamps
                    for key, values_list in vals.items():
                        time_list = times.get(key, [])
                        limit = min(len(values_list), len(time_list))
                        for i in range(limit):
                            dados_combinados.append((time_list[i], key, values_list[i]))
                
                dados_combinados.sort(key=lambda x: x[0])
                for t, k, v in dados_combinados:
                    writer.writerow([f"{t:.6f}", k, v])
            
            print(f"Log salvo: {os.path.abspath(filename)}")
        except Exception as e:
            print(f"Erro ao salvar: {e}")

    def encerrar_emergencia(self):
        print("\n!!! EMERGÊNCIA ACIONADA !!! TENTANDO POUSAR O DRONE...")
        if self.drone:
            self.drone.pousar()
            time.sleep(1) 
        self._encerrar_tudo()

    def _encerrar_tudo(self):
        if self.drone: self.drone.fechar()
        self.destroy()
        sys.exit()

    def _conectar_e_inicializar(self):
        telemetry_buffer.reset_timer()
        sucesso = self.drone.conectar()
        if sucesso:
            self.log("Conexão estabelecida. Lendo parâmetros iniciais...")
            self.ler_pids_drone() 
        else:
            self.log("Sem conexão MAVLink (Modo Offline?).")

    def log(self, msg):
        if "SETPOINT INIT" in msg:
            self.t_inicio_calculo = time.time() - telemetry_buffer.init_stamp
        elif "SETPOINT END" in msg and self.t_inicio_calculo is not None:
            t_fim = time.time() - telemetry_buffer.init_stamp
            self.after(100, self._processar_score_performance, self.t_inicio_calculo, t_fim)
            self.t_inicio_calculo = None
        self.after(0, lambda: self._log_safe(msg))

    def _log_safe(self, msg):
        try:
            elapsed = time.time() - telemetry_buffer.init_stamp
            t_str = f"{elapsed:.3f}s"
        except: t_str = "0.000s"
        full_msg = f"[{t_str}] {msg}"
        self.log_history.append(full_msg)
        try:
            self.log_textbox.configure(state="normal")
            self.log_textbox.insert("end", f"{full_msg}\n")
            self.log_textbox.see("end")
            self.log_textbox.configure(state="disabled")
        except: pass

    def _setup_left_panel(self):
        self.frame_left = ctk.CTkFrame(self, fg_color="transparent")
        self.frame_left.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self._criar_secao_voo()
        self._criar_secao_pid()
        self._criar_secao_comandos()

    def _criar_secao_voo(self):
        frame = ctk.CTkFrame(self.frame_left)
        frame.pack(fill="x", pady=(0, 10))
        ctk.CTkLabel(frame, text="FLIGHT CONTROL", font=("Segoe UI", 12, "bold"), text_color="#aaaaaa").pack(pady=5)
        
        grid_voo = ctk.CTkFrame(frame, fg_color="transparent")
        grid_voo.pack(fill="x", padx=5, pady=5)
        
        self.btn_arm = ctk.CTkButton(grid_voo, text="ARMAR", font=("Segoe UI", 14, "bold"), 
                                     height=40, command=self.drone.armar)
        self.btn_arm.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        btn_land = ctk.CTkButton(grid_voo, text="LAND", font=("Segoe UI", 14, "bold"), 
                                 fg_color="#e65100", hover_color="#bf360c", height=40,
                                 command=self.drone.pousar)
        btn_land.pack(side="left", fill="x", expand=True, padx=(5, 0))

        f_status = ctk.CTkFrame(frame, fg_color="transparent")
        f_status.pack(fill="x", padx=5, pady=5)
        self.lbl_status = ctk.CTkLabel(f_status, text="DISCONNECTED", font=("Segoe UI", 12, "bold"), text_color="#ff3333")
        self.lbl_status.pack(side="left", padx=5)
        ctk.CTkButton(f_status, text="LOAD LOG", fg_color="#444", hover_color="#666", 
                      width=80, command=self.carregar_log_externo).pack(side="right", padx=(2, 0))
        ctk.CTkButton(f_status, text="LIVE", fg_color="#6600cc", hover_color="#4d0099", 
                      width=60, command=self.voltar_para_live).pack(side="right", padx=(0, 2))

    def _criar_secao_pid(self):
        frame = ctk.CTkFrame(self.frame_left)
        frame.pack(fill="x", pady=10)
        ctk.CTkLabel(frame, text="FINE TUNING (PID)", font=("Segoe UI", 12, "bold"), text_color="#aaaaaa").pack(pady=5)
        self.axis_selector = ctk.CTkSegmentedButton(frame, values=["Pitch", "Roll", "Yaw"], 
                                                    font=("Segoe UI", 14, "bold"), command=self.mudar_eixo)
        self.axis_selector.set("Pitch")
        self.axis_selector.pack(pady=(5, 10), padx=20, fill="x")
        self.f_pids = ctk.CTkFrame(frame, fg_color="transparent")
        self.f_pids.pack(padx=10, pady=5)
        self.pid_widgets = {}
        labels = [("Proportional (P)", "P"), ("Integral (I)", "I"), ("Derivative (D)", "D"), ("Integrator Max", "IMAX")]
        for i, (txt, key) in enumerate(labels):
            ctk.CTkLabel(self.f_pids, text=txt, font=("Segoe UI", 13)).grid(row=i, column=0, pady=6, sticky="e", padx=(0, 10))
            lbl_val = ctk.CTkLabel(self.f_pids, text="0.0000", fg_color="#1f1f1f", corner_radius=6, 
                                   width=150, height=28, font=("Consolas", 14), anchor="center")
            lbl_val.grid(row=i, column=1, padx=0)
            self.pid_widgets[key] = lbl_val
            f_btns = ctk.CTkFrame(self.f_pids, fg_color="transparent")
            f_btns.grid(row=i, column=2, sticky="w", padx=(10, 0))
            ctk.CTkButton(f_btns, text="-", width=35, height=28, fg_color="#444", hover_color="#666",
                          command=lambda k=key: self.alterar_pid(k, -1)).pack(side="left", padx=2)
            ctk.CTkButton(f_btns, text="+", width=35, height=28, fg_color="#444", hover_color="#666",
                          command=lambda k=key: self.alterar_pid(k, 1)).pack(side="left", padx=2)
        ctk.CTkFrame(frame, height=2, fg_color="#333").pack(fill="x", padx=10, pady=10)
        f_aux = ctk.CTkFrame(frame, fg_color="transparent")
        f_aux.pack(pady=(0, 10))
        self.switch_link = ctk.CTkSwitch(f_aux, text="Link Roll/Pitch", font=("Segoe UI", 12),
                                         command=self._atualizar_visual_link)
        self.switch_link.select()
        self.switch_link.pack(side="left", padx=15)
        f_mult = ctk.CTkFrame(f_aux, fg_color="transparent")
        f_mult.pack(side="left", padx=15)
        ctk.CTkLabel(f_mult, text="Multiplier:").pack(side="left", padx=5)
        self.entry_mult = ctk.CTkEntry(f_mult, width=150, justify="center")
        self.entry_mult.insert(0, "0.01")
        self.entry_mult.pack(side="left", padx=5)
        ctk.CTkButton(f_mult, text="OK", width=30, fg_color="#333", command=self.atualizar_multiplier).pack(side="left")

    def _criar_secao_comandos(self):
        container = ctk.CTkFrame(self.frame_left, fg_color="transparent")
        container.pack(fill="both", expand=True, pady=10)
        f_params = ctk.CTkFrame(container)
        f_params.pack(fill="x", pady=(0, 10))
        ctk.CTkLabel(f_params, text="PARAMS MANAGER", font=("Segoe UI", 11, "bold"), text_color="#aaaaaa").pack(pady=2)
        grid_params = ctk.CTkFrame(f_params, fg_color="transparent")
        grid_params.pack(fill="x", padx=5, pady=5)
        ctk.CTkButton(grid_params, text="READ PARAMS", fg_color="#444", hover_color="#555", 
                      height=40, command=self.ler_pids_drone).pack(side="left", fill="x", expand=True, padx=(0,5))
        ctk.CTkButton(grid_params, text="SEND PARAMS", fg_color="#009933", hover_color="#006622", 
                      height=40, font=("Segoe UI", 13, "bold"), 
                      command=self.enviar_pids_drone).pack(side="left", fill="x", expand=True, padx=(5,0))
        f_tests = ctk.CTkFrame(container, fg_color="#2b2b2b", border_width=1, border_color="#444")
        f_tests.pack(fill="x", pady=0)
        ctk.CTkLabel(f_tests, text="MAVLINK TEST AUTOMATION", font=("Segoe UI", 11, "bold"), text_color="#aaaaaa").pack(pady=5)
        self.lbl_teste_atual = ctk.CTkLabel(f_tests, text="No Test", text_color="#666", font=("Segoe UI", 12))
        self.lbl_teste_atual.pack(pady=2)
        grid_tests = ctk.CTkFrame(f_tests, fg_color="transparent")
        grid_tests.pack(fill="x", padx=5, pady=5)
        ctk.CTkButton(grid_tests, text="CATALOG", fg_color="#555", width=80,
                      command=self.abrir_testes).pack(side="left", padx=(0,5))
        self.btn_executar_teste = ctk.CTkButton(grid_tests, text="START TEST", fg_color="#e65100", hover_color="#bf360c",
                                                state="disabled", command=self.iniciar_execucao_teste)
        self.btn_executar_teste.pack(side="left", fill="x", expand=True)

    def _setup_right_panel(self):
        frame_log = ctk.CTkFrame(self, fg_color="#1a1a1a")
        frame_log.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        frame_log.grid_rowconfigure(1, weight=1)
        frame_log.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(frame_log, text="SYSTEM LOG", font=("Segoe UI", 12, "bold"), text_color="#666").grid(row=0, column=0, pady=5)
        self.log_textbox = ctk.CTkTextbox(frame_log, font=("Consolas", 11), fg_color="#000000", text_color="#00ff00")
        self.log_textbox.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.log_textbox.configure(state="disabled")

    def _atualizar_visual_link(self):
        if self.switch_link.get():
            cor_alerta = "#e65100"
            cor_hover = "#ef6c00"
            self.axis_selector.configure(selected_color=cor_alerta, selected_hover_color=cor_hover)
            self.switch_link.configure(button_color=cor_alerta, progress_color=cor_alerta, text_color="#ff9800")
        else:
            cor_padrao = ["#3B8ED0", "#1F6AA5"]
            cor_hover_padrao = ["#36719F", "#144870"]
            self.axis_selector.configure(selected_color=cor_padrao, selected_hover_color=cor_hover_padrao)
            self.switch_link.configure(button_color=cor_padrao, progress_color=cor_padrao, text_color=["#000", "#fff"])

    def validar_multiplier(self, valor):
        if valor == "": return True
        try:
            Decimal(valor)
            return True
        except: return False

    def atualizar_multiplier(self):
        try:
            self.multiplier = Decimal(self.entry_mult.get())
            self.entry_mult.configure(fg_color="#113311") 
            self.after(200, lambda: self.entry_mult.configure(fg_color=["#F9F9FA", "#343638"]))
        except:
            self.entry_mult.delete(0, "end")
            self.entry_mult.insert(0, str(self.multiplier))

    def update_loop(self):
        if not self.viewing_log:
            telemetry_buffer.process_queue()
        
        if self.drone.connected:
            estado = self.drone.vehicle.armed
            txt = "DISARM" if estado else "ARM"
            cor = "#bd2828" if estado else "#1f6aa5"
            if self.btn_arm.cget("text") != txt:
                self.btn_arm.configure(text=txt, fg_color=cor, hover_color=cor)
            self.lbl_status.configure(text="CONNECTED", text_color="#00ff00")
        else:
            status_text = "LOG VIEW" if self.viewing_log else "DISCONNECTED"
            color_text = "#FFFF00" if self.viewing_log else "#ff3333"
            self.lbl_status.configure(text=status_text, text_color=color_text)
        self.after(100, self.update_loop)

    def mudar_eixo(self, valor):
        self.axis_mode = valor
        self.atualizar_display_pid()

    def alterar_pid(self, param, direction):
        step = Decimal(direction) * self.multiplier
        targets = [self.axis_mode]
        if self.switch_link.get() == 1:
            if self.axis_mode == "Roll": targets.append("Pitch")
            elif self.axis_mode == "Pitch": targets.append("Roll")
        for axis in targets:
            atual = self.pid_vals[axis][param]
            self.pid_vals[axis][param] = atual + step
        self.atualizar_display_pid()

    def atualizar_display_pid(self):
        dados = self.pid_vals[self.axis_mode]
        for key, widget in self.pid_widgets.items():
            val = dados[key]
            widget.configure(text=f"{val:.6f}".rstrip('0').rstrip('.'))

    def ler_pids_drone(self):
        raw = self.drone.ler_pids()
        if raw:
            for axis, prefix in [('Roll', 'ATC_RAT_RLL'), ('Pitch', 'ATC_RAT_PIT'), ('Yaw', 'ATC_RAT_YAW')]:
                self.pid_vals[axis]['P'] = raw.get(f'{prefix}_P', 0)
                self.pid_vals[axis]['I'] = raw.get(f'{prefix}_I', 0)
                self.pid_vals[axis]['D'] = raw.get(f'{prefix}_D', 0)
                self.pid_vals[axis]['IMAX'] = raw.get(f'{prefix}_IMAX', 0)
            self.atualizar_display_pid()
            self.log("PIDs lidos do Drone.")

    def enviar_pids_drone(self):
        to_send = {}
        for axis, prefix in [('Roll', 'ATC_RAT_RLL'), ('Pitch', 'ATC_RAT_PIT'), ('Yaw', 'ATC_RAT_YAW')]:
            to_send[f'{prefix}_P'] = self.pid_vals[axis]['P']
            to_send[f'{prefix}_I'] = self.pid_vals[axis]['I']
            to_send[f'{prefix}_D'] = self.pid_vals[axis]['D']
            to_send[f'{prefix}_IMAX'] = self.pid_vals[axis]['IMAX']
        self.drone.enviar_pids(to_send)
        self.log(">>> PIDs ENVIADOS AO DRONE <<<")

    def abrir_graficos(self):
        self._abrir_janela_telemetria_forcada()

    def abrir_testes(self):
        JanelaCatalogo(self, self.drone, self.log)

    def definir_teste_selecionado(self, path):
        self.teste_selecionado = path
        nome = os.path.basename(path)
        self.lbl_teste_atual.configure(text=f"Ready: {nome}", text_color="#00ff00")
        self.btn_executar_teste.configure(state="normal", fg_color="#e65100")
        self.log(f"Load: {nome}")

    def iniciar_execucao_teste(self):
        if not self.teste_selecionado: return
        if not self.drone.connected:
            self.log("ERRO: Drone não conectado.")
            return
        self.btn_executar_teste.configure(state="disabled", fg_color="#555")
        t = threading.Thread(target=self._run_script_thread)
        t.start()

    def _processar_score_performance(self, t_start, t_end):
        eixo = self.axis_mode
        mapa_chaves = {
            "Pitch": "err_pitch", 
            "Roll":  "err_roll", 
            "Yaw":   "err_yaw",
        }
        chave_erro = mapa_chaves.get(eixo, "err_pitch")
        tempos, valores = telemetry_buffer.get_values(chave_erro)
        soma_modulos = 0.0
        contagem = 0
        for t, val in zip(tempos, valores):
            if t_start <= t <= t_end:
                soma_modulos += abs(val) 
                contagem += 1
        score = 0.0
        media_erro = 0.0
        if contagem > 0:
            media_erro = soma_modulos / contagem
            score = (media_erro / 10.0) * 100.0
            if score > 100: score = 100.0
        pids = self.pid_vals[eixo]
        str_pids = f"P:{pids['P']}  I:{pids['I']}  D:{pids['D']}  IMAX:{pids['IMAX']}"
        self.log("="*40)
        self.log(f"--- RELATÓRIO DE PERFORMANCE ({eixo}) ---")
        self.log(f"Duração: {t_end - t_start:.2f}s | Amostras: {contagem}")
        self.log(f"Somatório Erro (Abs): {soma_modulos:.4f}")
        self.log(f"Erro Médio: {media_erro:.4f}°")
        self.log(f">>> NOTA DE ERRO (0-100): {score:.1f} <<<")
        self.log(f"Configuração: {str_pids}")
        self.log("="*40)

    def _run_script_thread(self):
        nome = os.path.basename(self.teste_selecionado)
        self.log(f"--- INICIANDO TESTE: {nome} ---")
        test_dir = os.path.dirname(self.teste_selecionado)
        path_adicionado = False
        if test_dir not in sys.path:
            sys.path.append(test_dir)
            path_adicionado = True
        stdout_original = sys.stdout
        sys.stdout = StdoutRedirector(self.log)
        try:
            with open(self.teste_selecionado, "r", encoding="utf-8") as f:
                codigo = f.read()
            contexto = {
                "vehicle": self.drone.vehicle,
                "log": self.log,
                "time": time,
                "math": math,
                "mavutil": mavutil
            }
            exec(codigo, contexto)
            self.log(f"--- TESTE CONCLUÍDO: {nome} ---")
        except Exception as e:
            self.log(f"ERRO DE EXECUÇÃO: {e}")
        finally:
            sys.stdout = stdout_original
            if path_adicionado:
                sys.path.remove(test_dir)