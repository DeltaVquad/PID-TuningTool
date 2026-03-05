import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.backend_bases import MouseButton
from ardupilot_log_reader import Ardupilot
from .. import telemetry_buffer

class JanelaTelemetria(ctk.CTkToplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("DeltaV - TuningTool - Telemetria")
        self.geometry("1100x700")
        
        # --- Configurações de Estado ---
        self.paused = False
        self.lines = {}
        self.checkboxes = {}
        self.groups_ui = {} # Guarda referencias dos grupos colapsáveis
        self.active_keys = ['roll', 'pitch', 'yaw', 'vn', 've']
        
        # Paleta de Cores Original
        self.palette = ['#00ff00', '#00ccff', '#ff3333', '#ffff00', '#ff00ff', '#ff9900', '#aa00ff', '#ffffff', '#00ff99', '#ff66b2']
        self.assigned_colors = {}
        self.color_index = 0
        
        # Inicializa cores para as chaves padrão
        for key in self.active_keys:
            self.assigned_colors[key] = self.palette[self.color_index % len(self.palette)]
            self.color_index += 1

        self.param_groups = {
            "ATITUDE (Graus)": [
                ("roll", "Roll Atual"), ("des_roll", "Roll Desejado"),
                ("pitch", "Pitch Atual"), ("des_pitch", "Pitch Desejado"),
                ("yaw", "Yaw Atual"), ("des_yaw", "Yaw Desejado")
            ],
            "ERRO ATITUDE (Graus)": [
                ("err_roll", "Erro Roll"),
                ("err_pitch", "Erro Pitch"),
                ("err_yaw", "Erro Yaw")
            ],
            "VELOCIDADE East (m/s)": [
            ("dve", "Vel East Desejada (DVE)"),
            ("ve", "Vel East Real (VE)")
            ],
            "VELOCIDADE North (m/s)": [
            ("dvn", "Vel North Desejada (DVN)"),
            ("vn", "Vel North Real (VN)"),
            ],
            "ERRO VELOCIDADE (m/s)": [
            ("err_vn", "Erro Vel North "),
            ("err_ve", "Erro Vel East")
            ],
            "RC INPUT (RCIN)": [("rcin1", "C1 (Rll)"), ("rcin2", "C2 (Pit)"), ("rcin3", "C3 (Thr)"), ("rcin4", "C4 (Yaw)")],
            "RC OUTPUT (RCOUT)": [("rcout1", "C1"), ("rcout2", "C2"), ("rcout3", "C3"), ("rcout4", "C4")]
        }

        # --- Layout Principal ---
        self.grid_columnconfigure(0, weight=4) # Gráfico
        self.grid_columnconfigure(1, weight=1) # Sidebar
        self.grid_rowconfigure(0, weight=1)

        # 1. Área do Gráfico (Esquerda)
        self.frame_plot = ctk.CTkFrame(self, fg_color="#2b2b2b")
        self.frame_plot.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.fig.patch.set_facecolor('#2b2b2b')
        self.ax = self.fig.add_subplot(111)
        self.configurar_eixos()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_plot)
        self.canvas.draw()
        
        # Toolbar do Matplotlib
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.frame_plot, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(side="top", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Eventos do Mouse (Zoom, Pan, Hover)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.canvas.mpl_connect('motion_notify_event', self.on_hover)

        # Tooltip Flutuante
        self.annotation = self.ax.annotate("", xy=(0,0), xytext=(20,20), textcoords="offset points", 
                                           bbox=dict(boxstyle="round", fc="black", ec="white", alpha=0.8),
                                           arrowprops=dict(arrowstyle="->", color="white"), color="white")
        self.annotation.set_visible(False)

        # 2. Sidebar de Controles (Direita)
        self.frame_sidebar = ctk.CTkFrame(self)
        self.frame_sidebar.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        ctk.CTkLabel(self.frame_sidebar, text="Parâmetros", font=("Segoe UI", 18, "bold")).pack(pady=10)
        
        # Frame Scrollável
        self.scroll_params = ctk.CTkScrollableFrame(self.frame_sidebar, label_text="Selecionar Dados")
        self.scroll_params.pack(fill="both", expand=True, padx=5, pady=5)

        # Cria os grupos colapsáveis e checkboxes
        self.criar_controles()
        self.criar_secao_analise()

        self.btn_pause = ctk.CTkButton(self.frame_sidebar, text="PAUSAR", fg_color="#bd2828", command=self.toggle_pause)
        self.btn_pause.pack(pady=20, padx=10, fill="x")

        # Aplica cores iniciais
        self.recalcular_cores()
        self.atualizar_visibilidade_linhas()
        
        # Inicia loop de plotagem
        self.update_plot_loop()
            
    def criar_secao_analise(self):
        # Frame container
        self.frame_analise = ctk.CTkFrame(self.frame_sidebar, fg_color="#2b2b2b")
        self.frame_analise.pack(fill="x", padx=5, pady=10)

        ctk.CTkLabel(self.frame_analise, text="Análise de Erro (Módulo)", font=("Segoe UI", 14, "bold")).pack(pady=5)

        # Inputs de Tempo
        f_inputs = ctk.CTkFrame(self.frame_analise, fg_color="transparent")
        f_inputs.pack(fill="x", padx=5)

        self.entry_t_start = ctk.CTkEntry(f_inputs, placeholder_text="T. Ini", width=60)
        self.entry_t_start.pack(side="left", padx=2, expand=True)
        
        self.entry_t_end = ctk.CTkEntry(f_inputs, placeholder_text="T. Fim", width=60)
        self.entry_t_end.pack(side="left", padx=2, expand=True)

        # Botão Calcular
        ctk.CTkButton(self.frame_analise, text="CALCULAR TOTAL", fg_color="#0066cc", 
                      command=self.calcular_metricas).pack(fill="x", pady=5, padx=5)

        # Display de Resultados
        self.txt_resultado = ctk.CTkTextbox(self.frame_analise, height=80, font=("Consolas", 11))
        self.txt_resultado.pack(fill="x", padx=5, pady=5)
        self.txt_resultado.insert("0.0", "Defina o tempo e clique em calcular.")
        self.txt_resultado.configure(state="disabled")

    def calcular_metricas(self):
        try:
            t_ini = float(self.entry_t_start.get().replace(',', '.'))
            t_fim = float(self.entry_t_end.get().replace(',', '.'))
        except ValueError:
            self._exibir_resultado("Erro: Tempos inválidos. Use números.")
            return

        if t_ini >= t_fim:
            self._exibir_resultado("Erro: T. Inicial deve ser menor que T. Final.")
            return

        results = []

        # 1. Analisar Erros de Atitude (já calculados no drone_manager)
        keys_atitude = ['err_roll', 'err_pitch', 'err_yaw']
        for key in keys_atitude:
            timestamps, values = telemetry_buffer.get_values(key)
            if not values:
                continue

            total_erro = 0.0      # Erro Simples (Viés)
            total_abs_error = 0.0 # Erro Absoluto (Magnitude)
            count = 0
            
            for t, v in zip(timestamps, values):
                if t_ini <= t <= t_fim:
                    total_erro += v
                    total_abs_error += abs(v)
                    count += 1
            
            if count > 0:
                nome_display = key.replace('err_', '').upper()
                media_erro = total_erro / count
                media_abs = total_abs_error / count
                results.append(f"{nome_display}:\n  Média: {media_erro:+.4f}° | MAE: {media_abs:.4f}°")

        # 2. Analisar Erros de Velocidade (Comparando Target vs Real)
        # Como VN/DVN e VE/DVE são gravados juntos pelo Ardupilot, podemos comparar os índices.
        pares_vel = [('VN (North)', 'dvn', 'vn'), ('VE (East)', 'dve', 've')]
        
        for nome_display, key_desejada, key_real in pares_vel:
            t_des, v_des = telemetry_buffer.get_values(key_desejada)
            t_real, v_real = telemetry_buffer.get_values(key_real)
            
            if not v_des or not v_real:
                continue
                
            total_erro = 0.0
            total_abs_error = 0.0
            count = 0
            
            t_min_len = min(len(t_des), len(t_real))
            for i in range(t_min_len):
                t = t_real[i]
                if t_ini <= t <= t_fim:
                    erro = v_des[i] - v_real[i] # Erro = Desejado - Real
                    total_erro += erro
                    total_abs_error += abs(erro)
                    count += 1
                    
            if count > 0:
                media_erro = total_erro / count
                media_abs = total_abs_error / count
                results.append(f"Erro {nome_display}:\n  Média: {media_erro:+.4f} m/s | Erro médio: {media_abs:.4f} m/s")

        if not results:
            self._exibir_resultado("Nenhum dado encontrado nesse intervalo.")
            return

        texto_final = f"=== INTERVALO: {t_ini}s a {t_fim}s ===\n" + "\n".join(results)
        self._exibir_resultado(texto_final)

    def _exibir_resultado(self, texto):
        self.txt_resultado.configure(state="normal")
        self.txt_resultado.delete("0.0", "end")
        self.txt_resultado.insert("0.0", texto)
        self.txt_resultado.configure(state="disabled")

    def configurar_eixos(self):
        self.ax.set_facecolor('#1e1e1e')
        self.ax.grid(True, color='#444444', linestyle='--')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        for spine in self.ax.spines.values(): spine.set_color('white')
        self.ax.set_xlabel("Tempo (s)", color="white")
        self.ax.set_ylabel("Valor", color="white")

    def criar_controles(self):
        for group_name, items in self.param_groups.items():
            # Container do Grupo
            group_container = ctk.CTkFrame(self.scroll_params, fg_color="transparent")
            group_container.pack(fill="x", pady=2)
            
            # Botão de Header (Colapsável)
            btn_header = ctk.CTkButton(group_container, text=f"▼ {group_name}", font=("Segoe UI", 13, "bold"), 
                                       fg_color="transparent", text_color="#dddddd", hover_color="#3a3a3a", anchor="w",
                                       command=lambda g=group_name: self.toggle_group(g))
            btn_header.pack(fill="x", padx=2)
            
            # Frame com os itens (Filhos)
            frame_children = ctk.CTkFrame(group_container, fg_color="transparent")
            frame_children.pack(fill="x", padx=10, pady=2)
            
            for key, label in items:
                is_checked = (key in self.active_keys)
                var = ctk.BooleanVar(value=is_checked)
                # Checkbox
                cb = ctk.CTkCheckBox(frame_children, text=label, variable=var, text_color="gray",
                                     command=lambda k=key: self.ao_alternar(k), 
                                     height=20, font=("Segoe UI", 12))
                cb.pack(anchor="w", padx=5, pady=2)
                self.checkboxes[key] = cb
                
                # Cria a linha no plot (inicialmente vazia)
                line, = self.ax.plot([], [], label=label, linewidth=1.5)
                line.set_visible(is_checked)
                self.lines[key] = line
            
            self.groups_ui[group_name] = {'btn': btn_header, 'frame': frame_children, 'expanded': True}

    def toggle_group(self, group_name):
        group = self.groups_ui[group_name]
        if group['expanded']:
            group['frame'].pack_forget()
            group['btn'].configure(text=f"▶ {group_name}")
            group['expanded'] = False
        else:
            group['frame'].pack(fill="x", padx=10, pady=2)
            group['btn'].configure(text=f"▼ {group_name}")
            group['expanded'] = True

    def ao_alternar(self, key):
        checado = self.checkboxes[key].get()
        if checado:
            if key not in self.active_keys:
                self.active_keys.append(key)
                if key not in self.assigned_colors:
                    self.assigned_colors[key] = self.palette[self.color_index % len(self.palette)]
                    self.color_index += 1
        else:
            if key in self.active_keys:
                self.active_keys.remove(key)
            self.lines[key].set_data([], [])
            
            if not self.active_keys:
                self.assigned_colors.clear()
                self.color_index = 0

        self.recalcular_cores()
        self.atualizar_visibilidade_linhas()
        self.ax.relim()
        self.ax.autoscale_view()

    def recalcular_cores(self):
        for key, line in self.lines.items():
            if key in self.active_keys:
                cor = self.assigned_colors.get(key, 'white')
                line.set_color(cor)
                self.checkboxes[key].configure(text_color=cor)
            else:
                self.checkboxes[key].configure(text_color="gray")

    def atualizar_visibilidade_linhas(self):
        handles, labels = [], []
        # Atualiza visibilidade baseada no checkbox
        for key, line in self.lines.items():
            line.set_visible(self.checkboxes[key].get())
        
        # Monta legenda apenas com ativos
        for key in self.active_keys:
            line = self.lines[key]
            handles.append(line)
            labels.append(line.get_label())

        if handles:
            self.ax.legend(handles, labels, facecolor='#2b2b2b', edgecolor='white', labelcolor='white', loc='upper left', fontsize='small')
        else:
            try: self.ax.legend_.remove()
            except: pass
        self.canvas.draw_idle()

    def update_plot_loop(self):
        if not self.winfo_exists():
            return
            
        if not self.paused:
            has_data = False
            for key in self.active_keys:
                t, v = telemetry_buffer.get_values(key)
                if t and len(t) > 0:
                    self.lines[key].set_data(t, v)
                    has_data = True
            
            if has_data:
                self.ax.relim()
                self.ax.autoscale_view()
            
            self.canvas.draw_idle()
        
        # O self.after DEVE estar aqui, recuado dentro da função
        self.after(33, self.update_plot_loop)

    # --- Funcionalidades de Zoom/Pan/Hover ---
    def on_scroll(self, event):
        if event.inaxes != self.ax: return
        base_scale = 1.2
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        xdata, ydata = event.xdata, event.ydata
        if xdata is None or ydata is None: return

        if event.button == 'up': scale_factor = 1 / base_scale
        elif event.button == 'down': scale_factor = base_scale
        else: scale_factor = 1

        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])

        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        self.canvas.draw_idle()
        if not self.paused: self.toggle_pause()

    def on_click(self, event):
        if event.button == MouseButton.RIGHT:
            self.ax.autoscale(enable=True, axis='both', tight=None)
            self.canvas.draw_idle()

    def on_hover(self, event):
        if not self.paused or event.inaxes != self.ax: return
        vis = self.annotation.get_visible()
        found = False
        for key in self.active_keys:
            line = self.lines[key]
            cont, ind = line.contains(event)
            if cont:
                x, y = line.get_data()
                idx = ind["ind"][0]
                self.annotation.xy = (x[idx], y[idx])
                self.annotation.set_text(f"{key.upper()}: {y[idx]:.2f}\nT: {x[idx]:.1f}s")
                self.annotation.set_visible(True)
                self.canvas.draw_idle()
                found = True
                break
        if not found and vis:
            self.annotation.set_visible(False)
            self.canvas.draw_idle()

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self.btn_pause.configure(text="CONTINUAR", fg_color="#009933")
            self.toolbar.pan()
        else:
            self.btn_pause.configure(text="PAUSAR", fg_color="#bd2828")
            self.annotation.set_visible(False)