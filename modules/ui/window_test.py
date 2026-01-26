import customtkinter as ctk
import os

class JanelaCatalogo(ctk.CTkToplevel):
    def __init__(self, parent, drone_manager, log_callback):
        super().__init__(parent)
        self.title("Catálogo de Testes de Voo")
        self.geometry("900x600")
        self.drone_manager = drone_manager
        self.log = log_callback
        
        # Layout
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=2)
        self.grid_rowconfigure(0, weight=1)

        # Lista de Arquivos
        self.frame_lista = ctk.CTkFrame(self)
        self.frame_lista.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        ctk.CTkLabel(self.frame_lista, text="Testes Disponíveis", font=("Segoe UI", 16, "bold")).pack(pady=5)
        self.scroll_files = ctk.CTkScrollableFrame(self.frame_lista)
        self.scroll_files.pack(fill="both", expand=True, padx=5, pady=5)

        # Preview
        self.frame_preview = ctk.CTkFrame(self)
        self.frame_preview.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        ctk.CTkLabel(self.frame_preview, text="Código do Teste", font=("Segoe UI", 16, "bold")).pack(pady=5)
        self.textbox_preview = ctk.CTkTextbox(self.frame_preview, font=("Consolas", 12))
        self.textbox_preview.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Botão modificado para "SELECIONAR"
        self.btn_selecionar = ctk.CTkButton(self.frame_preview, text="SELECIONAR ESTE TESTE", 
                                          fg_color="#1f6aa5", hover_color="#144870", 
                                          state="disabled", command=self.confirmar_selecao)
        self.btn_selecionar.pack(fill="x", pady=10, padx=5)

        self.caminho_selecionado = None
        self.carregar_lista()

    def carregar_lista(self):
        base_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        pasta_tests = os.path.join(base_path, "tests")
        
        if not os.path.exists(pasta_tests):
            ctk.CTkLabel(self.scroll_files, text="Pasta 'tests' não encontrada!").pack()
            return

        arquivos = []
        for root, dirs, files in os.walk(pasta_tests):
            if "controle" in dirs: dirs.remove("controle") 
            for file in files:
                if file.endswith(".py"):
                    full = os.path.join(root, file)
                    arquivos.append(full)

        for path in sorted(arquivos):
            nome = os.path.basename(path)
            btn = ctk.CTkButton(self.scroll_files, text=nome, anchor="w", fg_color="transparent", 
                                border_width=1, border_color="#444444", text_color="#dddddd",
                                command=lambda p=path: self.mostrar_preview(p))
            btn.pack(fill="x", pady=2)

    def mostrar_preview(self, path):
        self.caminho_selecionado = path
        self.textbox_preview.delete("0.0", "end")
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.textbox_preview.insert("0.0", f.read())
            self.btn_selecionar.configure(state="normal", text=f"SELECIONAR: {os.path.basename(path)}")
        except Exception as e:
            self.textbox_preview.insert("0.0", f"Erro: {e}")

    def confirmar_selecao(self):
        if not self.caminho_selecionado: return
        
        # Chama método no pai (MainApp) para registrar a seleção
        self.master.definir_teste_selecionado(self.caminho_selecionado)
        
        # Fecha a janela de catálogo
        self.destroy()