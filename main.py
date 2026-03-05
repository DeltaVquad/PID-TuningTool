import customtkinter as ctk
from modules.ui.app_main import MainApp
import sys

CONNECTION_STRING = 'COM3'
BAUD_RATE = 57600

if __name__ == "__main__":
    app = None
    try:
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        conn_string = sys.argv[1] if len(sys.argv) > 1 else CONNECTION_STRING
        app = MainApp(conn_string, BAUD_RATE) 
        app.mainloop()

    except KeyboardInterrupt:
        print("\nInterrupção pelo usuário (Ctrl+C).")
        if app: app.encerrar_emergencia()
        
    except Exception as e:
        print(f"\nOcorreu um erro crítico! {e}")
        if app: app.encerrar_emergencia()