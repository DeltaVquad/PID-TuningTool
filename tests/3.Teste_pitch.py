import funcoes as fn

logan = vehicle

try:
    fn.armar(logan)
    fn.decolar(logan, 2)
    fn.manter_posicao(logan, 3)
    print("SETPOINT INIT")
    fn.mover_frente(logan, 0.5, 1)
    fn.manter_posicao(logan, 2)
    fn.mover_tras(logan, 0.5, 1)
    print("SETPOINT END")
    fn.manter_posicao(logan, 2)

except Exception as e:
    print(f"Ocorreu um erro! {e}")
    fn.pousar(logan)
    fn.desarmar(logan)
    
except KeyboardInterrupt:
    print("\nInterrupção pelo usuário (Ctrl+C).")
    fn.pousar(logan)
    fn.desarmar(logan)

finally:
    fn.pousar(logan)
    fn.desarmar(logan)