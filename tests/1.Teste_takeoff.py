import funcoes as fn

logan = vehicle

try:
    fn.armar(logan)
    fn.decolar(logan, 1)
    fn.manter_posicao(logan, 3)

except Exception as e:
    print(f"Aconteceu um erro: {e}")
    fn.pousar(logan)
    fn.desarmar(logan)

except KeyboardInterrupt:
    print("\nInterrupção pelo usuário (Ctrl+C).")
    fn.pousar(logan)
    fn.desarmar(logan)

finally:
    fn.pousar(logan)
    fn.desarmar(logan)