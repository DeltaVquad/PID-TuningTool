import time
from queue import Queue

# Filas e armazenamento
data_queue = Queue()
stored_values = {}
timestamps = {}
init_stamp = time.time()

def submit_data(key, value):
    """Chamado pelas threads do DroneManager para guardar dados."""
    if value is not None:
        data_queue.put((key, value))

def process_queue():
    """Chamado periodicamente pela UI para processar a fila."""
    while not data_queue.empty():
        key, val = data_queue.get()
        _store_value(key, val)

def _store_value(key, value):
    if key not in stored_values:
        stored_values[key] = []
        timestamps[key] = []
    
    stored_values[key].append(value)
    timestamps[key].append(time.time() - init_stamp)

def get_values(key):
    """Retorna listas (tempo, valor) para o plot."""
    if key in stored_values:
        return timestamps[key], stored_values[key]
    return [], []

def reset_timer():
    global init_stamp
    init_stamp = time.time()

# --- Novos Métodos para Leitura de Log ---

def clear_buffer():
    """Limpa todos os dados armazenados para carregar um log externo."""
    global stored_values, timestamps, data_queue, init_stamp
    stored_values = {}
    timestamps = {}
    with data_queue.mutex:
        data_queue.queue.clear()
    init_stamp = time.time() # Reseta o tempo base também, embora logs usem timestamp absoluto/relativo do arquivo

def insert_manual(key, timestamp, value):
    """Insere dados diretamente (usado ao ler CSV)."""
    if key not in stored_values:
        stored_values[key] = []
        timestamps[key] = []
    stored_values[key].append(value)
    timestamps[key].append(timestamp)