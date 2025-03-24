import cv2
import time

# Abrir a câmera
cap = cv2.VideoCapture(0)  # Usar 0 para a câmera padrão

# Verificar se a câmera foi aberta corretamente
if not cap.isOpened():
    print("Erro: Não foi possível abrir a câmera.")
    exit()

# Definir as dimensões (ajuste para resoluções suportadas pela câmera)

# Dar um tempo para a câmera inicializar
time.sleep(2)

# Capturar um frame
ret, frame = cap.read()

# Verificar se o frame foi capturado corretamente
if not ret:
    print("Erro: Não foi possível capturar o frame.")
else:
    # Salvar o frame em um arquivo
    cv2.imwrite('image.jpg', frame)
    print("Imagem salva como 'image.jpg'.")

# Liberar a câmera
cap.release()