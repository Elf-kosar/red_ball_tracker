import cv2
import socket
import pickle
import struct

# Kamerayı başlat
cap = cv2.VideoCapture(0)

# Socket oluştur
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# WSL'nin IP adresini ve portu ayarla
host_ip = '172.28.83.227'  # WSL'nin IP adresi
port = 5000

print(f"Kamera başlatılıyor ve {host_ip}:{port} adresine bağlanılıyor...")
client_socket.connect((host_ip, port))

try:
    while True:
        ret, frame = cap.read()
        
        # Görüntüyü sıkıştır
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        # Görüntü boyutunu gönder
        message = pickle.dumps(buffer)
        size = len(message)
        client_socket.sendall(struct.pack("L", size) + message)
        
        # Görüntüyü göster (isteğe bağlı)
        cv2.imshow('Windows Kamera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Temizlik
    cap.release()
    client_socket.close()
    cv2.destroyAllWindows()