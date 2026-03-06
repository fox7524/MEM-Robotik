import os
import pygame
import socket

# Allow joystick events in background
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

ESP32_IP = "192.168.1.152"   # ESP32 IP adresini buraya yaz 
ESP32_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

# --- UI Settings ---
screen = pygame.display.set_mode((800, 700)) 
pygame.display.set_caption("Controller Test")
font = pygame.font.SysFont("Consolas", 16)
clock = pygame.time.Clock()

COLUMN_SPACING = 400
LEFT_COLUMN_X = 50
RIGHT_COLUMN_X = LEFT_COLUMN_X + COLUMN_SPACING
Y_START = 40
Y_SPACING = 25

# Sürekli veri gönderimi için zamanlayıcı
last_send_time = 0
SEND_INTERVAL = 50  # Milisaniye (Her 50ms'de bir veri yollar)

running = True

while running:
    # 1. Eventleri Yakala (Pencere donmasın diye)
    pygame.event.pump()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((20,20,24))
    
    current_time = pygame.time.get_ticks()

    # ==========================================
    # 1. GÖRSELLEŞTİRME (Ekranda Barları Çiz)
    # ==========================================
    
    # --- AXES GÖRSELLEŞTİRME ---
    for i in range(joystick.get_numaxes()):
        val = joystick.get_axis(i)
        bar_len = int((val + 1.0) * 100)
        bar_y = Y_START + i * Y_SPACING
        
        # Arkaplan
        pygame.draw.rect(screen, (50,50,50), (LEFT_COLUMN_X, bar_y, 200, 18))
        # Değer Barı
        pygame.draw.rect(screen, (0,180,80), (LEFT_COLUMN_X, bar_y, bar_len, 18))
        # Yazı
        text = font.render(f"Axis {i}: {val:.3f}", True, (230,230,230))
        screen.blit(text, (LEFT_COLUMN_X + 220, bar_y))

    # --- BUTTONS GÖRSELLEŞTİRME ---
    for i in range(joystick.get_numbuttons()):
        state = joystick.get_button(i)
        button_y = Y_START + i * Y_SPACING + 150
        color = (220,60,60) if state else (80,80,90)
        pygame.draw.rect(screen, color, (RIGHT_COLUMN_X, button_y, 100, 18))
        text = font.render(f"Button {i}", True, (230,230,230))
        screen.blit(text, (RIGHT_COLUMN_X + 150, button_y))

    # ==========================================
    # 2. VERİ GÖNDERME (HEARTBEAT)
    # Değişim olsun olmasın, kritik motor verilerini
    # her 50ms'de bir zorla gönderiyoruz.
    # ==========================================
    
    if current_time - last_send_time > SEND_INTERVAL:
        last_send_time = current_time
        
        # Senin kullandığın kritik eksenleri alıyoruz
        # (PS5 kolunda buton 9-10 yerine Axis 10-11 olarak göründüğü için Axis okuyoruz)
        # Eğer senin PC'de bunlar farklı ID ise burayı güncellemelisin.
        
        axis_L_Fwd = joystick.get_axis(5)   # Sol İleri
        axis_L_Back = joystick.get_axis(0) # Sol Geri (Eski buton 10 variable)
        axis_R_Fwd = joystick.get_axis(4)   # Sağ İleri
        axis_R_Back = joystick.get_axis(2) # Sağ Geri (Eski buton 9 variable)
        
        # Paketleri Hazırla
        msgs = [
            f"AXIS 5 {axis_L_Fwd:.3f}",
            f"AXIS 0 {axis_L_Back:.3f}",
            f"AXIS 4 {axis_R_Fwd:.3f}",
            f"AXIS 2 {axis_R_Back:.3f}"
        ]
        
        # Hepsini Arduino'ya fırlat
        for msg in msgs:
            sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()