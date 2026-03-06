#!/usr/bin/env python3
import os
import pygame
import socket

def main():
    # Allow joystick events in background
    os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
    
    ESP32_IP = "10.224.10.188"
    ESP32_PORT = 4210
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Port reuse
    sock.settimeout(2.0)  # Timeout
    
    print(f"UDP hazır: {ESP32_IP}:{ESP32_PORT}")
    # --- Pygame ve Joystick Başlat ---
    pygame.init()
    pygame.joystick.init()
    
    clock = pygame.time.Clock()
    
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected")
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to controller: {joystick.get_name()}")
    
    last_send_time = 0
    interval = 50
    running = True
    
    while running:
        pygame.event.pump()
    
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
    
        current_time = pygame.time.get_ticks()
        
        if current_time - last_send_time > interval:
            last_send_time = current_time
    
            axis_Forward = joystick.get_axis(5)   # Sol İleri
            axis_Back = joystick.get_axis(2) # 
            axis_turn_360 = joystick.get_axis(4)   # 
            axis_slide_L_R = joystick.get_axis(3) # 
            button_LID = joystick.get_button(0)  # Button 0 - Servo toggle
            button_elevator_up = joystick.get_button(11)  # Button 11 - Elevator up
            button_elevator_down = joystick.get_button(12)  # Button 12 - Elevator down
       
    
            msgs = [
                f"AXIS 5 {axis_Forward:.3f}",
                f"AXIS 2 {axis_Back:.3f}",
                f"AXIS 4 {axis_turn_360:.3f}",
                f"AXIS 3 {axis_slide_L_R:.3f}",
                f"BUTTON 0 {1 if button_LID else 0}",
                f"BUTTON 11 {1 if button_elevator_up else 0}",
                f"BUTTON 12 {1 if button_elevator_down else 0}"
            ]
            
            for i in range(joystick.get_numaxes()):
                val = joystick.get_axis(i)
                print(f"Axis {i}: {val:.3f}")
    
            for i in range(joystick.get_numbuttons()):
                state = joystick.get_button(i)
                print(f"Button {i}:{state}")
    
            for msg in msgs:
                sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    
        clock.tick(60)

if __name__ == "__main__":
    main()
    pygame.quit()
    sock.close()
