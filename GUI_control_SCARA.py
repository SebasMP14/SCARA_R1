# -*- coding: utf-8 -*-
"""
Robot SCARA Arduino UNO - FIUNA - Robotica I
@author: Sebas Monje - Magno Caballero - Brenda Cruz
    El programa presenta una interfaz grafica donde se pueden asignar los 
puntos inicial y final del movimiento además del grado cierre del gripper.
Al presionar "Enviar" se determina si los puntos se encuentran en el area de 
trabajo, se calcula la trayectoria y a esta se le aplica la cinemática inversa
del robot SCARA. Además presenta en el navegador una previsualización de la 
trayectoria a realizar.
"Parada" detiene el movimiento, impidiendo que se vuelva a desarrollar la 
trayectoria cargada anteriormente, por lo que se debe presionar "Reestablecer",
que llevará el brazo a la posición de origen y preparará el Arduino para recibir 
una nueva trayectoria.
"""
import serial
import time
import numpy as np
### estos tres para el grafico de la trayectoria
import plotly.graph_objects as go
import plotly.io as pio
pio.renderers.default = 'browser'

from tkinter import Tk, Label, Button, Entry
import tkinter as tk
from PIL import Image, ImageTk


xp = 0
yp = 0
theta1 = 0
theta2 = 0
phi = 0
L1 = 228
L2 = 136.5
data = ""

def forwardKinematics(theta1, theta2):
    global xp, yp
    theta1F = theta1 * np.pi / 180  # grados a radianes
    theta2F = theta2 * np.pi / 180
    xp = np.round(L1 * np.cos(theta1F) + L2 * np.cos(theta1F + theta2F))
    yp = np.round(L1 * np.sin(theta1F) + L2 * np.sin(theta1F + theta2F))

def inverseKinematics(x, y):
    global theta1, theta2, phi

    theta2 = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
    if x < 0 and y < 0: # tercer cuadrante
        theta2 = (-1) * theta2
    if x < 0 and y < 0:
        theta1 = np.arctan2(y, x) - np.arctan2((L2 * np.sin(theta2)), (L1 + L2 * np.cos(theta2))) 
    else:
        theta1 = np.arctan2(x, y) - np.arctan2((L2 * np.sin(theta2)), (L1 + L2 * np.cos(theta2)))
    theta2 = (-1) * theta2 * 180 / np.pi
    theta1 = theta1 * 180 / np.pi

    # Ajuste de ángulos dependiendo del cuadrante en el que se encuentra la coordenada final x, y
    if x >= 0 and y >= 0:  # 1er cuadrante
        theta1 = 90 - theta1 
    elif x <= 0 and y > 0:  # 2do cuadrante 
        theta1 = 90 - theta1 
    elif x < 0 and y < 0:  # 3er cuadrante
        theta1 = 360 + theta1 
        theta2 = -theta2 
    elif x > 0 and y < 0:  # 4to cuadrante
        theta1 = 90 - theta1 
    elif x < 0 and y == 0: # eje -x
        theta1 = 90 - theta1 
    
    # Calcular el ángulo "phi" para que el gripper esté paralelo al eje X
    phi = 90 + theta1 + theta2
    phi = (-1) * phi 
    
    # Ajuste de ángulo dependiendo del cuadrante en el que se encuentra la coordenada final x, y
    if x < 0 and y < 0:  # 3er cuadrante
        phi = 90 + theta1 + theta2 
        phi = (-1) * phi
    if np.abs(phi) > 165: # Límites de giro
        phi = 180 + phi
        
    # delimitar para que no se pase de los limites de movimiento, PENDIENTE...
    theta1 = np.round(theta1) # respecto al eje x (positivo en sentido antihorario)
    theta2 = np.round(theta2) # respecto al eje y1 (positivo en sentido antihorario)
    phi = np.round(phi) # respecto al eje y2 (positivo en sentido antihorario)

def TrayRecta(A, B): # Utilizando ec. parametrica de la recta
    if (A[0] > 0 and A[1] >= 0 and B[0] < 0 and B[1] >= 0) or (A[0] < 0 and A[1] >= 0 and B[0] > 0 and B[1] >= 0): # primer y segundo cuadrante
        pasos = 20
        C = np.array([0, 160, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start= int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] >= 0 and B[0] > 0 and B[1] <= 0) or (A[0] > 0 and A[1] <= 0 and B[0] > 0 and B[1] >= 0): # primer y cuarto cuadrante
        pasos = 20
        direccion = B - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, pasos)): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int)
    elif (A[0] > 0 and A[1] < 0 and B[0] < 0 and B[1] < 0) or (A[0] < 0 and A[1] < 0 and B[0] > 0 and B[1] < 0): # tercer a cuarto cuadrante
        pasos = 30
        if A[0] > 0: # si A esta en el cuarto cuadrante
            C1 = np.array([150, 150, (A[2] + B[2])/2])
            direccion = C1 - A
            trayectoria = np.zeros((pasos, 3))
            for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
                punto_intermedio = A + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            C2 = np.array([-150, 150, (A[2] + B[2])/2])
            direccion = C2 - C1
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(pasos/3)):
                punto_intermedio = C1 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            direccion = B - C2
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(2*pasos/3)):
                punto_intermedio = C2 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            return trayectoria.astype(int), pasos
        else: # si A esta en el tercer cuadrante
            C1 = np.array([-150, 150, (A[2] + B[2])/2])
            direccion = C1 - A
            trayectoria = np.zeros((pasos, 3))
            for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
                punto_intermedio = A + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            C2 = np.array([150, 150, (A[2] + B[2])/2])
            direccion = C2 - C1
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(pasos/3)):
                punto_intermedio = C1 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            direccion = B - C2
            for i, t in enumerate(np.linspace(0, 1, int(pasos/3)), start= int(2*pasos/3)):
                punto_intermedio = C2 + t * direccion # ec. parametrica de una recta
                trayectoria[i] = punto_intermedio
            return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] < 0 and B[0] < 0 and B[1] >= 0) or (A[0] < 0 and A[1] >= 0 and B[0] > 0 and B[1] < 0): # segundo y cuarto cuadrante
        pasos = 20
        C = np.array([150, 200, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start = int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    elif (A[0] > 0 and A[1] >= 0 and B[0] < 0 and B[1] < 0) or (A[0] < 0 and A[1] <= 0 and B[0] > 0 and B[1] >= 0): # primer y tercer cuadrante
        pasos = 20
        C = np.array([-150, 200, (A[2] + B[2])/2])
        direccion = C - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2))): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        direccion = B - C
        for i, t in enumerate(np.linspace(0, 1, int(pasos/2)), start= int(pasos/2)):
            punto_intermedio = C + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos
    else: # para los demas casos, ambos en un cuadrante, 2y3 o 1y4
        pasos = 10
        direccion = B - A
        trayectoria = np.zeros((pasos, 3))
        for i, t in enumerate(np.linspace(0, 1, pasos)): # t varia de 0 a 1
            punto_intermedio = A + t * direccion # ec. parametrica de una recta
            trayectoria[i] = punto_intermedio
        return trayectoria.astype(int), pasos     
    
def TrayCia():
    # Centro de la circunferencia
    centro = np.array([0, 200])

    # Radio de la circunferencia
    radio = 50

    # Número de puntos en la trayectoria
    num_puntos = 30

    # Ángulos desde 0 a 2pi con pasos igualmente espaciados
    angulos = np.linspace(0, 2 * np.pi, num_puntos)

    # Coordenadas x e y de la trayectoria circular
    x_tray = centro[0] + radio * np.cos(angulos)
    y_tray = centro[1] + radio * np.sin(angulos)
    z_tray = np.linspace(-50, -100, num_puntos)

    # Crear el array de NumPy con las coordenadas
    trayectoria = np.column_stack((x_tray, y_tray, z_tray))

    return trayectoria.astype(int), num_puntos

def puntos_en_area(A, B, radio1, radio2):
    # Calcular la distancia al cuadrado desde el origen hasta cada punto
    dist1 = A[0]**2 + A[1]**2
    dist2 = B[0]**2 + B[1]**2
    
    # Calcular el cuadrado del radio
    radio1 = radio1**2
    radio2 = radio2**2
    
    # Verificar si los puntos están dentro del area de trabajo
    if dist1 <= radio1 and dist2 <= radio1 and dist1 >= radio2 and dist2 >= radio2 and A[2] > -220 and B[2] > -220 and (A[0] > 160 or A[0] < -140) and (B[0] > 160 or B[0] < -140) and (A[1] > -np.sqrt(radio1)*np.sin(88*np.pi/180) or B[1] > -np.sqrt(radio1)*np.sin(88*np.pi/180)):                              
        return True
    else:
        return False

def appCineI(tray, pasos):
    global theta1, theta2, phi
    Coord_art = np.zeros((pasos, 3))
    for i in range(len(tray)):
        inverseKinematics(tray[i, 0], tray[i, 1])
        Coord_art[i] = np.array([theta1, theta2, phi])
    Coord_art = Coord_art.astype(int)
    return Coord_art

def enviar_Trama(Coord_art, tray, gripper):
    global Ardu, data
    for i in range(len(tray)):
        if i == len(tray) - 1: # el ultimo valor pone en modo RUN con data[1] = 1
            data = f"1,1,{Coord_art[i, 0]},{Coord_art[i, 1]},{Coord_art[i, 2]},{tray[i, 2]},{gripper[i]}\n"
            Ardu.write(data.encode())
        else: # todos los datos enviados se guardan con data[0] = 1
            data = f"1,0,{Coord_art[i, 0]},{Coord_art[i, 1]},{Coord_art[i, 2]},{tray[i, 2]},{gripper[i]}\n"
            Ardu.write(data.encode())
        print(f"Dato enviado {i}: {data.encode()}")
        cadena = Ardu.readline().decode('utf-8').strip() # para esperar a que este listo el arduino
    #Dejamos abierta la comunicacion serial
    

def gripper(angulo, pasos):
    gripper = np.zeros(pasos).astype(int) 
    for i in range(pasos):
        gripper[i] = angulo # cerrar
    gripper[-1] = 100 # abrir cuando finaliza el movimiento
    return gripper

def enviarParametros():
    x1_val = int(x1.get())
    y1_val = int(y1.get())
    z1_val = int(z1.get())
    x2_val = int(x2.get())
    y2_val = int(y2.get())
    z2_val = int(z2.get())
    gripper_val = int(garra.get())

    A = np.array([x1_val, y1_val, z1_val])
    B = np.array([x2_val, y2_val, z2_val])
    if puntos_en_area(A, B, L1 + L2, L1 - L2 * np.cos(15*np.pi/180)): # Radio interior cia de radio 96mm, radio exterior 364mm
        #tray, pasos = TrayRecta(A, B)
        tray, pasos = TrayCia()
        x = tray[:, 0]
        y = tray[:, 1]
        z = tray[:, 2]
        fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z, mode='lines', marker=dict(color='green'))])
        fig.update_layout(scene=dict(xaxis=dict(title='X'),
                                     yaxis=dict(title='Y'),
                                     zaxis=dict(title='Z')),
                          title='Trayectoria en Linea Recta',
                          margin=dict(l=0, r=0, b=0, t=40))
        fig.show()
        Gripper = gripper(gripper_val, pasos)
        Coord_art = appCineI(tray, pasos)
        print(Coord_art)
        enviar_Trama(Coord_art, tray, Gripper)
    else:
        print("Punto/s fuera del area de trabajo.")
    

def STOP():
    global Ardu, data
    data = "Parar"
    Ardu.write(data.encode())

def reestablecer():
    global Ardu, data
    data = "2,0,0,0,0,0,0\n"
    Ardu.write(data.encode())
    print("reestableciendo")
    Ardu.close()
    time.sleep(15)
    Ardu = serial.Serial('COM8', 115200)
    time.sleep(2)


Ardu = serial.Serial('COM8', 115200)
time.sleep(2)

ventana = Tk()
ventana.title("SCARA - Robotica I - 2024")
ventana.geometry("400x300")
ventana.resizable(False, False) # Bloquea el redimensionamiento de la ventana (horizontal, vertical)
ventana.iconbitmap("C:\imagenSCARA\scara.ico") # si se presentan errores, comentar esta linea
ventana.configure(background='#FFFFFF')


## Primer Label (texto)
lbl1 = Label(ventana, text="Posicion Inicial (x,y,z): ", background='#FFFFFF')
lbl1.place(x=10,y=10, width=120,height=30)
## Cuadro para ingreso de texto
x1 = Entry(ventana, bg = "white")
x1.place(x=140,y=10, width=50,height=30)

y1 = Entry(ventana, bg = "white")
y1.place(x=200,y=10, width=50,height=30)

z1 = Entry(ventana, bg = "white")
z1.place(x=260,y=10, width=50,height=30)

## Segundo Label (texto)
lbl2 = Label(ventana, text="Posicion Final (x,y,z): ", background='#FFFFFF')
lbl2.place(x=10, y=50, width=120, height=30)
## Cuadro para ingreso de texto
x2 = Entry(ventana, bg = "white")
x2.place(x=140, y=50, width=50, height=30)

y2 = Entry(ventana, bg = "white")
y2.place(x=200, y=50, width=50, height=30)

z2 = Entry(ventana, bg = "white")
z2.place(x=260, y=50, width=50, height=30)

## tercer Label (texto)
lbl3 = Label(ventana, text="Gripper: ", background='#FFFFFF')
lbl3.place(x=10, y=90, width=100, height=30)
## Cuadro para ingreso de texto
garra = Entry(ventana, bg= "white")
garra.place(x=140, y=90, width=50, height=30)

yy = 140
BotonEnviar = Button(ventana, text= "Enviar", bg = '#9ACD32', command=enviarParametros)
BotonEnviar.place(x=50, y=yy, width=100, height=30)

BotonReestablecer = Button(ventana, text= "Reestablecer", bg = '#FFD700', fg = "black", command=reestablecer)
BotonReestablecer.place(x=160, y=yy, width=100, height=30)

# Crear un botón con la imagen
boton_con_imagen = tk.Button(ventana, text="Parada", bg = '#FF6347', command=STOP)
boton_con_imagen.place(x=270, y=yy, width=100, height=30)


# si se presentan errores, comentar este bloque
ruta_imagen = "C:/imagenSCARA/fiuna_logo.png"
imagen_pil = Image.open(ruta_imagen)
imagen = ImageTk.PhotoImage(imagen_pil)
label_imagen=Label(ventana, image=imagen)
label_imagen.place(x=70, y=190)

ventana.mainloop()

