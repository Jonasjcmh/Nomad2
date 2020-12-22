#!/usr/bin/python

##importando librerias
from __future__ import division
from pylab import *
import matplotlib.pyplot as plt
import sys
import socket
import threading
import serial, time
from PyQt4 import QtCore, QtGui, uic
import cv2
import numpy
global ser,s,conn,BUFFER_SIZE,addr,arduino,imu
import hokuyo_ug01   #Libreria modificada para leer el sensor laser
import xlwt
import math
from scipy.integrate import quad
from socket import SHUT_RDWR
import CHR_6dm  #Libreria creada para manejar el sensor
from scipy.integrate import quad
from matplotlib.legend_handler import HandlerLine2D

##Carga de archivo ui para interfaz
form_class = uic.loadUiType("servidorgui_2.ui")[0]            

##inicializacion de comunicacion con Arduino Due 
arduino = serial.Serial()
arduino.port = "/dev/ttymxc3"
arduino.baudrate =115200
arduino.bytesize =serial.EIGHTBITS #8 bits de datos
arduino.parity = serial.PARITY_NONE #no paridad
arduino.stopbits = serial.STOPBITS_ONE #un bit de parada


##inicializacion de comunicacion con driver de motores
ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate =115200
ser.bytesize =serial.EIGHTBITS #8 bits de datos
ser.parity = serial.PARITY_NONE #no paridad
ser.stopbits = serial.STOPBITS_ONE #un bit de parada
ser.timeout = 0             #lectura sin bloqueo
ser.xonxoff = False     #deshabilitar control de flujo
ser.rtscts = False     
ser.dsrdtr = False       
ser.writeTimeout = 2     #tiempo de espera 2 segundos


##inicializacion de comunicacion con sensor laser
uart_port = '/dev/ttyACM0'
uart_speed = 19200
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = laser_serial
laser = hokuyo_ug01.Hokuyo(port)


##Configuracion Puerto Serial para imu
uart_port2 = '/dev/ttyUSB1'                                                       #Nombre del puerto serial
uart_speed2 = 115200                                                              #Rapidez en [bps]
imu_serial = serial.Serial(port=uart_port2, baudrate=uart_speed2, timeout=0.5)    #Configurando puerto serial
port2 = imu_serial                                                                #variable coincidente con la libreria
imu = CHR_6dm.AHRS(port2)                                                         #creando el object
imu.set_silent()                                                                  #Llamando al metodo de configuracion SET_SILENT_MODE
imu.set_channel()                                                                 #Llamando al metodo de configuracion SET_ACTIVE_CHANNELS
try:
    ser.open()
except Exception, e:
    print "error open serial port: " + str(e)
    exit()
try:
    arduino.open()
except Exception, e:
    print "error open serial port: " + str(e)
    exit()

    

class MyWindowClass(QtGui.QMainWindow, form_class):
##declaracion de metodos de la clase a ejecutarse  
    def __init__(self, parent=None):
        

        global ser,s,conn,BUFFER_SIZE,addr
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.pushButton_iniciar.pressed.connect(self.inicio_hilo_1_2)
        self.pushButton_leer.clicked.connect(self.leer)
        self.pushButton_salir.clicked.connect(self.salir)
        self.pushButton_res_odom.clicked.connect(self.init_odometria)
        self.pushButton_obstaculos.clicked.connect(self.presentar_hokuyo)
        self.pushButton_irregularidad.clicked.connect(self.presentar_irregularidad)
        self.pushButton_protecciones.clicked.connect(self.presentar_estado)
        self.pushButton_protecciones.released.connect(self.apagar_estado)
        self.pushButton_leer_imu.clicked.connect(self.presentar_imu)
        self.pushButton_res_odom_2.clicked.connect(self.inicializacion_imu)
        self.iniciar_seguimiento.clicked.connect(self.inicio_trayectoria)
 ##Lectura de sensor inercial      
    def presentar_imu(self):
        (yaw, pitch, roll) = imu.sensor_data()   #Llamado al metodo de obtencion de los angulos de euler (yaw, pitch, roll)
        self.imu_pitch.setText(str(pitch))
        self.imu_roll.setText(str(roll))
        self.imu_yaw.setText(str(yaw))
        
    def inicializacion_imu(self):
        magneto_ref = imu.set_mag_ref()     #Llamando al metodo de configuracion AUTO_SET_MAG_REF

##Lectura de sensores arduino        
    def lectura_arduino(self):
        inicio=time.time()
        global arduino
        if arduino.isOpen():
                arduino.flushInput() #flush input buffer, discarding all its contents
                arduino.flushOutput()#flush output buffer, aborting current output
                a='em'
                arduino.write(a)
                time.sleep(0.001)  #give the serial port sometime to receive the data
                b=arduino.readline()
                if len(b)>=9:
                    #print b
                    bumper=b[2]
                    u_frontal_s=b[4]
                    u_frontal_d=ord(b[5])
                    u_trasero_s=b[7]
                    u_trasero_d=ord(b[8])
                else: 
                    bumper='S'
                    u_frontal_s='S'
                    u_frontal_d=15
                    u_trasero_s='S'                    
                    u_trasero_d=15
                               

        else:
            print "cannot open serial port "
        final=time.time()
        #print final-inicio
        return bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d
    
    def escritura_arduino(self,posicion):
        inicio=time.time()
        global arduino
        if arduino.isOpen():
                arduino.flushInput() #flush input buffer, discarding all its contents
                arduino.flushOutput()#flush output buffer, aborting current output
                a='s'
                b=posicion
                c=a+b
                arduino.write(c)

        else:
            print "cannot open serial port "
        final=time.time()
        print final-inicio
        return 

    def presentar_irregularidad(self):      #lectura de sensores ultrasonicos arduino
        try:
            bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
        except Exception, e:
            print 'error lectura'
            bumper='s'
            u_frontal_s='s'
            u_frontal_d=15
            u_trasero_s='s'
            u_trasero_d=15
        self.label_irregularidad_frontal.setText(str(u_frontal_d))
        self.label_irregularidad_posterior.setText(str(u_trasero_d))
        
    def presentar_estado(self):                #lectura de bumpers y paro de emergencia
        bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
        if  u_frontal_s=='S' and u_trasero_s=='S' and bumper=='S':
            self.label_proteccion_verde.setVisible(True) 
            self.label_proteccion_roja.setVisible(False) 
        else:
            self.label_proteccion_verde.setVisible(False) 
            self.label_proteccion_roja.setVisible(True)
            
    def apagar_estado(self):                              
        bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
        if  u_frontal_s=='S' and u_trasero_s=='S' and bumper=='S':
            self.label_proteccion_verde.setVisible(False) 
            self.label_proteccion_roja.setVisible(False) 
    
    def presentar_hokuyo(self):                 #lectura de bumpers y paro de emergencia
        
        laser.laser_on()
        distancia,angulo=self.leer_hokuyo_no_error_2(124,643)
        angulo=str(angulo)
        angulo=angulo[0:6]
        laser.laser_off()
        self.label_obstaculo_distancia.setText(str(distancia/10))
        self.label_obstaculo_angulo.setText(str(angulo))
        
##Manejo de hilos        
    def inicio_hilo_1_2(self):
        global t,t2
        self.pushButton_iniciar.setEnabled(False)
        self.pushButton_iniciar.setDisabled(True)
        self.iniciar_seguimiento.setEnabled(False)
        self.iniciar_seguimiento.setDisabled(True)
        threads  =  list()
        t  = threading.Thread(target= self.iniciar_p)
        threads.append(t)
        t.start()
        threads  =  list()
        t2  = threading.Thread(target= self.envio_video)
        threads.append(t2)
        t2.start()
        
##Metodo para trasmision de video
    def envio_video(self):
        TCP_IP = '172.31.38.207'  
        TCP_PORT = 5001
        BUFFER_SIZE =  18
        servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        servidor.bind((TCP_IP, TCP_PORT))
        servidor.listen(1)
        sck, addr = servidor.accept()   # Aceptamos conexiones
        print "Conectado a: {0}:{1}".format(*addr)
        while True:
            bandera = sck.recv(10)
            print bandera
            if bandera=='foto':
                camara=cv2.VideoCapture(7)
                exito, buffer = camara.read()
                encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
                result, imgencode = cv2.imencode('.jpg', buffer, encode_param)
                data = numpy.array(imgencode)
                stringData = data.tostring()
                buffer=stringData
                print "Enviando buffer"
                sck.send(str(len(buffer)))     # Enviamos al servidor la cantidad de bytes del archivo que queremos envia
                recibido = sck.recv(10)        # Esperamos la respuesta del cliente
                if recibido == "OK":
                    for byte in buffer:        # En el caso que la respuesta sea la correcta enviamos el archivo byte por byte y salimos del while
                        sck.send(byte)
            elif bandera=='q':
                sck.shutdown(SHUT_RDWR)
                sck.close()
                servidor.shutdown(SHUT_RDWR)
                servidor.close()
        return
##Metodo de lectura de odometria
    def init_odometria(self):
        self.inicializacion_odometria(0,0,0)
        return



    def decodificar_odometria(self,a):
        x_b=a[2:7]
        y_b=a[8:13]
        t_b=a[14:19]
        i_b=a[20:24]
        d_b=a[25:29]
        x_u=int(ord(x_b[1])+ord(x_b[2])*(2**8)+ord(x_b[3])*(2**16)+ord(x_b[4])*(2**24))
        y_u=int(ord(y_b[1])+ord(y_b[2])*(2**8)+ord(y_b[3])*(2**16)+ord(y_b[4])*(2**24))
        t_u=int(ord(t_b[1])+ord(t_b[2])*(2**8)+ord(t_b[3])*(2**16)+ord(t_b[4])*(2**24))
        i_u=int(ord(i_b[1])+ord(i_b[2])*(2**8)+ord(i_b[3])*(2**16))
        d_u=int(ord(d_b[1])+ord(d_b[2])*(2**8)+ord(d_b[3])*(2**16))
        if x_b[0]=='-':
            x=x_u*(-1)
        elif x_b[0]=='+':
            x=x_u
        if y_b[0]=='-':
            y=y_u*(-1)
        elif y_b[0]=='+':
            y=y_u
        if t_b[0]=='-':
            t=t_u*(-1)
        elif t_b[0]=='+':
            t=t_u
        if d_b[0]=='-':
            d=d_u*(-1)
        elif d_b[0]=='+':
            d=d_u
        if i_b[0]=='-':
            i=i_u*(-1)
        elif i_b[0]=='+':
            i=i_u
        return x,y,t,i,d

    def leer(self):
        global ser,s,conn,BUFFER_SIZE,addr
        while True:
            self.serial2('r')
            time.sleep(0.003)  #Tiempo de espera en la comunicacion serial
            numOfLines = 0
            a=''
            response = ser.readline(30)
            a=response
            print len(a)       #Imprimimos la longitud del valor leido
            numOfLines = numOfLines + 1
            if (numOfLines >= 1):
                try:
                    (x,y,t,i,d)=self.decodificar_odometria(a)
                except:
                    while True:
                        self.serial2('r')
                        time.sleep(0.003)  #tiempo de espera en la lectura
                        response = ser.readline(30)
                        a=response
                        if len(a)==30:
                            (x,y,t,i,d)=self.decodificar_odometria(a)
                            break
                
                x_p=x/10000
                y_p=y/10000
                t_p=t*180/(1000000*math.pi)
                x_s=str(x_p)
                y_s=str(y_p)
                t_s=str(t_p)
                self.odometria_x.setText(x_s)
                self.odometria_y.setText(y_s)
                self.odometria_th.setText(t_s)
                break
        return t
    
    def leer_total(self):
        global ser,s,conn,BUFFER_SIZE,addr
        while True:
            self.serial2('r')
            time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
            numOfLines = 0
            a=''
            response = ser.readline(30)
            a=response
            numOfLines = numOfLines + 1
            if (numOfLines >= 1):
                try:
                    (x,y,t,i,d)=self.decodificar_odometria(a)
                except:
                    print 'Error en la lectura'
                    while True:
                        self.serial2('r')
                        time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
                        response = ser.readline(30)
                        a=response
                        if len(a)==30:
                            (x,y,t,i,d)=self.decodificar_odometria(a)
                            break
                x_s=str(x)
                y_s=str(y)
                t_s=str(t)
                i_s=str(i)
                d_s=str(d)
                salto=chr(13)             
                break
        return x,y,t,i,d

    def leer_r(self):
        global ser,s,conn,BUFFER_SIZE,addr
        while True:
            self.serial2('r')
            time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
            numOfLines = 0
            a=''
            response = ser.readline(30)
            a=response
            numOfLines = numOfLines + 1
            if (numOfLines >= 1):
                try:
                    (x,y,t,i,d)=self.decodificar_odometria(a)
                except:
                    while True:
                        self.serial2('r')
                        time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
                        response = ser.readline(30)
                        a=response
                        if len(a)==30:
                            (x,y,t,i,d)=self.decodificar_odometria(a)
                            break
                x_s=str(x)
                y_s=str(y)
                t_s=str(t)
                d_s=str(d)
                i_s=str(i)
                salto=chr(13)
                print t
                break
        return t

    def leer_tcp(self):
        global ser,s,conn,BUFFER_SIZE,addr
        while True:
            self.serial2('r')
            time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
            numOfLines = 0
            a=''
            response = ser.readline(30)
            a=response
            numOfLines = numOfLines + 1
            if (numOfLines >= 1):
                try:
                    (x,y,t,i,d)=self.decodificar_odometria(a)
                except:
                    while True:
                        self.serial2('r')
                        time.sleep(0.003)  #Tiempo de espera para recepcion en la comunicacion serial
                        response = ser.readline(30)
                        a=response
                        if len(a)==30:
                            (x,y,t,i,d)=self.decodificar_odometria(a)
                            break
                x_s=str(x)
                y_s=str(y)
                t_s=str(t)
                i_s=str(i)
                d_s=str(d)
                salto=chr(13)
                imprimir=x_s+salto+y_s+salto+t_s+salto+i_s+salto+d_s+salto
                break
        return imprimir

    def serial(self,data):
        global ser,s,conn,BUFFER_SIZE,addr
        if ser.isOpen():
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output
                a=data
                alfa_2=int(a)
                alfa_a=int(alfa_2/2**16)
                alfa_b=int((alfa_2-alfa_a*(2**16))/(2**8))
                alfa_c=int(alfa_2-alfa_a*(2**16)-alfa_b*(2**8))
                ser.write(chr(alfa_c))
                ser.write(chr(alfa_b))
                ser.write(chr(alfa_a))
                time.sleep(0.001)  #give the serial port sometime to receive the data
                numOfLines = 0
        else:
            print "cannot open serial port "
        return

    def serial2(self,data):  #para enviar caracteres a propeller

        global ser,s,conn,BUFFER_SIZE,addr
        if ser.isOpen():
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output
                        #and discard all that is in buffer
                a=data
                ser.write(a)
                time.sleep(0.001) #give the serial port sometime to receive the data
        else:
            print "cannot open serial port "
        return
    
    def serial3(self,data):   # senvio 4 bytes para odometria
        global ser,s,conn,BUFFER_SIZE,addr
        if ser.isOpen():
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output
                        #and discard all that is in buffer
                a=data
                alfa_2=int(a)
                alfa_a=int(alfa_2/2**24)
                alfa_b=int((alfa_2-alfa_a*(2**24))/(2**16))
                alfa_c=int((alfa_2-alfa_a*(2**24)-alfa_b*(2**16))/(2**8))
                alfa_d=int(alfa_2-alfa_a*(2**24)-alfa_b*(2**16)-alfa_c*(2**8))
                ser.write(chr(alfa_d))
                ser.write(chr(alfa_c))
                ser.write(chr(alfa_b))
                ser.write(chr(alfa_a))
                time.sleep(0.001)  #give the serial port sometime to receive the data

        else:
            print "cannot open serial port "
        return
## Metodos para comunicacion inalambrca tcp ip      
    def starter(self):
        global ser,s,conn,BUFFER_SIZE,addr
        TCP_IP = '172.31.38.207'
        TCP_PORT = 5005
        BUFFER_SIZE =  18
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        s.setblocking(1)
        s.listen(1)
        conn, addr = s.accept()
        s.setblocking(0)
        conn.settimeout(1)
        
    def salir(self):
        global ser,s,conn,BUFFER_SIZE,addr
        ser.close()
        app.exit()
##Metodo para teleoperacion
    def iniciar_p(self):
        global bandera_apagado
        global ser,s,conn,BUFFER_SIZE,addr,velocidad
        try:
            self.starter()   #rutina de inicializacion
        except Exception, e:
            print 'no se realiza comunicacion'
            self.pushButton_iniciar.setEnabled(True)
            self.pushButton_iniciar.setDisabled(False)
            return
        while True:
            print 'aqui inicio'
            distancia_l=self.leer_hokuyo_no_error(364,404)
            try:
                bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
            except Exception, e:
                print 'error lectura'
                bumper='s'
                u_frontal_s='s'
                u_frontal_d=15
                u_trasero_s='s'
                u_trasero_d=15
                    
            print bumper
            distancia=distancia_l
            print 'la distancia es:'+str(distancia)
            if ((distancia > 800) | (distancia==0)) and bumper=='S' and u_frontal_s=='S' and u_trasero_s=='S':
                try:
                    dataa = conn.recv(BUFFER_SIZE)
                except Exception, e:
                    dataa=''
                if dataa!='':                  #comentado para que siempre este activo                print 'Connection address:', addr
                    print dataa
                    if (dataa != 'q') & (dataa != 'r') & (dataa != 'd') & (dataa != 'f'):
                        if len(dataa)> 15:  #si la longitud es menor a 5 datos es el del arduino servo
                                data=dataa[0]
                                data1=dataa[1:3]
                                data2=dataa[3:9]
                                data3=dataa[9:11]
                                data4=dataa[11:17]
                                data5=dataa[17]
                                if data == 'w':
                                    if data1 == '1f':
                                        seri_m1=chr(99)
                                    elif data1 == '1b':
                                        seri_m1=chr(147)
                                    if data3 == '2f':
                                        seri_m2=chr(108)
                                    elif data3 == '2b':
                                        seri_m2=chr(156)
                                    if data2 !='':
                                        val_m1=int(data2)
                                    else:
                                        val_m1=0
                                    if data4 !='':
                                        val_m2=int(data4)
                                    else:
                                        val_m2=0
                                    if data5 == 'p':
                                        m_velo1=self.rampa(val_m1)
                                        m_velo2=self.rampa(val_m2)
                                    elif data5 == 'n':
                                        m_velo1=self.rampa_negativa(val_m1)
                                        m_velo2=self.rampa_negativa(val_m2)
                                    a=len(m_velo1)
                                    
                                    velocidad_1=int(m_velo1[0])
                                    velocidad_2=int(m_velo2[0])
                                    encabezado=data+seri_m1
                                    self.serial2(encabezado)
                                    self.serial(velocidad_1)
                                    self.serial2(seri_m2)
                                    self.serial(velocidad_2)
                                    print str(velocidad_1)+'y'+str(velocidad_2)
                                    time.sleep(0.15)
                                    for i in range(1,a):
                                        velocidad_1=int(m_velo1[i])
                                        velocidad_2=int(m_velo2[i])
                                        encabezado=data+seri_m1
                                        self.serial2(encabezado)
                                        self.serial(velocidad_1)
                                        self.serial2(seri_m2)
                                        self.serial(velocidad_2)
                                        print str(velocidad_1)+'y'+str(velocidad_2)
                                        time.sleep(0.02)
                        else:
                            datas=dataa[0]
                            data1s=dataa[1]
                            if datas=='z':  #servo motor z
                                self.escritura_arduino(data1s)
                            
                if dataa == 'r':
                    self.serial2(dataa)
                    a=self.leer_tcp()
                    conn.send(a)
                if dataa == 'd':
                    conn.send('d'+str(distancia))
                if dataa == 'f':
                    print 'llego dato foto'
                if dataa == 'q':
                    conn.shutdown(SHUT_RDWR)
                    conn.close()
                    s.shutdown(SHUT_RDWR)
                    s.close()
                    self.pushButton_iniciar.setEnabled(True)
                    self.pushButton_iniciar.setDisabled(False)
                    self.iniciar_seguimiento.setEnabled(True)
                    self.iniciar_seguimiento.setDisabled(False)
                    break
                
            elif (distancia <= 800) and (distancia!=0) and bumper=='S' and u_frontal_s=='S' and u_trasero_s=='S':
                
                print 'inicio rutina yuhuuu'
                self.rutina_evasion()
                    
            elif bumper=='S' and u_frontal_s=='P' and u_trasero_s=='S':
                
                print 'inicio rutina retroseso'

                velocidad=5333
                self.parar()
                self.atras()
                time.sleep(4)
                self.parar()
                self.evasion_giro()
                self.parar()
                   
            elif bumper=='S' and u_frontal_s=='S' and u_trasero_s=='P':
                
                velocidad=5333
                print 'inicio rutina yuhuuu'
                self.parar()
                self.adelante()
                time.sleep(4)
                self.parar()
            elif bumper=='P':
                
                print 'inicio rutina yuhuuu'
                conn.send('evasion')  #le aviso al cliente que pierde el control
                self.parar()
                conn.shutdown(SHUT_RDWR)
                conn.close()
                s.shutdown(SHUT_RDWR)
                s.close()
                break
##Rutinas de evasion asistida
    def rutina_evasion(self):
        conn.send('evasion')  #le aviso al cliente que pierde el control
        self.parar()
        self.parar()
        distan_i=self.leer_hokuyo_no_error(620,660)
        distan_d=self.leer_hokuyo_no_error(108,148)
        ## hay cuatro opciones no bloqueo a los dos lados 2 bloqueo al lado izquierdo 3 bloqueo a lado derecho 4 bloqueo a dos lados
        if (distan_i>=2000) & (distan_d>2000):
            print 'evadiendo izquierda'
            self.evasion_izquierda()

        elif (distan_i>=2000) & (distan_d<2000):
            print 'evadiendo izquierda'
            self.evasion_izquierda()

        elif (distan_d>=2000) & (distan_i<2000):
            print 'evadiendo derecha'
            self.evasion_derecha()

        elif (distan_d<=2000) & (distan_i<2000):
            print 'evadiendo giro'
            self.evasion_giro()


    def evasion_derecha(self):
        global velocidad
        velocidad=5333
        angulo_limite=-1396263
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.giro()
        time.sleep(0.1) ##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        ser.flushOutput()
        x,y,t,d,i=self.leer_odometria_total()
        self.inicializacion_odometria(0,0,0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        ser.flushOutput
        
        angulo=self.leer_odometria_r()
        while angulo>angulo_limite:
            time.sleep(0.005)
            angulo=self.leer_odometria_r()
        self.parar()
        
        ## leo si no hay obstaculos a la izquierda
        distance_=self.leer_laser(640,640)
        distance=distance_[0]
        print distance
        ##leo si no hay obstaculos adelante
        bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
        if u_frontal_s=='S':
            
            self.adelante()
            while distance<1000:
                
                print 'sigue avanzando'  ##cuando ya no hay obstaculos paro y hago ultimo giro
                distance_=self.leer_laser(640,640)
                distance=distance_[0]
                bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
                distancia_a=self.leer_hokuyo_no_error(364,404)
                if distancia_a<500 and distancia_a !=0 :
                    self.parar()
                    self.evasion_giro
                    return
                if u_frontal_s=='P':
                    self.parar()
                    self.atras()
                    time.sleep(2)
                    self.parar()
                    self.evasion_giro
                    return
                
                
            time.sleep(3)  #tiempo que se sigue moviendo hasta que todo el cuerpo este fuera del obstaculo
            self.parar()
            angulo_limite=1274090    ## ultimo giro para retomar orientacion
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.1)
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.5)
            angulo=self.leer_odometria_r()
            print 'angulo='+str(angulo)
            time.sleep(0.5)
            self.giro_ah()
            time.sleep(0.1)  #tiempo por ruido
            while angulo<angulo_limite:
                time.sleep(0.01)
                angulo=self.leer_odometria_r()
            self.parar()
            conn.send('finevasion derecha')
        else:
            self.parar()
            self.atras()
            time.sleep(2)
            self.parar()
            self.evasion_giro
            self.parar()
                    
        return

    def evasion_izquierda(self):
        global velocidad
        velocidad=5333
        angulo_limite=1274090
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.giro_ah()
        time.sleep(0.1) ##evitar problemas en serial
        angulo=self.leer_odometria_r()
        while angulo<angulo_limite:
            time.sleep(0.005)
            angulo=self.leer_odometria_r()
        self.parar()
        ## leo si no hay obstaculos a la derecha
        distance_=self.leer_laser(128,128)
        distance=distance_[0]
        print distance
        bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
        if u_frontal_s=='S':
            self.adelante()
            while distance<1000:
                print 'sigue avanzando'  ##cuando ya no hay obstaculos paro y hago ultimo giro
                distance_=self.leer_laser(128,128)
                distance=distance_[0]
                bumper, u_frontal_s, u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
                distancia_a=self.leer_hokuyo_no_error(364,404)
                if distancia_a<500 and distancia_a !=0 :
                    self.parar()
                    self.evasion_giro()
                    self.parar()
                    return
                if u_frontal_s=='P':
                    self.parar()
                    self.atras()
                    time.sleep(2)
                    self.parar()
                    self.evasion_giro()
                    self.parar()
                    return
                
            time.sleep(3)  #tiempo que se sigue moviendo hasta que todo el cuerpo este fuera del obstaculo
            self.parar()
            angulo_limite=-1396263    ## ultimo giro para retomar orientacion
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.1)
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.1)
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.1)
            self.inicializacion_odometria(0,0,0)
            time.sleep(0.1)
            angulo=self.leer_odometria_r()
            print 'angulo='+str(angulo)
            time.sleep(1)
            self.giro()
            time.sleep(0.1)  #tiempo por ruido
            while angulo_limite<angulo:
                time.sleep(0.01)
                angulo=self.leer_odometria_r()
            self.parar()
            conn.send('finevasion izquierda')
        
        else:
            self.parar()
            self.atras()
            time.sleep(2)
            self.parar()
            self.evasion_giro
            self.parar()
            
        return

    def evasion_giro(self):
        global velocidad
        velocidad=5333
        angulo_limite=-2967067
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        time.sleep(0.1)##evitar problemas en serial
        self.inicializacion_odometria(0,0,0)
        ser.flushOutput()
        x,y,t,d,i=self.leer_odometria_total()
        self.inicializacion_odometria(0,0,0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        ser.flushOutput
        self.giro()
        time.sleep(0.1) ##evitar problemas en serial
        angulo=self.leer_odometria_r()
        while angulo>angulo_limite:
            time.sleep(0.005)
            angulo=self.leer_odometria_r()
        self.parar()

    def adelante(self):
        global velocidad, limite
        self.serial2('w')
        self.serial2(chr(99))  #configuracion motor 1 adelante 1f
        self.serial(velocidad)
        self.serial2(chr(156))  #configuracion motor 2 atras 2b
        self.serial(velocidad)
        return
    
    def atras(self):
        global velocidad, limite
        self.serial2('w')
        self.serial2(chr(147))  #configuracion motor 1 adelante 1f
        self.serial(velocidad)
        self.serial2(chr(108))  #configuracion motor 2 atras 2b
        self.serial(velocidad)
        return

    def giro(self):
        global velocidad, limite
        self.serial2('w')
        self.serial2(chr(99))  #configuracion motor 1 adelante 1f
        self.serial(velocidad)
        self.serial2(chr(108))  #configuracion motor 2 adelante 2f
        self.serial(velocidad)

    def giro_ah(self):
        global velocidad, limite
        self.serial2('w')
        self.serial2(chr(147))  #configuracion motor 1 adelante 1f
        self.serial(velocidad)
        self.serial2(chr(156))  #configuracion motor 2 adelante 2f
        self.serial(velocidad)

    def parar(self):
        global velocidad, limite
        self.serial2('w')
        self.serial2(chr(99))  #configuracion motor 1 adelante 1f
        self.serial(0)
        self.serial2(chr(108))  #configuracion motor 2 adelante 2f
        self.serial(0)
        return
##Metodo de lectura de valores odometricos
    def leer_odometria(self):
        self.serial2('r')
        time.sleep(0.005)
        teta=self.leer()
        return teta

    def leer_odometria_total(self):
        self.serial2('r')
        time.sleep(0.005)
        x,y,theta,i,d=self.leer_total()
        x=x/10000
        y=y/10000
        theta=theta/1000000
        i=i
        d=d
        return x,y,theta,i,d

    def leer_odometria_r(self):
        self.serial2('r')
        time.sleep(0.005)
        teta=self.leer_r()
        return teta
    
    def inicializacion_odometria(self,a,b,c):
        self.serial2('o')
        self.serial2('x')
        if a>=0:
            self.serial2('+')
        elif a<0:
            self.serial2('-')
        self.serial3(abs(a))
        self.serial2('y')
        if b>=0:
            self.serial2('+')
        elif b<0:
            self.serial2('-')
        self.serial3(abs(b))
        self.serial2('t')
        if c>=0:
            self.serial2('+')
        elif c<0:
            self.serial2('-')
        self.serial3(abs(c))
        return

##Metodo de lectura para sensor laser
    def leer_laser(self, angulo,angulo2):
        laser.laser_on()
        (a,b,c)=laser.get_scan(angulo,angulo2,1)
        laser.laser_off()
        return b

    def leer_laser2(self):
        laser.laser_on()
        (a,b,c)=laser.get_scan(124,643,1) #124 -- 643
        laser.laser_off()
        return a,b
    

    def leer_hokuyo_no_error(self,angulo1,angulo2):
        laser.laser_on()
        laser.set_high_sensitive(False)
        angulos,distancias,time_st = laser.get_scan(angulo1,angulo2,1)
        laser.laser_off()
        obstac = min(distancias)
        if obstac <= 10:
            while obstac <=10:
                ind_obstac = distancias.index(obstac)
                del distancias[ind_obstac]
                del angulos[ind_obstac]
                a=len(distancias)
                if a!=0:
                    obstac = min(distancias)
                else:
                    break

        if (obstac>=10):
            ind_obstac = distancias.index(obstac)
            print 'Distancia al obstaculo: ',distancias[ind_obstac]
            print 'Angulo del obstaculo: ',angulos[ind_obstac]
            dis_min=distancias[ind_obstac]
            ang_min=angulos[ind_obstac]
        else:
            dis_min=4000
            ang_min=0
        return dis_min

    def leer_hokuyo_no_error_2(self,angulo1,angulo2):
        inicio=time.time()
        angulos,distancias,time_st = laser.get_scan(angulo1,angulo2,10)
        final=time.time()
        obstac = min(distancias)
        if obstac <= 10:
            while obstac <=10:
                ind_obstac = distancias.index(obstac)
                del distancias[ind_obstac]
                del angulos[ind_obstac]
                a=len(distancias)
                if a!=0:
                    obstac = min(distancias)
                else:
                    break

        if (obstac>=10):
            ind_obstac = distancias.index(obstac)
            dis_min=distancias[ind_obstac]
            ang_min=angulos[ind_obstac]
        else:
            dis_min=4000
            ang_min=0
        final2=time.time()
        print 'el tiempo en soolo leer el hokuyo es'+ str(final-inicio)
        print 'tiempo en analizar y buscar errores'+str(final2-final)
        return dis_min,ang_min
    
##Metodos de rampa de velocidad
    def rampa(self,velocidad):
        velocidad_paso=velocidad/10
        valor=[0,0,0,0,0,0,0,0,0,0]
        for i in range(0,10):
            valor[i]=velocidad_paso*(i+1)
        valor[9]=velocidad
        return valor

    def rampa_negativa(self,velocidad):
        velocidad_paso=velocidad/10
        valor=[0,0,0,0,0,0,0,0,0,0]
        for i in range(0,10):
            valor[i]=velocidad-velocidad_paso*(i+1)
        valor[9]=0
        return valor
##Metodo de seguimiento de trayectoria
    def seguimiento_trayectorias(self):
        #llamada a la funcion de trayectoria
        if self.radioButton_cuadrada.isChecked():
            x_d,y_d,punt_p=self.trayectoria_cuadrada()            
        elif self.radioButton_circular.isChecked():
            x_d,y_d,punt_p=self.trayectoria_circular()
        elif self.radioButton_senoidal.isChecked():
            x_d,y_d,punt_p=self.trayectoria_senoidal()
        ##variable para graficos de resultados
        elemt=int(punt_p)
        rapidez=np.zeros(elemt)
        r_angular=np.zeros(elemt)
        w_l=np.zeros(elemt)
        w_r=np.zeros(elemt)
        x_out=np.zeros(elemt)
        y_out=np.zeros(elemt)
        theta_out=np.zeros(elemt)
        theta_ref=np.zeros(elemt)
        w_r_l=np.zeros(elemt)
        w_r_r=np.zeros(elemt)
        per_muestreo=np.zeros(elemt)
        ## Constantes del  controlador
        pi=math.pi
        k_x = 0.96                        #constante para suavisar la variacion en x
        k_y = 0.96                        #constante para suavisar la variacion en y
        k_theta = 0.73                    #constante para suavisar la variacion en theta
        a = 0
        T_0 = 0.1                         #periodo de muestreo
        f_0 = 1/T_0                       #frecuencia de controlador
        t_out=np.linspace(0,(elemt*T_0),elemt)
        ## posicion inicial
        x_0=0             #Posicion inicial
        y_0=0
        theta_0=0
        d_baseline=65.65  #distancia entre llantas en cm
        r_wheel=17.272    #radio de la llanta en cm
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        ser.flushOutput
        ##variables para el algoritmo: Inicializar el cuadrante anterior
        revol=0                         #numero de revoluciones
        if theta_0<0:                   #Busco si el angulo es negativo o positivo
            theta_0_1=theta_0+2*pi
        else:
            theta_0_1=theta_0           #busco en que cuadrante esta el angulo
        if (theta_0_1>=0) & (theta_0_1<=(pi/2)):
            cuad_ant=1
        elif (theta_0_1>(pi/2)) & (theta_0_1<=pi):
            cuad_ant=2
        elif (theta_0_1>pi) & (theta_0_1<=(3*pi/2)):
            cuad_ant=3
        else:
            cuad_ant=4
        ##Calculos del controlador y modelo cinematico
        x_n=x_0                           #inicializando posicion del movil
        y_n=y_0
        theta_n=theta_0
        theta_d_n_1=0
        ##lazo de calculos, sigue una trayectoria completa
        inicio_trayectoria=time.time()
        for i in range (0,elemt-1,1):
            bumper,u_frontal_s,u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
            if (bumper=='S'):
                start=time.time()
                #Calculando delta x correspondiente
                deltaX=x_d[i+1]-x_n-k_x*(x_d[i]-x_n)
                deltaY=y_d[i+1]-y_n-k_y*(y_d[i]-y_n)
                print (deltaX,deltaY)
                #calculando el angulo deseaado para la trayectoria en dicho punto (iteracion actual)
                #deteminando el sentido de giro anterior
                if deltaX>=0:
                    if deltaY>=0:  # 1 cuadrante
                        cuad_ac=1  #cuadrante actual
                        #Determinar el sentido del theta_d
                        #usando el sentido de giro anterior
                        if cuad_ant==4:      #Cuadrante conflicto
                            if theta_d_n_1>=0:  #Sentido anterior (+)
                                revol=revol+1   #Da una revolucion mas
                                theta_d=np.arctan(deltaY/deltaX)+(revol*2*pi)
                            else:    #Sentido anterior (-)
                                if revol>0:   #A dado mas de 1 revolucion
                                    revol=revol-1 #a regresado una revolucion
                                    theta_d=np.arctan(deltaY/deltaX)-((revol+1)*2*pi)
                                else:  #solo cambia de sentido
                                    theta_d=np.arctan(deltaY/deltaX)
                        else:  #otros cuadrantes
                            if theta_d_n_1>=0:  #sentido anterior (+)
                                theta_d=np.arctan(deltaY/deltaX)+(revol*2*pi)
                            else:
                                theta_d=np.arctan(deltaY/deltaX)-((revol+1)*2*pi)
                    else:     #4 cuadrante
                        cuad_ac=4
                        #Determinar el sentido de theta
                        #usando el sentido de giro anterior
                        if cuad_ant==1: #cuadrante conflictivo
                            if theta_d_n_1>=0:  #sentido anterior (+)
                                if revol>0:   #a dado mas de 1 revolucion
                                    revol=revol-1 #se regresa 1 revolucion
                                    theta_d=np.arctan(deltaY/deltaX)+((revol+1)*2*pi)
                                else:    #solo cambio de sentido de giro
                                    theta_d=np.arctan(deltaY/deltaX)
                            else:  #sentido anterior (-)
                                revol=revol+1  #da una revolucion mas
                                theta_d=np.arctan(deltaY/deltaX)-(revol*2*pi)

                        else:   #otros cuadrantes
                            if theta_d_n_1>=0:  #sentido anterior positivo (+)
                                theta_d=np.arctan(deltaY/deltaX)+((revol+1)*2*pi)
                            else:    #sentido anterior (-)
                                theta_d=np.arctan(deltaY/deltaX)-(revol*2*pi)
                else:  # II y III cuadrante
                    if deltaY>=0:   #II cuadrante
                        cuad_ac=2   #cuadrante actual
                    else:  # III cuadrante
                        cuad_ac=3  #cuadrante actual
                    theta_d=np.arctan(deltaY/deltaX)+((np.sign(theta_d_n_1))*(pi+(revol*2*pi)))
                cuad_ant=cuad_ac   #actualizando el cuadrante
                theta_d_n_1=theta_d   #actualizando la orientacion deseada
                theta_ref[i]=theta_d
                deltaTheta=theta_d-theta_n-k_theta*(theta_d-theta_n)
                #calculando las acciones de control
                v_n=f_0*((deltaX*cos(theta_d))+(deltaY*sin(theta_d)))
                rapidez[i]=v_n #guardando el valor calculado
                w_n=f_0*(1/((a^2)+1))*(deltaTheta-a*((deltaX*sin(theta_d))-(deltaY*cos(theta_d))))
                r_angular[i]=w_n   #guardando el valor calculado
                #calculando la rapidez angular de cada llanta
                w_l_n=(v_n-((d_baseline/2)*w_n))/(r_wheel)
                w_l[i]=w_l_n
                w_r_n=(v_n+((d_baseline/2)*w_n))/(r_wheel)
                w_r[i]=w_r_n
                f_l_n=(20480/pi)*w_l_n   #transformacion a frecuencia de motores
                f_r_n=(20480/pi)*w_r_n
                f_l_n_a=int(f_l_n)
                f_r_n_a=int(f_r_n)
                if f_l_n_a>22000:  #saturador
                    f_l_n_a=22000
                if f_r_n_a>22000:
                    f_r_n_a=22000
                if f_l_n_a<0:  #saturador
                    f_l_n_a=0
                if f_r_n_a<0:
                    f_r_n_a=0
                print 'el valor de velocidad es'+str(f_l_n_a)
                print 'el valor de velocidad es'+str(f_r_n_a)
                self.serial2('w')
                self.serial2(chr(99))  #configuracion motor 1 adelante 1f
                self.serial(f_l_n_a)
                self.serial2(chr(156))  #configuracion motor 2 atras 2b
                self.serial(f_r_n_a)
                time.sleep(0.09)
                stop=time.time()
                per_muestreo[i]=stop-inicio_trayectoria
                print 'tiempo='+str(stop-start)
                ##Calculando la posicion con el modelo cinematico
                x_out[i]=x_n   # guardando posiciones
                y_out[i]=y_n
                theta_out[i]=theta_n
                x_n,y_n,theta_n,iz,d=self.leer_odometria_total()
                w_r_l[i]=iz
                w_r_r[i]=d
            else:
                break
        final_trayectoria=time.time()
        tiempo_trayectoria=final_trayectoria-inicio_trayectoria
        self.parar()
        print(tiempo_trayectoria)
        
        fig1=figure(1)
        line1,=plot(x_out,y_out,'bo',label='Trayectoria Seguida')
        line2,=plot(x_d,y_d,'r',label='Trayectoria Deseada')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=4)})
        plt.legend(bbox_to_anchor=(0, 0), loc=2, borderaxespad=0.)
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        fig1.suptitle('Trayectoria Seguida vs Trayectoria Deseada', fontsize=18)
        savefig('trayectoria_comparada.png')
        
        jo=len(per_muestreo)-2
        fig2=figure(2)
        theta_out=theta_out*180/math.pi
        line2,=plot(per_muestreo[:jo-1],theta_out[:jo-1],'b',label='Posicion Angular')
        grid(True)
        plt.ylabel('Theta (grad)')
        plt.xlabel('Tiempo (s)')
        fig2.suptitle('Posicion Angular', fontsize=18)
        savefig('posicionAngular.png')
        
        fig3=figure(3)
        line1,=plot(per_muestreo[:jo-1],w_l[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 1')
        line2,=plot(per_muestreo[:jo-1],w_r_l[:jo-1]*3/2048,'r',label='Velocidad angular real motor 1')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig3.suptitle('Velocidad Angular Motor 1 (Izquierdo)', fontsize=18)
        savefig('velocidad_motor1.png')

        fig4=figure(4)
        line1,=plot(per_muestreo[:jo-1],w_r[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 2')
        line2,=plot(per_muestreo[:jo-1],w_r_r[:jo-1]*3/2048,'r',label='Velocidad angular real motor 2')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig4.suptitle('Velocidad Angular Motor 2 (Derecho)', fontsize=18)
        savefig('velocidad_motor2.png')
        
        
        fig5=figure(5)
        line1,=plot(per_muestreo[:jo-1],rapidez[:jo-1],'bo',label='Velocidad Lineal Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]+w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*2),'r',label='Velocidad Lineal Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad lineal (cm/s)')
        fig5.suptitle('Velocidad Lineal', fontsize=18)
        savefig('velocidad_lineal.png')

        fig6=figure(6)
        line1,=plot(per_muestreo[:jo-1],r_angular[:jo-1],'bo',label='Velocidad Angular Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]-w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*d_baseline),'r',label='Velocidad Angular Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad Angular (rad/s)')
        fig6.suptitle('Velocidad Angular', fontsize=18)
        savefig('velocidad_angular.png')

        plt.show()
        self.guardar_excel('prueba1','noevasion',t_out,x_out,y_out,x_d,y_d,theta_out,w_r,w_l,w_r_r,w_r_l,rapidez,r_angular,tiempo_trayectoria,per_muestreo)
        return

    def seguimiento_trayectorias_evasion(self):
        if self.radioButton_cuadrada.isChecked():
            x_d,y_d,punt_p=self.trayectoria_cuadrada()            
        elif self.radioButton_circular.isChecked():
            x_d,y_d,punt_p=self.trayectoria_circular()
        elif self.radioButton_senoidal.isChecked():
            x_d,y_d,punt_p=self.trayectoria_senoidal()
        ##variable para graficos de resultados
        elemt=int(punt_p)
        rapidez=np.zeros(elemt)
        r_angular=np.zeros(elemt)
        w_l=np.zeros(elemt)
        w_r=np.zeros(elemt)
        x_out=np.zeros(elemt)
        y_out=np.zeros(elemt)
        theta_out=np.zeros(elemt)
        theta_ref=np.zeros(elemt)
        w_r_l=np.zeros(elemt)
        w_r_r=np.zeros(elemt)
        per_muestreo=np.zeros(elemt)
        ## Constantes del  controlador
        pi=math.pi
        k_x = 0.98                        #constante para suavisar la variacion en x
        k_y = 0.98                        #constante para suavisar la variacion en y
        k_theta = 0.73                    #constante para suavisar la variacion en theta
        a = 0
        T_0 = 0.1                         #periodo de muestreo
        f_0 = 1/T_0                       #frecuencia de controlador
        t_out=np.linspace(0,(elemt*T_0),elemt)

        ## posicion inicial
        x_0=0   #Posicion inicial
        y_0=0
        theta_0=0
        d_baseline=65.65  #distancia entre llantas en cm
        r_wheel=17.272    #radio de la llanta en cm
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        #while (x_0!=x) | (y_0!=y) | (theta_0!=t):
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,d=self.leer_odometria_total()
        ser.flushOutput
        #x,y,t,d,i=self.leer_odometria_total()

    ##evasion de obstaculos
        k_alpha = 0.6                     #constante calculo angulo de desviacion
        d_max = 100                       #distancia que define la zona de repulsion
        d_min = 50                        #distancia minima sin contacto robot obstaculo
        sigma = 0.5                       #parametro proporcional de la rapidez modificada
        ##variables para el algoritmo: Inicializar el cuadrante anterior
        revol=0  #numero de revoluciones
        if theta_0<0:    #Busco si el angulo es negativo o positivo
            theta_0_1=theta_0+2*pi
        else:
            theta_0_1=theta_0
                      #busco en que cuadrante esta el angulo
        if (theta_0_1>=0) & (theta_0_1<=(pi/2)):
            cuad_ant=1
        elif (theta_0_1>(pi/2)) & (theta_0_1<=pi):
            cuad_ant=2
        elif (theta_0_1>pi) & (theta_0_1<=(3*pi/2)):
            cuad_ant=3
        else:
            cuad_ant=4
        ##Calculos del controlador y modelo cinematico
        x_n=x_0              #inicializando posicion del movil
        y_n=y_0
        theta_n=theta_0
        theta_d_n_1=0
        ##lazo de calculos, sigue una trayectoria completa
        laser.laser_on()  #enciendo el laser
        laser.set_high_sensitive(False)
        inicio_trayectoria=time.time()
        for i in range (0,elemt-1,1):
            bumper,u_frontal_s,u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
            if (bumper=='S'):
                start=time.time()
                #Calculando delta x correspondiente
                deltaX=x_d[i+1]-x_n-k_x*(x_d[i]-x_n)
                deltaY=y_d[i+1]-y_n-k_y*(y_d[i]-y_n)
                #print (deltaX,deltaY)
                inicio=time.time()
                d,betha=self.leer_hokuyo_no_error_2(124,643)
                final=time.time()
                print 'la medicion dura'+str(final-inicio)
                d=d/10
                betha=pi*betha/180
                #Comparando la distancia con la zona de repulsion
                if d > d_max:                 #Fuera de la zona de repulsion
                    deltaXu = deltaX          #sin cambios en el algoritmo
                    deltaYu = deltaY
                elif d <= d_max:              #Dentro de la zona de repulsion
                #Calculando d prima
                    if d > d_min:
                        d_prima = d
                    elif d <= d_min:
                        d_prima = d_min
                    if d_prima==d_min:
                        angulo=float('inf')
                    else:
                        angulo=(k_alpha)/(abs(d_min - d_prima))
                    alpha = (arctan(angulo))*(sign(sin(betha)))
                #Calculando los desplazamientos en x e y modificados
                    deltaXu = sigma*((deltaX*cos(alpha)) - (deltaY*sin(alpha)))
                    deltaYu = sigma*((deltaX*sin(alpha)) + (deltaY*cos(alpha)))
                if deltaXu>=0:
                    if deltaYu>=0:  # 1 cuadrante
                        cuad_ac=1  #cuadrante actual
                        #Determinar el sentido del theta_d
                        #usando el sentido de giro anterior
                        if cuad_ant==4:      #Cuadrante conflicto
                            if theta_d_n_1>=0:  #Sentido anterior (+)
                                revol=revol+1   #Da una revolucion mas
                                theta_d=np.arctan(deltaYu/deltaXu)+(revol*2*pi)
                            else:    #Sentido anterior (-)
                                if revol>0:   #A dado mas de 1 revolucion
                                    revol=revol-1 #a regresado una revolucion
                                    theta_d=np.arctan(deltaYu/deltaXu)-((revol+1)*2*pi)
                                else:  #solo cambia de sentido
                                    theta_d=np.arctan(deltaYu/deltaXu)
                        else:  #otros cuadrantes
                            if theta_d_n_1>=0:  #sentido anterior (+)
                                theta_d=np.arctan(deltaYu/deltaXu)+(revol*2*pi)
                            else:
                                theta_d=np.arctan(deltaYu/deltaXu)-((revol+1)*2*pi)
                    else:     #4 cuadrante
                        cuad_ac=4
                        #Determinar el sentido de theta
                        #usando el sentido de giro anterior
                        if cuad_ant==1: #cuadrante conflictivo
                            if theta_d_n_1>=0:  #sentido anterior (+)
                                if revol>0:   #a dado mas de 1 revolucion
                                    revol=revol-1 #se regresa 1 revolucion
                                    theta_d=np.arctan(deltaYu/deltaXu)+((revol+1)*2*pi)
                                else:    #solo cambio de sentido de giro
                                    theta_d=np.arctan(deltaYu/deltaXu)
                            else:  #sentido anterior (-)
                                revol=revol+1  #da una revolucion mas
                                theta_d=np.arctan(deltaYu/deltaXu)-(revol*2*pi)

                        else:   #otros cuadrantes
                            if theta_d_n_1>=0:  #sentido anterior positivo (+)
                                theta_d=np.arctan(deltaYu/deltaXu)+((revol+1)*2*pi)
                            else:    #sentido anterior (-)
                                theta_d=np.arctan(deltaYu/deltaXu)-(revol*2*pi)
                else:  # II y III cuadrante
                    if deltaYu>=0:   #II cuadrante
                        cuad_ac=2   #cuadrante actual
                    else:  # III cuadrante
                        cuad_ac=3  #cuadrante actual
                    theta_d=np.arctan(deltaYu/deltaXu)+((np.sign(theta_d_n_1))*(pi+(revol*2*pi)))
                cuad_ant=cuad_ac   #actualizando el cuadrante
                theta_d_n_1=theta_d   #actualizando la orientacion deseada
                theta_ref[i]=theta_d
                deltaTheta=theta_d-theta_n-k_theta*(theta_d-theta_n)
                #calculando las acciones de control
                v_n=f_0*((deltaXu*cos(theta_d))+(deltaYu*sin(theta_d)))
                rapidez[i]=v_n #guardando el valor calculado
                w_n=f_0*(1/((a^2)+1))*(deltaTheta-a*((deltaXu*sin(theta_d))-(deltaYu*cos(theta_d))))
                r_angular[i]=w_n   #guardando el valor calculado
                #calculando la rapidez angular de cada llanta
                w_l_n=(v_n-((d_baseline/2)*w_n))/(r_wheel)
                w_l[i]=w_l_n
                w_r_n=(v_n+((d_baseline/2)*w_n))/(r_wheel)
                w_r[i]=w_r_n
                f_l_n=(20480/pi)*w_l_n   #transformacion a frecuencia de motores
                f_r_n=(20480/pi)*w_r_n
                f_l_n_a=int(f_l_n)
                f_r_n_a=int(f_r_n)
                if f_l_n_a>22000:  #saturador
                    f_l_n_a=22000
                if f_r_n_a>22000:
                    f_r_n_a=22000
                if f_l_n_a<0:  #saturador
                    f_l_n_a=0
                if f_r_n_a<0:
                    f_r_n_a=0
                self.serial2('w')
                self.serial2(chr(99))  #configuracion motor 1 adelante 1f
                self.serial(f_l_n_a)
                self.serial2(chr(156))  #configuracion motor 2 atras 2b
                self.serial(f_r_n_a)
                stop=time.time()
                per_muestreo[i]=stop-inicio_trayectoria
                print 'tiempo='+str(stop-start)
                print i
                x_out[i]=x_n   # guardando posiciones
                y_out[i]=y_n
                theta_out[i]=theta_n
                x_n,y_n,theta_n,iz,d=self.leer_odometria_total()
                w_r_l[i]=iz
                w_r_r[i]=d
                
            else:
                break
        
        laser.laser_off()  #apago laser
        final_trayectoria=time.time()
        tiempo_trayectoria=final_trayectoria-inicio_trayectoria
        print(tiempo_trayectoria)
        self.parar()
        
        fig1=figure(1)
        line1,=plot(x_out,y_out,'bo',label='Trayectoria Seguida')
        line2,=plot(x_d,y_d,'r',label='Trayectoria Deseada')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=4)})
        plt.legend(bbox_to_anchor=(0, 0), loc=2, borderaxespad=0.)
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        fig1.suptitle('Trayectoria Seguida vs Trayectoria Deseada', fontsize=18)
        savefig('trayectoria_comparada.png')
        
        jo=len(per_muestreo)-2
        fig2=figure(2)
        theta_out=theta_out*180/math.pi
        line2,=plot(per_muestreo[:jo-1],theta_out[:jo-1],'b',label='Posicion Angular')
        grid(True)
        plt.ylabel('Theta (grad)')
        plt.xlabel('Tiempo (s)')
        fig2.suptitle('Posicion Angular', fontsize=18)
        savefig('posicionAngular.png')
        
        fig3=figure(3)
        line1,=plot(per_muestreo[:jo-1],w_l[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 1')
        line2,=plot(per_muestreo[:jo-1],w_r_l[:jo-1]*3/2048,'r',label='Velocidad angular real motor 1')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig3.suptitle('Velocidad Angular Motor 1 (Izquierdo)', fontsize=18)
        savefig('velocidad_motor1.png')

        fig4=figure(4)
        line1,=plot(per_muestreo[:jo-1],w_r[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 2')
        line2,=plot(per_muestreo[:jo-1],w_r_r[:jo-1]*3/2048,'r',label='Velocidad angular real motor 2')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig4.suptitle('Velocidad Angular Motor 2 (Derecho)', fontsize=18)
        savefig('velocidad_motor2.png')
        
        
        fig5=figure(5)
        line1,=plot(per_muestreo[:jo-1],rapidez[:jo-1],'bo',label='Velocidad Lineal Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]+w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*2),'r',label='Velocidad Lineal Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad lineal (cm/s)')
        fig5.suptitle('Velocidad Lineal', fontsize=18)
        savefig('velocidad_lineal.png')

        fig6=figure(6)
        line1,=plot(per_muestreo[:jo-1],r_angular[:jo-1],'bo',label='Velocidad Angular Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]-w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*d_baseline),'r',label='Velocidad Angular Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad Angular (rad/s)')
        fig6.suptitle('Velocidad Angular', fontsize=18)
        savefig('velocidad_angular.png')

        plt.show()
        self.guardar_excel('prueba2','evasion',t_out,x_out,y_out,x_d,y_d,theta_out,w_r,w_l,w_r_r,w_r_l,rapidez,r_angular,tiempo_trayectoria,per_muestreo)

        return

    def seguimiento_trayectorias_careli(self):
        ##llamada a la funcion de trayectoria
        if self.radioButton_cuadrada.isChecked():
            x_d,y_d,punt_p,d_x_d,d_y_d=self.trayectoria_careli_cuadrada()            
        elif self.radioButton_circular.isChecked():
            x_d,y_d,punt_p,d_x_d,d_y_d=self.trayectoria_careli_circular() 
        elif self.radioButton_senoidal.isChecked():
            x_d,y_d,punt_p,d_x_d,d_y_d=self.trayectoria_careli_senoidal()
        elemt=int(punt_p)
        rapidez=np.zeros(elemt)
        r_angular=np.zeros(elemt)
        w_l=np.zeros(elemt)
        w_r=np.zeros(elemt)
        x_out=np.zeros(elemt)
        y_out=np.zeros(elemt)
        theta_out=np.zeros(elemt)
        theta_ref=np.zeros(elemt)
        w_r_l=np.zeros(elemt)
        w_r_r=np.zeros(elemt)
        per_muestreo=np.zeros(elemt)
    ##Inicializacion de las Constantes del Controlador
        pi=math.pi
        k_x = 0.4                        #Ganancias del compensador de velocidad
        k_y = 0.4
        a = 3.347393                   #Distancia entre el punto a ser movido por el controlado A y el centro de la baseline de las llantas B
        l_x = 25                       #Constantes de saturacion
        l_y = 25
        T_0 = 0.1                      #Periodo de Muestreo Paper asegura que es 0.1 s para pioneer 2dx
        d_baseline=65.65  #distancia entre llantas en cm
        r_wheel=17.272    #radio de la llanta en cm
        T_0 = 0.1                         ##periodo de muestreo
        f_0 = 1/T_0                       ##frecuencia de controlador
        t_out=np.linspace(0,(elemt*T_0)-T_0,elemt)
        print(t_out)
        ## posicion inicial
        x_0=0   #Posicion inicial
        y_0=0
        theta_0=0
        d_baseline=65.65  #distancia entre llantas en cm
        r_wheel=17.272    #radio de la llanta en cm
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,r=self.leer_odometria_total()
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,r=self.leer_odometria_total()
        ser.flushOutput
        ##Calculos del controlador y modelo cinematico
        x_n=x_0              #inicializando posicion del movil
        y_n=y_0
        theta_n=theta_0
        inicio_trayectoria=time.time()
        ##lazo de calculos, sigue una trayectoria completa
        for i in range (0,elemt-1,1):
            bumper,u_frontal_s,u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
            if (bumper=='S'):
                start=time.time()
            #Calculando entradas al bloque compensador de velocidad (desplazamiento)
                x_error = x_d[i] - x_n         #Desplazamiento en x
                y_error = y_d[i] - y_n        #Desplazamiento en y
                #Bloque compensador de velocidad (salidas del bloque v_x y v_y)
                v_x = (-l_x)*np.tanh((k_x/l_x)*x_error)
                v_y = (-l_y)*np.tanh((k_y/l_y)*y_error)
                #Calculando Entradas al Bloque Cinematica Inversa (Referencias de Velocidad)
                d_x_r = d_x_d[i] - v_x
                d_y_r = d_y_d[i] - v_y
                #Bloque cinematica inversa (salidas del bloque v_n y w_n)
                #Acciones de control
                v_n = (d_x_r*cos(theta_n)) + (d_y_r*sin(theta_n))
                rapidez[i] = v_n          #Guardando el valor calculado
                w_n = ((-1/a)*(d_x_r)*sin(theta_n)) + ((1/a)*(d_y_r)*cos(theta_n))
                r_angular[i] = w_n         #Guardando el valor calculado
                #Calculando la rapidez anduglar de cada llanta
                w_l_n = (v_n - ((d_baseline/2)*w_n))/(r_wheel)
                w_l[i] = w_l_n
                w_r_n = (v_n + ((d_baseline/2)*w_n))/(r_wheel)
                w_r[i] = w_r_n
                stop=time.time()
                f_l_n=(20480/pi)*w_l_n   #transformacion a frecuencia de motores
                f_r_n=(20480/pi)*w_r_n
                f_l_n_a=int(f_l_n)
                f_r_n_a=int(f_r_n)
                if f_l_n_a>=0:
                    dir_l=chr(99)   #configuracion motor 1 adelante 1f
                    f_l_n_b=abs(f_l_n_a)
                    if f_l_n_b>22000:  #saturador
                        f_l_n_b=22000
                elif f_l_n_a<0:  #saturador
                    dir_l=chr(147)  #configuracion motor 1 atras 1b
                    f_l_n_b=abs(f_l_n_a)
                    if f_l_n_b>22000:  #saturador
                        f_l_n_b=22000
                if f_r_n_a>=0:
                    dir_r=chr(156)   #configuracion motor 2 atras 2b
                    f_r_n_b=abs(f_r_n_a)
                    if f_r_n_b>22000:  #saturador
                        f_r_n_b=22000
                elif f_r_n_a<0:  #saturador
                    dir_r=chr(108)  #configuracion motor 2 adelante 2f
                    f_r_n_b=abs(f_r_n_a)
                    if f_r_n_b>22000:  #saturador
                        f_r_n_b=22000
                print 'el valor de velocidad es'+str(f_l_n_a)
                print 'el valor de velocidad es'+str(f_r_n_a)
                self.serial2('w')
                self.serial2(dir_l)  #configuracion motor 1 adelante 1f
                self.serial(f_l_n_b)
                self.serial2(dir_r)  #configuracion motor 2 atras 2b
                self.serial(f_r_n_b)
                time.sleep(0.085)
                stop=time.time()
                per_muestreo[i]=stop-inicio_trayectoria
                print 'tiempo='+str(stop-start)
                ##Calculando la posicion con el modelo cinematico
                x_out[i]=x_n   # guardando posiciones
                y_out[i]=y_n
                theta_out[i]=theta_n
                w_r_l[i]=iz
                w_r_r[i]=r
                ##lectura de odometria
                x_n,y_n,theta_n,iz,r=self.leer_odometria_total()
            else:
                break
        
        self.parar()
        final_trayectoria=time.time()
        tiempo_trayectoria=final_trayectoria-inicio_trayectoria
        print tiempo_trayectoria
        print len(theta_out)
        print len(d_x_d)
        print len(x_d)
        
        fig1=figure(1)
        line1,=plot(x_out,y_out,'bo',label='Trayectoria Seguida')
        line2,=plot(x_d,y_d,'r',label='Trayectoria Deseada')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=4)})
        plt.legend(bbox_to_anchor=(0, 0), loc=2, borderaxespad=0.)
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        fig1.suptitle('Trayectoria Seguida vs Trayectoria Deseada', fontsize=18)
        savefig('trayectoria_comparada.png')
        
        jo=len(per_muestreo)-2
        fig2=figure(2)
        theta_out=theta_out*180/math.pi
        line2,=plot(per_muestreo[:jo-1],theta_out[:jo-1],'b',label='Posicion Angular')
        grid(True)
        plt.ylabel('Theta (grad)')
        plt.xlabel('Tiempo (s)')
        fig2.suptitle('Posicion Angular', fontsize=18)
        savefig('posicionAngular.png')
        
        fig3=figure(3)
        line1,=plot(per_muestreo[:jo-1],w_l[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 1')
        line2,=plot(per_muestreo[:jo-1],w_r_l[:jo-1]*3/2048,'r',label='Velocidad angular real motor 1')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig3.suptitle('Velocidad Angular Motor 1 (Izquierdo)', fontsize=18)
        savefig('velocidad_motor1.png')

        fig4=figure(4)
        line1,=plot(per_muestreo[:jo-1],w_r[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 2')
        line2,=plot(per_muestreo[:jo-1],w_r_r[:jo-1]*3/2048,'r',label='Velocidad angular real motor 2')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig4.suptitle('Velocidad Angular Motor 2 (Derecho)', fontsize=18)
        savefig('velocidad_motor2.png')
        
        
        fig5=figure(5)
        line1,=plot(per_muestreo[:jo-1],rapidez[:jo-1],'bo',label='Velocidad Lineal Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]+w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*2),'r',label='Velocidad Lineal Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad lineal (cm/s)')
        fig5.suptitle('Velocidad Lineal', fontsize=18)
        savefig('velocidad_lineal.png')

        fig6=figure(6)
        line1,=plot(per_muestreo[:jo-1],r_angular[:jo-1],'bo',label='Velocidad Angular Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]-w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*d_baseline),'r',label='Velocidad Angular Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad Angular (rad/s)')
        fig6.suptitle('Velocidad Angular', fontsize=18)
        savefig('velocidad_angular.png')

        plt.show()
        self.guardar_excel_careli('prueba1careli','noevasion',t_out,x_out,y_out,x_d,y_d,theta_out,w_r,w_l,w_r_r,w_r_l,rapidez,r_angular,tiempo_trayectoria,per_muestreo,d_x_d,d_y_d)
        return

    def seguimiento_trayectorias_careli_evasion(self):
        
        ##llamada a la funcion de trayectoria
        if self.radioButton_cuadrada.isChecked():
            x_d,y_d,punt_p,d_x_m,d_y_m=self.trayectoria_careli_cuadrada()            
        elif self.radioButton_circular.isChecked():
            x_d,y_d,punt_p,d_x_m,d_y_m=self.trayectoria_careli_circular() 
        elif self.radioButton_senoidal.isChecked():
            x_d,y_d,punt_p,d_x_m,d_y_m=self.trayectoria_careli_senoidal()
        ##variable para graficos de resultados
        elemt=int(punt_p)
        rapidez=np.zeros(elemt)
        r_angular=np.zeros(elemt)
        w_l=np.zeros(elemt)
        w_r=np.zeros(elemt)
        x_out=np.zeros(elemt)
        y_out=np.zeros(elemt)
        theta_out=np.zeros(elemt)
        theta_ref=np.zeros(elemt)
        w_r_l=np.zeros(elemt)
        w_r_r=np.zeros(elemt)
        per_muestreo=np.zeros(elemt)
        bandera=False

    ##Inicializacion de las Constantes del Controlador
        pi=math.pi
        k_x =0.4                     #Ganancias del compensador de velocidad
        k_y = 0.4
        k_a = 0.95                      #retardo para estabilizar alpha        
        alpha_n = 0
        alpha_n_1 = 0
        
        a = 3.347393                   #Distancia entre el punto a ser movido por el controlado A y el centro de la baseline de las llantas B
        l_x = 25                       #Constantes de saturacion
        l_y = 25
        T_0 = 0.1                      #Periodo de Muestreo Paper asegura que es 0.1 s para pioneer 2dx
        d_baseline=65.65  #distancia entre llantas en cm
        r_wheel=17.272    #radio de la llanta en cm
        T_0 = 0.1                         ##periodo de muestreo
        f_0 = 1/T_0                       ##frecuencia de controlador
        t_out=np.linspace(0,(elemt*T_0)-T_0,elemt)
        print(t_out)
        
        ## Constantes Evasion
        k_alpha=500
        d_max=40
        d_min=40
        sigma=0.1       
        k_v=0.5
        k_w=0.5

        ## posicion inicial
        x_0=0   #Posicion inicial
        y_0=0
        theta_0=0
        
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,r=self.leer_odometria_total()
        self.inicializacion_odometria(x_0,y_0,theta_0)
        ser.flushOutput()
        x,y,t,iz,r=self.leer_odometria_total()
        ser.flushOutput

        ##Calculos del controlador y modelo cinematico
        x_n=x_0              #inicializando posicion del movil
        y_n=y_0
        theta_n=theta_0
        v_n_1=0
        w_n_1=0

        ##lazo de calculos, sigue una trayectoria completa
        #Encender Laser
        laser.laser_on() #encender laser
        laser.set_high_sensitive(False)
        inicio_trayectoria=time.time()
        for i in range (0,elemt-1,1):
            bumper,u_frontal_s,u_frontal_d, u_trasero_s, u_trasero_d=self.lectura_arduino()
            if (bumper=='S'):
                start=time.time()
                d,betha = self.leer_hokuyo_no_error_2(124,643)
                d=d/10
                betha=pi*betha/180
                if d > d_max:                 #Fuera de la zona de repulsion
                    bandera=False
                    d_x_d = d_x_m[i]
                    d_y_d = d_y_m[i]
                elif d <= d_max:              #Dentro de la zona de repulsion
                    print 'evadiendo'
                #Calculando d prima
                    if d >=  d_min:
                        bandera=False
                        d_prima = d
                    elif d <d_min:
                        bandera=False
                        d_prima = d_min
                    if d_prima==d_min:
                        angulo=float('inf')
                    else:
                        print 'si calcula el angulo'
                        angulo=(k_alpha)/(abs(d_min - d_prima))
                    alpha_n = (arctan(angulo))*(sign(sin(betha)))
                    alpha_n = alpha_n - k_a*(alpha_n - alpha_n_1)
                    alpha_n_1 = alpha_n
                    #Calculando las velocidades modificadas
                    d_x_d = sigma*(d_x_m[i]*cos(alpha_n)) - (d_y_m[i]*sin(alpha_n))
                    d_y_d = sigma*(d_x_m[i]*sin(alpha_n)) + (d_y_m[i]*cos(alpha_n))
            
            #Calculando entradas al bloque compensador de velocidad (desplazamiento)
                x_error = x_d[i] - x_n        #Desplazamiento en x
                y_error = y_d[i] - y_n        #Desplazamiento en y

                #Bloque compensador de velocidad (salidas del bloque v_x y v_y)
                v_x = (-l_x)*np.tanh((k_x/l_x)*x_error)
                v_y = (-l_y)*np.tanh((k_y/l_y)*y_error)

                #Calculando Entradas al Bloque Cinematica Inversa (Referencias de Velocidad)
                d_x_r = d_x_d - v_x
                d_y_r = d_y_d - v_y

                #Bloque cinematica inversa (salidas del bloque v_n y w_n)
                #Acciones de control
                v_n = (d_x_r*cos(theta_n)) + (d_y_r*sin(theta_n))
                w_n = ((-1/a)*(d_x_r)*sin(theta_n)) + ((1/a)*(d_y_r)*cos(theta_n))
                if bandera==True:
                    v_n=v_n-k_v*(v_n-v_n_1)
                    w_n=w_n-k_w*(w_n-w_n_1)
                rapidez[i] = v_n          #Guardando el valor calculado
                r_angular[i] = w_n         #Guardando el valor calculado
                #Calculando la rapidez anduglar de cada llanta
                w_l_n = (v_n - ((d_baseline/2)*w_n))/(r_wheel)
                w_l[i] = w_l_n
                w_r_n = (v_n + ((d_baseline/2)*w_n))/(r_wheel)
                w_r[i] = w_r_n
                f_l_n=(20480/pi)*w_l_n   #transformacion a frecuencia de motores
                f_r_n=(20480/pi)*w_r_n
                f_l_n_a=int(f_l_n)
                f_r_n_a=int(f_r_n)
                if f_l_n_a>=0:
                    dir_l=chr(99)   #configuracion motor 1 adelante 1f
                    f_l_n_b=abs(f_l_n_a)
                    if f_l_n_b>22000:  #saturador
                        f_l_n_b=22000
                elif f_l_n_a<0:  #saturador
                    dir_l=chr(147)  #configuracion motor 1 atras 1b
                    f_l_n_b=abs(f_l_n_a)
                    if f_l_n_b>22000:  #saturador
                        f_l_n_b=22000
                if f_r_n_a>=0:
                    dir_r=chr(156)   #configuracion motor 2 atras 2b
                    f_r_n_b=abs(f_r_n_a)
                    if f_r_n_b>22000:  #saturador
                        f_r_n_b=22000
                elif f_r_n_a<0:  #saturador
                    dir_r=chr(108)  #configuracion motor 2 adelante 2f
                    f_r_n_b=abs(f_r_n_a)
                    if f_r_n_b>22000:  #saturador
                        f_r_n_b=22000
                self.serial2('w')
                self.serial2(dir_l)  #configuracion motor 1 adelante 1f
                self.serial(f_l_n_b)
                self.serial2(dir_r)  #configuracion motor 2 atras 2b
                self.serial(f_r_n_b)
                ##Calculando la posicion con el modelo cinematico
                x_out[i]=x_n   # guardando posiciones
                y_out[i]=y_n
                theta_out[i]=theta_n
                w_r_l[i]=iz
                w_r_r[i]=r
                ##lectura de odometria
                odometria=time.time()
                x_n,y_n,theta_n,iz,r=self.leer_odometria_total()
                v_n_1=(i+r)*r_wheel/2
                w_n_1=(r-i)*r_wheel/d_baseline
                stop=time.time()
                per_muestreo[i]=stop-inicio_trayectoria
                print 'tiempo='+str(stop-start)
            else:
                break
        laser.laser_off() #apagar el laser
        self.parar()
        final_trayectoria=time.time()
        tiempo_trayectoria=final_trayectoria-inicio_trayectoria
        print len(theta_out)
        print len(d_x_m)
        print len(x_d)
        
        fig1=figure(1)
        line1,=plot(x_out,y_out,'bo',label='Trayectoria Seguida')
        line2,=plot(x_d,y_d,'r',label='Trayectoria Deseada')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=4)})
        plt.legend(bbox_to_anchor=(0, 0), loc=2, borderaxespad=0.)
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        fig1.suptitle('Trayectoria Seguida vs Trayectoria Deseada', fontsize=18)
        savefig('trayectoria_comparada.png')
        
        jo=len(per_muestreo)-2
        fig2=figure(2)
        theta_out=theta_out*180/math.pi
        line2,=plot(per_muestreo[:jo-1],theta_out[:jo-1],'b',label='Posicion Angular')
        grid(True)
        plt.ylabel('Theta (grad)')
        plt.xlabel('Tiempo (s)')
        fig2.suptitle('Posicion Angular', fontsize=18)
        savefig('posicionAngular.png')
        
        fig3=figure(3)
        line1,=plot(per_muestreo[:jo-1],w_l[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 1')
        line2,=plot(per_muestreo[:jo-1],w_r_l[:jo-1]*3/2048,'r',label='Velocidad angular real motor 1')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig3.suptitle('Velocidad Angular Motor 1 (Izquierdo)', fontsize=18)
        savefig('velocidad_motor1.png')

        fig4=figure(4)
        line1,=plot(per_muestreo[:jo-1],w_r[:jo-1]*30/math.pi,'bo',label='Velocidad angular calculada motor 2')
        line2,=plot(per_muestreo[:jo-1],w_r_r[:jo-1]*3/2048,'r',label='Velocidad angular real motor 2')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('velocidad angular (RPM)')
        fig4.suptitle('Velocidad Angular Motor 2 (Derecho)', fontsize=18)
        savefig('velocidad_motor2.png')
        
        
        fig5=figure(5)
        line1,=plot(per_muestreo[:jo-1],rapidez[:jo-1],'bo',label='Velocidad Lineal Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]+w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*2),'r',label='Velocidad Lineal Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad lineal (cm/s)')
        fig5.suptitle('Velocidad Lineal', fontsize=18)
        savefig('velocidad_lineal.png')

        fig6=figure(6)
        line1,=plot(per_muestreo[:jo-1],r_angular[:jo-1],'bo',label='Velocidad Angular Calculada')
        line2,=plot(per_muestreo[:jo-1],(w_r_r[:jo-1]-w_r_l[:jo-1])*(math.pi)*(r_wheel)/(20480*d_baseline),'r',label='Velocidad Angular Real')
        grid(True)
        plt.legend(handler_map={line1: HandlerLine2D(numpoints=8)})
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad Angular (rad/s)')
        fig6.suptitle('Velocidad Angular', fontsize=18)
        savefig('velocidad_angular.png')

        plt.show()
        self.guardar_excel_careli('prueba2careli','evasion',t_out,x_out,y_out,x_d,y_d,theta_out,w_r,w_l,w_r_r,w_r_l,rapidez,r_angular,tiempo_trayectoria,per_muestreo,d_x_d,d_y_d)
        return

##Generacion de trayectorias para metodos numericos
    def trayectoria_circular(self):
        pi=math.pi
        t_s = 0.1
        v_lineal =int(self.Slider_velocidad.value())
        r = int(self.circular_radio.value())
        vueltas=self.vueltas.value()
        psi = 2*pi*vueltas
        d_psi = v_lineal/r
        t_mov=psi/d_psi
        theta_d_n_1 = 0
        punt_p = math.ceil(t_mov/t_s)
        t_p = np.linspace(0,t_s*punt_p,punt_p)
        x_d = r*cos(t_p*d_psi)
        y_d = r*sin(t_p*d_psi)
        return x_d,y_d,punt_p

    def trayectoria_cuadrada(self):
        t_s = 0.1
        v_lineal =int(self.Slider_velocidad.value())
        x_max = self.cuadrada_largo.value()
        y_max = self.cuadrada_ancho.value()
        theta_d_n_1 = 0
        vueltas=self.vueltas.value()
        esquinas=1.1

        t_mov1 = x_max/v_lineal
        punt_p1 = math.ceil(t_mov1/t_s)
        t_mov2 = y_max/v_lineal
        punt_p2 = math.ceil(t_mov2/t_s)
        t_mov = vueltas*((2*t_mov1) + (2*t_mov2))

        punt_p = vueltas*((2*punt_p1) + (2*punt_p2) + 4)

        x_d = np.zeros((punt_p/vueltas)+1)
        y_d = np.zeros((punt_p/vueltas)+1)


        x_d[0:punt_p1+1] = np.linspace(esquinas,x_max-esquinas,punt_p1+1)

        x_d[punt_p1 + 1:punt_p1 + punt_p2 + 2] = x_max
        y_d[punt_p1 + 1:punt_p1 + punt_p2 + 2] = np.linspace(esquinas,y_max-esquinas,punt_p2+1)

        x_d[punt_p1 + punt_p2 + 2:2*punt_p1 + punt_p2 + 3] = np.linspace(x_max-esquinas,esquinas,punt_p1+1)   # se debe poner un numero mas en el indice porque python excluye el ultimo numero
        y_d[punt_p1 + punt_p2 + 2:2*punt_p1 + punt_p2 + 3] = y_max

        y_d[2*punt_p1 + punt_p2 + 3:2*punt_p1 + 2*punt_p2 + 4] = np.linspace(y_max-esquinas,esquinas,punt_p2+1)

        f=vueltas
        x_d_aux=np.zeros(punt_p+vueltas)
        y_d_aux=np.zeros(punt_p+vueltas)
        aux_dim=len(x_d)


        while f > 0 :
            a=(vueltas-f)*aux_dim
            print a
            b=(vueltas-f+1)*aux_dim
            print b
            x_d_aux[a:b]=x_d
            y_d_aux[a:b]=y_d
            f=f-1

        x_d=x_d_aux
        y_d=y_d_aux
        x_d[punt_p] = x_d[1]
        return x_d,y_d,punt_p

    def trayectoria_senoidal(self):
        def integrand(x, A, k):
            return sqrt((1 + (A*k*cos(k*x))*(A*k*cos(k*x))))
        t_s = 0.1
        v_lineal =int(self.Slider_velocidad.value())
        A = self.senoidal_amplitud.value()
        lambda_ = self.senoidal_lambda.value()
        periodos=self.vueltas.value()
        theta_d_n_1 = 0
        pi=math.pi
        k = 2*pi / lambda_
        I = quad(integrand, 0, periodos*lambda_, args=(A,k))
        t_mov = I[0]/v_lineal
        punt_p = math.ceil(t_mov/t_s) + 1

        x_d_b = np.linspace(0,periodos*lambda_,punt_p)
        x_d= np.linspace(0,periodos*lambda_,punt_p+1)
        x_d[0:punt_p]=x_d_b
        x_d[punt_p]=x_d[punt_p-1]+x_d[1]
        y_d = A*sin(k*x_d)
        return x_d,y_d,punt_p

    
##Generacion de trayectorias por cinematica inversa
    def trayectoria_careli_circular(self):
        pi=math.pi
        t_s = 0.11
        v_lineal =int(self.Slider_velocidad.value())
        r = int(self.circular_radio.value())
        vueltas=self.vueltas.value()
        rampa=30
        psi = 2*pi*vueltas
        d_psi = v_lineal/r
        t_mov=psi/d_psi
        punt_p = math.ceil(t_mov/t_s)
        t_p = np.linspace(0,t_s*punt_p,punt_p)
        x_d = r*cos(t_p*d_psi)
        y_d = r*sin(t_p*d_psi)
        d_x_d = (-v_lineal)*sin(d_psi*t_p)     #componente x de la velocidad
        d_y_d = v_lineal*cos(d_psi*t_p)        #componente y de la velocidad
        d_y_d[0:rampa]=np.linspace(0,d_y_d[rampa+1],rampa)  #se agrega para dar una rampa inicial
        #genero el tiempo de muestreo para velocidades
        dim_t=len(d_x_d)
        t_traj= np.linspace(0,((punt_p+(vueltas-1))*t_s),dim_t)
        return x_d,y_d,punt_p,d_x_d,d_y_d

    def trayectoria_careli_cuadrada(self):
        t_s = 0.1
        v_lineal =int(self.Slider_velocidad.value())
        x_max = self.cuadrada_largo.value()
        y_max = self.cuadrada_ancho.value()
        
        vueltas=self.vueltas.value()
        rampa=40
        esquinas=1.1

        t_mov1 = x_max/v_lineal
        punt_p1 = math.ceil(t_mov1/t_s)
        t_mov2 = y_max/v_lineal
        punt_p2 = math.ceil(t_mov2/t_s)
        t_mov = vueltas*((2*t_mov1) + (2*t_mov2))

        punt_p = vueltas*((2*punt_p1) + (2*punt_p2) + 4)

        x_d = np.zeros((punt_p/vueltas)+1)
        y_d = np.zeros((punt_p/vueltas)+1)
        d_x_d = np.zeros((punt_p/vueltas)+1)
        d_y_d = np.zeros((punt_p/vueltas)+1)

    #Generar tryaectoria
        #primer intervalo de trayectoria

        x_d[0:punt_p1+1] = np.linspace(esquinas,x_max-esquinas,punt_p1+1)
        #y_m ya tiene los ceros correspondientes
        d_x_d[0:rampa+1]=np.linspace(0,v_lineal,rampa+1)
        d_x_d[rampa+1:punt_p1-rampa]=v_lineal
        d_x_d[punt_p1-rampa:punt_p1+1]=np.linspace(v_lineal,0,rampa+1)
        #d_y_d ya tiene los ceros correspondientes

        #Segundo intervalo de la trayectoria
        x_d[punt_p1 + 1:punt_p1 + punt_p2 + 2] = x_max
        y_d[punt_p1 + 1:punt_p1 + punt_p2 + 2] = np.linspace(esquinas,y_max-esquinas,punt_p2+1)
        #d_x_d ya tiene los ceros correspondientes
        d_y_d[punt_p1 + 1:punt_p1+1+rampa]=np.linspace(0,v_lineal,rampa)
        d_y_d[punt_p1+1+rampa:punt_p1 + punt_p2 + 2-rampa]=v_lineal
        d_y_d[punt_p1 + punt_p2 + 2-rampa:punt_p1 + punt_p2 + 2]=np.linspace(v_lineal,0,rampa)

        #Tercer intervalo de la trayectoria

        x_d[punt_p1 + punt_p2 + 2:2*punt_p1 + punt_p2 + 3] = np.linspace(x_max-esquinas,esquinas,punt_p1+1)   # se debe poner un numero mas en el indice porque python excluye el ultimo numero
        y_d[punt_p1 + punt_p2 + 2:2*punt_p1 + punt_p2 + 3] = y_max
                               #d_x_d[punt_p1 + punt_p2 + 2:2*punt_p1 + punt_p2 + 3] = -v_lineal   # sin modificacion
        d_x_d[punt_p1 + punt_p2 + 2:punt_p1 + punt_p2 + 2+rampa] =np.linspace(0,-v_lineal,rampa)
        d_x_d[punt_p1 + punt_p2 + 2+rampa:2*punt_p1 + punt_p2 + 3-rampa] =-v_lineal
        d_x_d[2*punt_p1 + punt_p2 + 3-rampa:2*punt_p1 + punt_p2 + 3] =np.linspace(-v_lineal,0,rampa)
        #d_y_d ya tiene los ceros correspondientes

        #Cuarto intervalo de la trayectoria
        #x_d ya esta completo con los ceros que faltaban
        y_d[2*punt_p1 + punt_p2 + 3:2*punt_p1 + 2*punt_p2 + 4] = np.linspace(y_max-esquinas,esquinas,punt_p2+1)
        #d_x_d ya tiene los ceros
        d_y_d[2*punt_p1 + punt_p2 + 3:2*punt_p1 + punt_p2 + 3+rampa]=np.linspace(0,-v_lineal,rampa)
        d_y_d[2*punt_p1 + punt_p2 + 3+rampa:2*punt_p1 + 2*punt_p2 + 4-rampa]=-v_lineal
        d_y_d[2*punt_p1 + 2*punt_p2 + 4-rampa:2*punt_p1 + 2*punt_p2 + 4]=np.linspace(-v_lineal,0,rampa)

        #uniendo los puntos calculados para cumplir el numero de vueltas
        f=vueltas
        x_d_aux=np.zeros(punt_p+vueltas)
        y_d_aux=np.zeros(punt_p+vueltas)
        d_x_d_aux=np.zeros(punt_p+vueltas)
        d_y_d_aux=np.zeros(punt_p+vueltas)
        aux_dim=len(x_d)


        while f > 0 :
            a=(vueltas-f)*aux_dim
            print a
            b=(vueltas-f+1)*aux_dim
            print b
            x_d_aux[a:b]=x_d
            y_d_aux[a:b]=y_d
            d_x_d_aux[a:b]=d_x_d
            d_y_d_aux[a:b]=d_y_d
            f=f-1
        #recupero los valores finales
        x_d=x_d_aux
        y_d=y_d_aux
        d_x_d=d_x_d_aux
        d_y_d=d_y_d_aux
        #genero el tiempo de muestreo para velocidades
        dim_t=len(d_x_d)
        t_traj= np.linspace(0,((punt_p+(vueltas-1))*t_s),dim_t)
        x_d[punt_p] = x_d[1]
        return x_d,y_d,punt_p,d_x_d,d_y_d

    def trayectoria_careli_senoidal(self):

        def integrand(x, A, k):
            return sqrt((1 + (A*k*cos(k*x))*(A*k*cos(k*x))))
        t_s = 0.1
        v_lineal =int(self.Slider_velocidad.value())
        A = self.senoidal_amplitud.value()
        lambda_ = self.senoidal_lambda.value()
        lambda_=600
        periodos=self.vueltas.value()
        rampa=40
        theta_d_n_1 = 0
        pi=math.pi
        k = 2*pi / lambda_
        I = quad(integrand, 0, periodos*lambda_, args=(A,k))
        t_mov = I[0]/v_lineal
        punt_p = math.ceil(t_mov/t_s) + 1

        x_d_b = np.linspace(0,periodos*lambda_,punt_p)
        x_d= np.linspace(0,periodos*lambda_,punt_p+1)
        x_d[0:punt_p]=x_d_b
        x_d[punt_p]=x_d[punt_p-1]+x_d[1]
        y_d = A*sin(k*x_d)
        #Generando las Coordenadas de la velocidad
        d_x_d = v_lineal*cos(np.arctan(A*k*cos(k*x_d)))
        d_y_d = v_lineal*sin(np.arctan(A*k*cos(k*x_d)))
        d_x_d[0:rampa]=np.linspace(0,d_x_d[rampa+1],rampa)  #modificacion rampas
        d_y_d[0:rampa]=np.linspace(0,d_y_d[rampa+1],rampa)  #modificacion rampas
        #genero tiempo de trayectoria
        dim_t=len(d_x_d)
        return x_d,y_d,punt_p,d_x_d,d_y_d
    
##Metodo para inicio de seguimiento de trayectoria
    def inicio_trayectoria(self):
        if self.radioButton_scaglia.isChecked():
            if self.checkBox_evasion.isChecked():
                print 'evasion trayectoria euler'
                self.seguimiento_trayectorias_evasion()
            else:
                print 'seguimiento trayectoria euler'
                self.seguimiento_trayectorias()
                
        elif self.radioButton_careli.isChecked():
            if self.checkBox_evasion.isChecked():
                print 'evasion trayectoria careli'
                self.seguimiento_trayectorias_careli_evasion()
            else:
                print 'seguimiento trayectoria careli'
                self.seguimiento_trayectorias_careli()
        return
    
##Metodo para almacenamiento de datos    
    def guardar_excel(self,nombre,sheet,a,b,c,d,e,f,g,h,j,k,l,m,tie,n):
        wb = xlwt.Workbook()
        ws = wb.add_sheet(sheet)
        for i, row in enumerate(a):
            ws.write(i, 0, a[i])
        for i, row in enumerate(b):
            ws.write(i, 1, b[i])
        for i, row in enumerate(c):
            ws.write(i, 2, c[i])
        for i, row in enumerate(d):
            ws.write(i, 3, d[i])
        for i, row in enumerate(e):
            ws.write(i, 4, e[i])
        for i, row in enumerate(f):
            ws.write(i, 5, f[i])
        for i, row in enumerate(g):
            ws.write(i, 6, g[i])
        for i, row in enumerate(h):
            ws.write(i, 7, h[i])
        for i, row in enumerate(j):
            ws.write(i, 8, j[i])
        for i, row in enumerate(k):
            ws.write(i, 9, k[i])
        for i, row in enumerate(l):
            ws.write(i, 10, l[i])
        for i, row in enumerate(m):
            ws.write(i, 11, m[i])
        ws.write(1,12,tie)
        for i, row in enumerate(n):
            ws.write(i, 13, n[i])
        wb.save(nombre+".xls")
        return
    
    def guardar_excel_careli(self,nombre,sheet,a,b,c,d,e,f,g,h,j,k,l,m,tie,n,o,p):
        wb = xlwt.Workbook()
        ws = wb.add_sheet(sheet)
        for i, row in enumerate(a):
            ws.write(i, 0, a[i])
        for i, row in enumerate(b):
            ws.write(i, 1, b[i])
        for i, row in enumerate(c):
            ws.write(i, 2, c[i])
        for i, row in enumerate(d):
            ws.write(i, 3, d[i])
        for i, row in enumerate(e):
            ws.write(i, 4, e[i])
        for i, row in enumerate(f):
            ws.write(i, 5, f[i])
        for i, row in enumerate(g):
            ws.write(i, 6, g[i])
        for i, row in enumerate(h):
            ws.write(i, 7, h[i])
        for i, row in enumerate(j):
            ws.write(i, 8, j[i])
        for i, row in enumerate(k):
            ws.write(i, 9, k[i])
        for i, row in enumerate(l):
            ws.write(i, 10, l[i])
        for i, row in enumerate(m):
            ws.write(i, 11, m[i])
        ws.write(1,12,tie)
        for i, row in enumerate(n):
            ws.write(i, 13, n[i])
        wb.save(nombre+".xls")
        for i, row in enumerate(o):
            ws.write(i, 14, o[i])
        for i, row in enumerate(p):
            ws.write(i, 15, p[i])
        wb.save(nombre+".xls")
        wb.save(nombre+".xls")
        return

app = QtGui.QApplication(sys.argv)
myWindow = MyWindowClass(None)
myWindow.show()
app.exec_()
