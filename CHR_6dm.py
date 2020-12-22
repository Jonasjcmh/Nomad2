class AHRS(object):                                                             #Clase para manejo del CHR_6dm
    ##Inicializacion de Constantes
    INICIO_PACKET = 'snp'   #Inicio de un nuevo paquete
    FACTOR_ANG = 0.0109863
    ##Packet Type Indicator
    SILENT_MODE = chr(0x81) 
    ACTIVE_CHANNEL = chr(0x80)
    AUTO_MAG_REF = chr(0x91)
    GET_DATA = chr(0x01) 
    
    COMMAND_COMPLETE = chr(0xB0)   
    COMMAND_FAILED = chr(0xB1) 
    BAD_CHECKSUM = chr(0xB2)
    BAD_DATA_LENGTH = chr(0xB3)
    UNRECOGNIZED_PACKET = chr(0xB4)
    MAG_REF_REPORT = chr(0xC5)
    SENSOR_REPORT = chr(0xB7)
    
    ##Constructor
    def __init__(self, port):
        self.__port = port  #Variable del puerto serial y su Lock
        
        ##Variables para Datos del Sensor
        self.__yaw, self.__pitch, self.__roll = 0,0,0
    
    ##Envio de Comandos al CHR_6dm, retorno de datos leidos
    def __execute_command(self, command, N_c):
        self.__port.write(AHRS.INICIO_PACKET)    #Inicio Paquete
        self.__port.write(command)  #PT correspondiente
        self.__port.write(N_c)    #Numero de datos correspondiente
        
        if (ord(N_c) > 0):   #Condicion para enviar los datos para activar los canales
            D_1 = chr(0xE0) #Activando el canal del yaw,pitch y roll
            self.__port.write(D_1)  #Enviando el byte
            D_2 = chr(0x00)
            self.__port.write(D_2)
        else:
            D_1 = chr(0x00) #Para calcular el checksum cuando no se envian datos
            D_2 = chr(0x00)
        
        check_sum = bin(sum(bytearray(AHRS.INICIO_PACKET + command + N_c + D_1 + D_2)))  #Valor Binario del checksum
        self.__port.write(chr(int(check_sum[2:-8],2)))  #MSB del checksum 
        self.__port.write(chr(int(check_sum[-8:],2)))   #LSB del checksum
            
        ##Ubicacion del inicio de paquete 'snp'
        result = self.__port.read(1)    #Leo el buffer buscando 's'
        while (result != 's'):    
            result = self.__port.read(1)    #Leo hasta encontrar 's'
        result += self.__port.read(4)    #Leyendo el resto de la respuesta hasta N
        
        
##        print result    #imprimir la respuesta
##        for i in range(1,6):    #Imprimir la respuesta en hexadecimal
##            print hex(ord(result[i-1]))
        
        
        PT = result[3]  #Obteniendo el PT de respuesta
        if (PT == AHRS.SENSOR_REPORT):  #Angulos de Euler 
            datos = self.__port.read(10)   #Leyendo los 10 bytes 2 informacion + 6 datos + 2 checksum
            
            
##            print datos    #imprimir la respuesta
##            for i in range(1,11):    #Imprimir la respuesta en hexadecimal
##                print hex(ord(datos[i-1]))
            
            
            datos = datos[2:]   #Tomando solo los datos
            data_s = [0,0,0]  #Inicilizar una lista con 3 valores
            j = 1
            
            for i in range(1, 7, 2):
                d1 = ord(datos[i-1]) << 8 #Tomando el MSB y desplazando a la izquierda (multiplicacion)
                #print bin(d1)
                d2 = ord(datos[i])   #Tomando el LSB
                #print bin(d2)
                d1 = d1 + d2
                #print d1
                
                ##Determinando el Complemento a 2
                prub = d1 >> 15 #Buscamos el bit 16
                
                if (prub == 1): #El bit 16 es signo (1L == (-))
                    data_s[j-1] = d1 - (1 << 16)    #Regresando del complemento a 2
                else:
                    data_s[j-1] = d1    #Cargando el valor cuando es positivo 
                j += 1
            return data_s
        
        elif (PT == AHRS.MAG_REF_REPORT):   #Vector de Ref. Magnetometro  
            magneto = self.__port.read(8)   #Leyendo los 6 bytes de datos y 2 de checksum
            data_mag = [0,0,0]  #Inicilizar una lista con 3 valores
            j = 1
            
            for i in range(1, 7, 2):
                d1 = ord(magneto[i-1]) << 8 #Tomando el MSB y desplazando a la izquierda (multiplicando)
                d2 = ord(magneto[i])   #Tomando el LSB
                d1 = d1 + d2
                #print d1
                
                ##Determinando el Complemento a 2
                prub_m = d1 >> 15 #Buscamos el bit 16
                
                if (prub_m == 1): #El bit 16 es signo (1L == (-))
                    data_mag[j-1] = d1 - (1 << 16)    #Regresando del complemento a 2
                else:
                    data_mag[j-1] = d1    #Cargando el valor cuando es positivo 
                j += 1
            return data_mag
        
        elif (PT == AHRS.COMMAND_COMPLETE):   #El comando ha sido recibido correctamente
            r_comm = self.__port.read(3)    #Leer el comando de respuesta y el checksum
            #print hex(ord(r_comm[0]))    
            print 'Comando Completado'
        elif (PT == AHRS.COMMAND_FAILED):
            print 'Comando Fallido'
        elif (PT == AHRS.BAD_CHECKSUM):
            print 'Error Checksum'
        elif (PT == AHRS.BAD_DATA_LENGTH):
            print 'Error Longitud de Datos'
        elif (PT == AHRS.UNRECOGNIZED_PACKET):
            print 'Paquete No-reconocible'
    
    ##Metodos para envio de comandos especificos
    def set_silent(self):
        N_command = chr(0x0) #Cero bytes enviados (comandos)
        self.__execute_command(AHRS.SILENT_MODE, N_command)
    
    def set_channel(self):
        N_command = chr(0x02) #Dos bytes enviados (comandos)
        self.__execute_command(AHRS.ACTIVE_CHANNEL, N_command)
    
    def set_mag_ref(self):
        N_command = chr(0x0)    #Cero bytes enviados (comandos)
        return self.__execute_command(AHRS.AUTO_MAG_REF, N_command)
    
    def sensor_data(self):
        N_command = chr(0x0)    #Cero bytes enviados (comandos)
        angulos = self.__execute_command(AHRS.GET_DATA, N_command) #Recupero la lista
        yaw = angulos[0]*AHRS.FACTOR_ANG
        pitch = angulos[1]*AHRS.FACTOR_ANG
        roll = angulos[2]*AHRS.FACTOR_ANG
        return yaw, pitch, roll