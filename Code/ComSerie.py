import serial

def read_adc_data(port, baud_rate=9600, timeout=1):
    try:
        # Configuración del puerto serial
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        
        print(f"Conectado al puerto {port} a un baud rate de {baud_rate}.")
        
        while True:
            try:
                temperature = ser.readline()
                if temperature:
                    # Convertimos el byte recibido a un entero, especificando el byteorder
                    temperature = int.from_bytes(temperature, byteorder='little')
                    print(f"Temperatura de referencia: {temperature}°C")

            except KeyboardInterrupt:
                break

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Conexion en serie finalizada.")

if __name__ == "__main__":
    # Cambiar esto al puerto serial correcto
    port = 'COM3'  
    read_adc_data(port)
