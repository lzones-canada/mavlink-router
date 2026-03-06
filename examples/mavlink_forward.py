import serial
import threading

def forward_data(src: serial.Serial, dst: serial.Serial, direction: str):
    """ Continuously read from src and write to dst """
    try:
        while True:
            if src.in_waiting > 0:
                data = src.read(src.in_waiting)
                dst.write(data)
                #print(f"{direction}: {data}")
    except serial.SerialException as e:
        print(f"Serial error on {direction}: {e}")
    except KeyboardInterrupt:
        print(f"Stopping {direction} forwarding...")
    finally:
        src.close()
        dst.close()

if __name__ == "__main__":
    port1 = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_A7CGn114J20-if00-port0"  # TELEM1
    #port1 = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30B19LA-if00-port0"                               # MODEM
    port2 = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_A_CAb133812-if00-port0"  # RS232-USRN520
    baudrate = 57600

    try:
        # Open both serial ports
        ser1 = serial.Serial(port1, baudrate, timeout=1)
        ser2 = serial.Serial(port2, baudrate, timeout=1)
        print(f"Full-duplex forwarding between {port1} and {port2} at {baudrate} baud")

        # Start two threads for bidirectional communication
        thread1 = threading.Thread(target=forward_data, args=(ser1, ser2, f"{port1} → {port2}"), daemon=True)
        thread2 = threading.Thread(target=forward_data, args=(ser2, ser1, f"{port2} → {port1}"), daemon=True)

        thread1.start()
        thread2.start()

        # Keep main thread alive
        while True:
            pass  

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser1.close()
        ser2.close()
