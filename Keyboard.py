import serial
import time
import keyboard  # Install this library using `pip install keyboard`

# Configure the serial port
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port if necessary
time.sleep(2)  # Wait for the connection to establish

def send_command(speed, angle):
    """Send motor speed and servo angle to Arduino."""
    command = f"{speed},{angle}\n"
    arduino.write(command.encode())
    time.sleep(0.1)  # Small delay to ensure Arduino processes it

def main():
    """Control the robot using real-time keyboard input."""
    speed = 0
    angle = 90  # Neutral position for the servo

    print("Control the robot using the following keys:\n")
    print("W: Move Forward")
    print("S: Move Backward")
    print("A: Turn Left")
    print("D: Turn Right")
    print("Q: Stop")
    print("ESC: Exit")

    try:
        while True:
            # Check for real-time key presses
            if keyboard.is_pressed('w'):
                speed = -40  # Move forward
                print("Moving Forward")
            elif keyboard.is_pressed('s'):
                speed = 40  # Move backward
                print("Moving Backward")
            elif keyboard.is_pressed('a'):
                angle = min(180, angle + 10)  # Turn left
                print("Turning Left")
            elif keyboard.is_pressed('d'):
                angle = max(0, angle - 10)  # Turn right
                print("Turning Right")
            elif keyboard.is_pressed('q'):
                speed = 0  # Stop movement
                print("Stopping")
            elif keyboard.is_pressed('esc'):  # Exit program
                print("Exiting Program")
                break

            # Continuously send the current speed and angle
            send_command(speed, angle)
            time.sleep(0.05)  # Small delay to avoid excessive processing

    except KeyboardInterrupt:
        print("Program interrupted.")

    finally:
        # Stop the robot and close the serial connection
        send_command(0, 90)
        arduino.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
