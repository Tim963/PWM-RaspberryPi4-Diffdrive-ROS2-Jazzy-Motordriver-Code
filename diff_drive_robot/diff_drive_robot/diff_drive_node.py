#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_robot')
        
        # Hardware-Pins (Anpassen!)
        self.motor_left_pwm = 18    # GPIO18 (PWM0)
        self.motor_left_in1 = 23    # GPIO23 (Richtung Vorwärts)
        self.motor_left_in2 = 24    # GPIO24 (Richtung Rückwärts)
        
        self.motor_right_pwm = 13   # GPIO13 (PWM1)
        self.motor_right_in3 = 16   # GPIO16 (Richtung Vorwärts)
        self.motor_right_in4 = 20   # GPIO20 (Richtung Rückwärts)
        
        # Pigpio initialisieren
        self.pi = pigpio.pi()
        self._setup_pins()
        
        # Kinematik-Parameter (Anpassen!)
        self.wheel_radius = 0.065   # Meter
        self.wheel_base = 0.30      # Meter
        self.max_speed = 2.0        # Max. Geschwindigkeit in m/s
        
        # Subscriber für /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
    
    def _setup_pins(self):
        # Pins als PWM-Ausgänge konfigurieren
        for pin in [self.motor_left_pwm, self.motor_right_pwm]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, 1000)  # 1000 Hz
        
        # Richtungspins
        for pin in [self.motor_left_in1, self.motor_left_in2,
                    self.motor_right_in3, self.motor_right_in4]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)  # Initial auf LOW
    
    def cmd_vel_callback(self, msg):
        # Kinematik berechnen
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Radgeschwindigkeiten
        left_speed = (linear - (angular * self.wheel_base) / 2) / self.wheel_radius
        right_speed = (linear + (angular * self.wheel_base) / 2) / self.wheel_radius
        
        # Motoren ansteuern
        self._set_motor_speed(
            self.motor_left_pwm, self.motor_left_in1, self.motor_left_in2, left_speed
        )
        self._set_motor_speed(
            self.motor_right_pwm, self.motor_right_in3, self.motor_right_in4, right_speed
        )
    
    def _set_motor_speed(self, pwm_pin, in_forward, in_backward, speed):
        # PWM-Wert berechnen (0–255)
        pwm_value = int(abs(speed) / self.max_speed * 255)
        pwm_value = max(0, min(255, pwm_value))
        
        # Richtung und PWM setzen
        if speed > 0:
            self.pi.write(in_forward, 1)
            self.pi.write(in_backward, 0)
        elif speed < 0:
            self.pi.write(in_forward, 0)
            self.pi.write(in_backward, 1)
        else:
            self.pi.write(in_forward, 0)
            self.pi.write(in_backward, 0)
        
        # PWM senden
        self.pi.set_PWM_dutycycle(pwm_pin, pwm_value)
    
    def shutdown(self):
        # Motoren stoppen
        self.pi.set_PWM_dutycycle(self.motor_left_pwm, 0)
        self.pi.set_PWM_dutycycle(self.motor_right_pwm, 0)
        for pin in [self.motor_left_in1, self.motor_left_in2,
                    self.motor_right_in3, self.motor_right_in4]:
            self.pi.write(pin, 0)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
