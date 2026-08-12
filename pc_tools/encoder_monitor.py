#!/usr/bin/env python3
"""
Monitor para el encoder rotativo

- Muestra la posición actual (ticks) y velocidad (RPM) en la terminal.
- Grafica en vivo el historial de posición.

Uso:
    source /opt/ros/jazzy/setup.bash
    python3 encoder_monitor.py
"""
import csv
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import para ROS2
import rclpy
from rclpy.node import Node

# Dato usado por los topicos
from std_msgs.msg import Int32, Float32

CSV_PATH = "encoder_log.csv"
HISTORY_LEN = 500  # cantidad de puntos visibles en la curva


class EncoderMonitor(Node):
    def __init__(self):
        super().__init__('encoder_monitor')
        # Crea la create_subscription al topicp encoder y su buffer
        self.create_subscription(Int32, '/encoder_ticks', self.ticks_cb, 10)
        self.create_subscription(Float32, '/encoder_rpm', self.rpm_cb, 10)

        # Guarda su historial y lo muestra usando matplotlib 
        self.last_rpm = 0.0
        self.t0 = time.time()
        self.time_hist = deque(maxlen=HISTORY_LEN)
        self.pos_hist = deque(maxlen=HISTORY_LEN)

        self.csv_file = open(CSV_PATH, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'ticks', 'rpm'])
        self.get_logger().info(f"Logueando a {CSV_PATH}")

    def rpm_cb(self, msg):
        self.last_rpm = msg.data

    def ticks_cb(self, msg):
        now = time.time()
        elapsed = now - self.t0

        print(f"[{elapsed:7.2f}s] Posicion: {msg.data:6d} ticks | Velocidad: {self.last_rpm:7.2f} RPM")

        self.time_hist.append(elapsed)
        self.pos_hist.append(msg.data)

        self.csv_writer.writerow([f"{now:.3f}", msg.data, f"{self.last_rpm:.3f}"])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderMonitor()

    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Posicion (ticks)")
    ax.set_title("Historial de posicion del encoder")

    def update_plot(_frame):
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.time_hist:
            line.set_data(node.time_hist, node.pos_hist)
            ax.relim()
            ax.autoscale_view()
        return line,

    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
