#!/usr/bin/env python

import rclpy
import signal
import sys

from ros_tcp_endpoint import TcpServer

def signal_handler(sig, frame):
    print('Shutting down...')
    tcp_server.destroy_nodes()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    global tcp_server
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    # 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    tcp_server.start()
    tcp_server.setup_executor()

if __name__ == "__main__":
    main()
