#!/usr/bin/env python3
import argparse
import collections
import collections.abc
import select
import socket
import time

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(description="Forward MAVLink telemetry from SITL TCP to UDP listeners")
    parser.add_argument("--master", default="tcp:127.0.0.1:5760")
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=14550)
    parser.add_argument("--autopilot-udp-port", type=int, default=14551)
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=5762)
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.udp_host, 0))
    sock.setblocking(False)

    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind((args.tcp_host, args.tcp_port))
    tcp_server.listen(1)
    tcp_server.setblocking(False)

    tcp_client = None
    targets = [
        (args.udp_host, args.udp_port),
        (args.udp_host, args.autopilot_udp_port),
    ]

    while True:
        try:
            master = mavutil.mavlink_connection(args.master, autoreconnect=True, robust_parsing=True)
            master.wait_heartbeat(timeout=30)
            while True:
                try:
                    readers = [sock, tcp_server]
                    if tcp_client is not None:
                        readers.append(tcp_client)
                    ready, _, _ = select.select(readers, [], [], 0.02)
                except (ValueError, OSError):
                    ready = []

                if ready:
                    if tcp_server in ready:
                        try:
                            client, _ = tcp_server.accept()
                            client.setblocking(False)
                            if tcp_client is not None:
                                try:
                                    tcp_client.close()
                                except OSError:
                                    pass
                            tcp_client = client
                        except OSError:
                            pass

                    try:
                        data, addr = sock.recvfrom(65535)
                    except (BlockingIOError, ConnectionResetError, OSError):
                        data = None
                        addr = None

                    if data and addr:
                        # Any client packet is forwarded back to SITL.
                        master.write(data)

                    if tcp_client is not None and tcp_client in ready:
                        try:
                            tcp_data = tcp_client.recv(65535)
                        except (BlockingIOError, ConnectionResetError, OSError):
                            tcp_data = b""

                        if tcp_data:
                            master.write(tcp_data)
                        else:
                            try:
                                tcp_client.close()
                            except OSError:
                                pass
                            tcp_client = None

                msg = master.recv_msg()
                if msg is None:
                    continue
                payload = msg.get_msgbuf()
                if payload:
                    for target in targets:
                        sock.sendto(payload, target)
                    if tcp_client is not None:
                        try:
                            tcp_client.sendall(payload)
                        except OSError:
                            try:
                                tcp_client.close()
                            except OSError:
                                pass
                            tcp_client = None
        except KeyboardInterrupt:
            break
        except Exception:
            time.sleep(1.0)


if __name__ == "__main__":
    main()