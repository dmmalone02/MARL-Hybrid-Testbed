#!/usr/bin/env python
from xmlrpc.server import SimpleXMLRPCServer
import os
import time


def compute_per(fname, n_tx):
    try:
        with open(fname, "r") as f:
            raw_lines = [line.rstrip() for line in f if line.strip()]
    except Exception as e:
        print(f"[RX_EVAL] read error for {fname}: {e}")
    time.sleep(0.5)

    print(f"[RX_EVAL] file={fname}")
    per = (n_tx - len(raw_lines)) / n_tx
    print(f"[RX_EVAL] lines={len(raw_lines)} per={per}")
    return per


def ping():
    return "rx_file_evaluator_alive"


def main():
    host = "0.0.0.0"
    port = 8082

    server = SimpleXMLRPCServer((host, port), allow_none=True, logRequests=False)
    server.register_function(compute_per, "compute_per")
    server.register_function(ping, "ping")

    print(f"RX file evaluator running on {host}:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
