import time
import xmlrpc.client
import argparse
import os

def get_proxy(node_id, port=8080):
    ip = f"10.1.1.{node_id}"
    return xmlrpc.client.ServerProxy(f"http://{ip}:{port}")

def trigger_tx(tx_node):
    tx = get_proxy(tx_node)
    tx.set_variable_pb(1)
    time.sleep(0.05)
    tx.set_variable_pb(0)
    time.sleep(0.05)
    tx.set_variable_pb(1)
    time.sleep(0.05)
    tx.set_variable_pb(0)
    time.sleep(0.05)

def switch_rx_file(rx_node, filename):
    rx = get_proxy(rx_node)
    rx.set_filename(filename)
    time.sleep(2)

def run_episode(tx_node, rx_node, ep):
    rx_fname = f"/home/ucanlab/Mission_Leader/Sensing_Files/Ep_{ep}_Node_{tx_node}.txt"
    dummy_fname = "/home/ucanlab/Mission_Leader/Sensing_Files/Ep_dummy.txt"

    rx = get_proxy(rx_node)

    print(f"Setting Rx File to {rx_fname}")
    rx.set_filename(rx_fname)
    time.sleep(2)

    print(f"Triggering Tx node {tx_node}")
    trigger_tx(tx_node)

    time.sleep(5)

    print("Switching file to dummy file")
    rx.set_filename(dummy_fname)
    time.sleep(5)

    print("Switching RX back to original file")
   # rx.set_filename(rx_fname)
    time.sleep(2)

    print("Episode Complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--tx_node", type=int, required=True)
    parser.add_argument("--rx_node", type=int, required=True)
    parser.add_argument("--ep", type=int, default=1)

    args = parser.parse_args()

    run_episode(
        tx_node=args.tx_node,
        rx_node=args.rx_node,
        ep=args.ep
    )
