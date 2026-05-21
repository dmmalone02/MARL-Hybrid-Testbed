import time
import argparse
import xmlrpc.client


def get_proxy(node_id, port=8080):
    ip = f"10.1.1.{node_id}"
    return xmlrpc.client.ServerProxy(f"http://{ip}:{port}")


def trigger_optical(tx_node):
    tx = get_proxy(tx_node)

    tx.set_variable_pb(1)
    time.sleep(0.5)
    tx.set_variable_pb(0)
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--tx_node", type=int, required=True)
    parser.add_argument("-r", "--rx_node", type=int, required=True)
    parser.add_argument("-e","--ep", type=int, required=True)
    args = parser.parse_args()

    rx_fname = f"/home/ucanlab/Downloads/Ep_{args.ep}_Node_{args.tx_node}.txt"
    dummy_fname = "/home/ucanlab/Downloads/Ep_dummy.txt"

    rx = get_proxy(args.rx_node)

    print(f"Setting Rx file to {rx_fname}")
    rx.set_filename(rx_fname)
    time.sleep(2)

    print(f"Triggering Optical Tx node {args.tx_node}")
    trigger_optical(args.tx_node)

    time.sleep(3)

    print(f"Switching Rx file to dummy file")
    rx.set_filename(dummy_fname)
    time.sleep(2)

    print("Done")


if __name__ == "__main__":
    main()
