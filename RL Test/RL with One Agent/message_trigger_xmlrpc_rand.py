import time
import argparse
import random
from xmlrpc.client import ServerProxy

def trigger(tx, hold):
    tx.set_variable_pb(1)
    time.sleep(hold)
    tx.set_variable_pb(0)
    time.sleep(hold)
    
def run_owc(tx, iterations):
    tx.set_selector(0)
    
    for i in range(iterations):
        print(f"OWC tx_iter={i+1}")
        
        trigger(tx, 0.5)
        
        time.sleep(1)
        
def rf_press_button(tx, pulse=0.1):
    tx.set_variable_pb(1)
    time.sleep(pulse)
    tx.set_variable_pb(0)
    time.sleep(pulse)
        
def trigger_rf(tx, flush_wait=7.0):
    # RF needs two quick button presses
    rf_press_button(tx)
    
    # wait for transmission and buffers to flush
    time.sleep(flush_wait)


def run_rf(tx, iterations):
    tx.set_selector(1)
    rf_packet_count = 0

    for i in range(iterations):
        print(f"RF tx_iter={i+1}")

        trigger_rf(tx, flush_wait=4.0)
        
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--p_rf", type=float, required=True)
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--node", type=int, default=182)
    
    args = parser.parse_args()
    
    tx = ServerProxy("http://10.1.1.183:8080/")
    rx = ServerProxy(f"http://10.1.1.{args.node}:8081/")
    
    for ep in range(1, args.episodes + 1):
        print(f"\n=== Episode {ep} ===")
        
        rx.set_filename(f"/home/ucanlab/Downloads/Ep_{ep}.txt")
        
        Sc = 1 if random.random() < args.p_rf else 0
        
        if Sc == 0:
            print("OWC Episode")
            run_owc(tx, args.iterations)
        else:
            print("RF Episode")
            run_rf(tx, args.iterations)

if __name__ == "__main__":
    main()
