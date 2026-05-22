from xmlrpc.client import ServerProxy
import time
import argparse
import random

RF = 0
OWC = 1


def trigger(tx, hold):
    tx.set_variable_pb(1)
    time.sleep(hold)
    tx.set_variable_pb(0)


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
    rf_press_button(tx)
    
    # wait for transmission and buffers to flush
    time.sleep(flush_wait)


def run_rf(tx, iterations):
    tx.set_selector(1)

    for i in range(iterations):
        print(f"RF tx_iter={i+1}")

        trigger_rf(tx, flush_wait=4.0)


def select_channel(x_rf, x_ow, beta):
    if x_rf < x_ow:
        c_pref = RF
    else:
        c_pref = OWC

    explore = random.random() < beta

    if explore:
        c_sel = OWC if c_pref == RF else RF
    else:
        c_sel = c_pref

    return c_pref, c_sel, explore


def update_stat(x_old, alpha, outcome):
    return (1 - alpha) * x_old + alpha * outcome
    
def wait_for_rx_eval(rx_host, retries=15, delay=1.0):
    for attempt in range(retries):
        try:
            rx_eval = ServerProxy(f"http://{rx_host}:8082/")
            status = rx_eval.ping()
            print(f"RX evaluator status: {status}")
            return
        except Exception as e:
            print(f"RX evaluator not ready on attempt {attempt+1}/{retries}: {e}")
            time.sleep(delay)

    raise RuntimeError(f"Unable to access PER evaluator at {rx_host}:8082")

def get_episode_per(rx_host, fname, iterations, retries=10, delay=1.0):
    for attempt in range(retries):
        try:
            rx_eval = ServerProxy(f"http://{rx_host}:8082/")
            return rx_eval.compute_per(fname, iterations)
        except Exception as e:
            print(f"compute_per failed on attempt {attempt+1}/{retries}: {e}")
            time.sleep(delay)

    raise RuntimeError(f"Unable to get PER from RX evaluator at {rx_host}:8082")

def print_results_table(results):
    print("\n=== RL Results Table ===")

    if not results:
        print("No results stored.")
        return

    header = (
        f"{'Instance(i)':<12}"
        f"{'X_RF':<10}"
        f"{'X_OW':<10}"
        f"{'C_pref':<10}"
        f"{'Explore':<10}"
        f"{'C_sel':<10}"
        f"{'PER':<10}"
        f"{'PSR':<10}"
    )
    print(header)
    print("-" * len(header))

    for row in results:
        print(
            f"{row['Instance']:<12}"
            f"{row['X_RF']:<10.3f}"
            f"{row['X_OW']:<10.3f}"
            f"{row['C_pref']:<10}"
            f"{row['Explore']:<10}"
            f"{row['C_sel']:<10}"
            f"{row['PER']:<10.3f}"
            f"{row['PSR']:<10.3f}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)
    parser.add_argument("--tx_node", type=int, default=183)
    parser.add_argument("--rx_node", type=int, default=182)
    parser.add_argument("--beta", type=float, default=0.2)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--x_rf_init", type=float, default=0.5)
    parser.add_argument("--x_ow_init", type=float, default=0.7)

    args = parser.parse_args()

    tx = ServerProxy(f"http://10.1.1.{args.tx_node}:8080/")
    rx = ServerProxy(f"http://10.1.1.{args.rx_node}:8081/")
    rx_host = f"10.1.1.{args.rx_node}"

    wait_for_rx_eval(rx_host)

    x_rf = args.x_rf_init
    x_ow = args.x_ow_init

    results = []

    print("=== RL Experiment Start ===")
    print(f"Episodes   : {args.episodes}")
    print(f"Iterations : {args.iterations}")
    print(f"RX Node    : {args.rx_node}")
    print(f"alpha      : {args.alpha}")
    print(f"beta       : {args.beta}")
    print(f"x_rf_init  : {args.x_rf_init}")
    print(f"x_ow_init  : {args.x_ow_init}")

    for ep in range(1, args.episodes + 1):
        print(f"\n=== Episode {ep} ===")
        print(f"Current stats before selection: x_rf={x_rf:.4f}, x_ow={x_ow:.4f}")

        x_rf_before = x_rf
        x_ow_before = x_ow

        c_pref, c_sel, explore = select_channel(x_rf, x_ow, args.beta)

        pref_name = "RF" if c_pref == RF else "OWC"
        sel_name = "RF" if c_sel == RF else "OWC"

        print(f"Preferred channel: {pref_name}")
        print(f"Selected channel : {sel_name}")
        print(f"Explore          : {explore}")

        fname = f"/home/ucanlab/Downloads/Ep_{ep}.txt"
        print(f"RX filename set to: {fname}")
        rx.set_filename(fname)
        time.sleep(1.0)

        if c_sel == RF:
            print("Running RF episode...")
            run_rf(tx, args.iterations)
        else:
            print("Running OWC episode...")
            run_owc(tx, args.iterations)

        dummy_fname = "/home/ucanlab/Downloads/Ep_dummy.txt"
        print(f"Switching RX to dummy file: {dummy_fname}")
        rx.set_filename(dummy_fname)
        time.sleep(5.0)

        outcome = get_episode_per(rx_host, fname, args.iterations)

        per = outcome
        psr = 1.0 - per

        print(f"Episode PER: {per:.4f}")
        print(f"Episode PSR: {psr:.4f}")

        if c_sel == RF:
            x_rf = update_stat(x_rf, args.alpha, outcome)
            print(f"Updated x_rf -> {x_rf:.4f}")
        else:
            x_ow = update_stat(x_ow, args.alpha, outcome)
            print(f"Updated x_ow -> {x_ow:.4f}")

        results.append({
            "Instance": ep - 1,
            "X_RF": x_rf_before,
            "X_OW": x_ow_before,
            "C_pref": c_pref,
            "Explore": int(explore),
            "C_sel": c_sel,
            "PER": per,
            "PSR": psr,
        })

    print_results_table(results)

    avg_per = sum(row["PER"] for row in results) / len(results) if results else 0.0

    print("\n=== Final Summary ===")
    print(f"Final x_rf      : {x_rf:.4f}")
    print(f"Final x_ow      : {x_ow:.4f}")
    print(f"Average EP PER  : {avg_per:.4f}")
    print(f"Number of rows  : {len(results)}")


if __name__ == "__main__":
    main()
