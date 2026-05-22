from xmlrpc.client import ServerProxy
import time
import argparse
import random

RF = 0
OWC = 1


def rf_press_button(tx, pulse=0.05):
    tx.set_variable_pb(1)
    time.sleep(pulse)
    tx.set_variable_pb(0)
    time.sleep(pulse)


def trigger_rf(tx, mode):
    if mode == "double":
        rf_press_button(tx)
        rf_press_button(tx)
    elif mode == "single":
        rf_press_button(tx)
    else:
        raise ValueError(f"Unknown RF trigger mode: {mode}")

    time.sleep(7.0)


def trigger_owc(tx, hold=0.5):
    tx.set_variable_pb(1)
    time.sleep(hold)
    tx.set_variable_pb(0)


def run_rf(tx, tx_node, trigger_mode, iterations):
    print(f"Setting node {tx_node} to RF")

    tx.set_selector(1)
    time.sleep(0.5)

    for i in range(iterations):
        print(f"RF tx_iter={i + 1} on node {tx_node}, mode={trigger_mode}")
        trigger_rf(tx, trigger_mode)


def run_owc(tx, tx_node, iterations):
    print(f"Setting node {tx_node} to OWC")

    tx.set_selector(0)
    time.sleep(0.05)

    for i in range(iterations):
        print(f"OWC tx_iter={i + 1} on node {tx_node}")
        trigger_owc(tx)
        time.sleep(1)


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


def get_episode_per(rx_host, fname, iterations):
    rx_eval = ServerProxy(f"http://{rx_host}:8082/")
    return rx_eval.compute_per(fname, iterations)


def print_results_table(results):
    print("\n=== RL Results Table ===")

    header = (
        f"{'Instance':<10}"
        f"{'Episode':<10}"
        f"{'TX_Node':<10}"
        f"{'Mode':<10}"
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

    for r in results:
        print(
            f"{r['Instance']:<10}"
            f"{r['Episode']:<10}"
            f"{r['TX_Node']:<10}"
            f"{r['Mode']:<10}"
            f"{r['X_RF']:<10.3f}"
            f"{r['X_OW']:<10.3f}"
            f"{r['C_pref']:<10}"
            f"{r['Explore']:<10}"
            f"{r['C_sel']:<10}"
            f"{r['PER']:<10.3f}"
            f"{r['PSR']:<10.3f}"
        )


def run_tx_episode(tx, tx_node, trigger_mode, rx, rx_host, ep, args, x_rf, x_ow, results):
    print(f"\n=== Episode {ep}, TX Node {tx_node} ===")

    x_rf_before = x_rf
    x_ow_before = x_ow

    c_pref, c_sel, explore = select_channel(x_rf, x_ow, args.beta)

    fname = f"/home/ucanlab/Downloads/Ep_{ep}_Node_{tx_node}.txt"

    print(f"Writing RX output to {fname}")
    rx.set_filename(fname)
    time.sleep(1)

    if c_sel == RF:
        run_rf(tx, tx_node, trigger_mode, args.iterations)
    else:
        run_owc(tx, tx_node, args.iterations)

    print("Switching RX to dummy file")
    rx.set_filename("/home/ucanlab/Downloads/Ep_dummy.txt")
    time.sleep(5)

    per = get_episode_per(rx_host, fname, args.iterations)
    psr = 1 - per

    if c_sel == RF:
        x_rf = update_stat(x_rf, args.alpha, per)
    else:
        x_ow = update_stat(x_ow, args.alpha, per)

    results.append({
        "Instance": len(results),
        "Episode": ep,
        "TX_Node": tx_node,
        "Mode": trigger_mode,
        "X_RF": x_rf,
        "X_OW": x_ow,
        "C_pref": c_pref,
        "Explore": int(explore),
        "C_sel": c_sel,
        "PER": per,
        "PSR": psr
    })

    print_results_table(results)

    return x_rf, x_ow


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--iterations", type=int, required=True)

    parser.add_argument("--tx_nodes", type=int, nargs="+", required=True)
    parser.add_argument(
        "--rf_trigger_modes",
        type=str,
        nargs="+",
        required=True,
        choices=["single", "double"]
    )

    parser.add_argument("--rx_node", type=int, default=182)
    parser.add_argument("--beta", type=float, default=0.2)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--x_rf_init", type=float, default=0.5)
    parser.add_argument("--x_ow_init", type=float, default=0.7)

    args = parser.parse_args()

    if len(args.tx_nodes) != len(args.rf_trigger_modes):
        raise ValueError("Number of TX nodes must match number of RF trigger modes")

    rx_host = f"10.1.1.{args.rx_node}"
    rx = ServerProxy(f"http://{rx_host}:8081/")

    tx_sequence = []

    for tx_node, mode in zip(args.tx_nodes, args.rf_trigger_modes):
        tx_host = f"10.1.1.{tx_node}"
        tx = ServerProxy(f"http://{tx_host}:8080/")
        tx_sequence.append((tx_node, tx, mode))

    x_rf = args.x_rf_init
    x_ow = args.x_ow_init

    results = []

    for ep in range(1, args.episodes + 1):
        print(f"\n========== Starting Episode {ep} ==========")

        for tx_node, tx, trigger_mode in tx_sequence:
            x_rf, x_ow = run_tx_episode(
                tx=tx,
                tx_node=tx_node,
                trigger_mode=trigger_mode,
                rx=rx,
                rx_host=rx_host,
                ep=ep,
                args=args,
                x_rf=x_rf,
                x_ow=x_ow,
                results=results
            )

    print("\nExperiment complete.")
    print_results_table(results)


if __name__ == "__main__":
    main()
