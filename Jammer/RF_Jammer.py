#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.5.1

from gnuradio import analog
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time




class RF_Jammer(gr.top_block):

    def __init__(self, noise_amp=10, tx_gain=50):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)

        ##################################################
        # Parameters
        ##################################################
        self.noise_amp = noise_amp
        self.tx_gain = tx_gain

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 1e6
        self.Center_Frq = Center_Frq = 915e6

        ##################################################
        # Blocks
        ##################################################

        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(('', '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        # No synchronization enforced.

        self.uhd_usrp_sink_0.set_center_freq(Center_Frq, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_gain(tx_gain, 0)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, noise_amp, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.uhd_usrp_sink_0, 0))


    def get_noise_amp(self):
        return self.noise_amp

    def set_noise_amp(self, noise_amp):
        self.noise_amp = noise_amp
        self.analog_noise_source_x_0.set_amplitude(self.noise_amp)

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_Center_Frq(self):
        return self.Center_Frq

    def set_Center_Frq(self, Center_Frq):
        self.Center_Frq = Center_Frq
        self.uhd_usrp_sink_0.set_center_freq(self.Center_Frq, 0)



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "-t", "--noise-amp", dest="noise_amp", type=eng_float, default=eng_notation.num_to_str(float(10)),
        help="Set Noise Amplitude [default=%(default)r]")
    parser.add_argument(
        "-g", "--tx-gain", dest="tx_gain", type=eng_float, default=eng_notation.num_to_str(float(50)),
        help="Set Transmit Gain [default=%(default)r]")
    return parser


def main(top_block_cls=RF_Jammer, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(noise_amp=options.noise_amp, tx_gain=options.tx_gain)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    tb.wait()


if __name__ == '__main__':
    main()
