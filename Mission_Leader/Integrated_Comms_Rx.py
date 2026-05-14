#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.1.1

from gnuradio import blocks
from gnuradio import digital
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
from xmlrpc.server import SimpleXMLRPCServer
import threading




class Integrated_Comms_Rx(gr.top_block):

    def __init__(self, node_id='100'):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)

        ##################################################
        # Parameters
        ##################################################
        self.node_id = node_id

        ##################################################
        # Variables
        ##################################################
        self.carrier_list = carrier_list = (list(range(-12, -2)))
        self.payload_mod = payload_mod = digital.constellation_qpsk()
        self.occupied_carriers = occupied_carriers = (carrier_list,)
        self.len_tag_key = len_tag_key = "packet_len"
        self.header_mod = header_mod = digital.constellation_bpsk()
        self.fft_len = fft_len = 64
        self.carrier_list_rf = carrier_list_rf = (list(range(-7, -2)) + list(range(3, 8)))
        self.sync_word2 = sync_word2 = [0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 0, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 0, 0, 0, 0, 0]
        self.sync_word1 = sync_word1 = [0., 0., 0., 0., 0., 0., 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 0., 0., 0., 0., 0.]
        self.samp_rate = samp_rate = 500000
        self.rolloff = rolloff = 0
        self.pilot_syms_rf = pilot_syms_rf = ((-1,1,-1,1),)
        self.pilot_symbols = pilot_symbols = ((-1, 1,),)
        self.pilot_carriers_rf = pilot_carriers_rf = ((-10,-2,2,10),)
        self.pilot_carriers = pilot_carriers = ((-28, -1),)
        self.packet_len = packet_len = 18
        self.occupied_carriers_rf = occupied_carriers_rf = ((carrier_list_rf),)
        self.header_ph = header_ph = digital.packet_header_ofdm(occupied_carriers, 1, len_tag_key, "frame_"+len_tag_key, bits_per_header_sym=header_mod.bits_per_symbol(), bits_per_payload_sym=payload_mod.bits_per_symbol(), scramble_header=False)
        self.filename = filename = "/home/ucanlab/Mission_Leader/Sensing_Files/Ep"
        self.CP_len = CP_len = fft_len//4

        ##################################################
        # Blocks
        ##################################################
        self.xmlrpc_server_0 = SimpleXMLRPCServer(("10.1.1." + node_id, 8080), allow_none=True)
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_server_0_thread = threading.Thread(target=self.xmlrpc_server_0.serve_forever)
        self.xmlrpc_server_0_thread.daemon = True
        self.xmlrpc_server_0_thread.start()
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_samp_rate(1e6)
        # No synchronization enforced.

        self.uhd_usrp_source_0.set_center_freq(915e6, 0)
        self.uhd_usrp_source_0.set_antenna("RX2", 0)
        self.uhd_usrp_source_0.set_gain(70, 0)
        self.digital_ofdm_rx_0_0 = digital.ofdm_rx(
            fft_len=fft_len, cp_len=CP_len,
            frame_length_tag_key='frame_'+len_tag_key,
            packet_length_tag_key=len_tag_key,
            occupied_carriers=occupied_carriers_rf,
            pilot_carriers=pilot_carriers_rf,
            pilot_symbols=pilot_syms_rf,
            sync_word1=sync_word1,
            sync_word2=sync_word2,
            bps_header=1,
            bps_payload=2,
            debug_log=False,
            scramble_bits=False)
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_char*1, filename, False)
        self.blocks_file_sink_0_0.set_unbuffered(True)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.digital_ofdm_rx_0_0, 0), (self.blocks_file_sink_0_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.digital_ofdm_rx_0_0, 0))


    def get_node_id(self):
        return self.node_id

    def set_node_id(self, node_id):
        self.node_id = node_id

    def get_carrier_list(self):
        return self.carrier_list

    def set_carrier_list(self, carrier_list):
        self.carrier_list = carrier_list
        self.set_occupied_carriers((self.carrier_list,))

    def get_payload_mod(self):
        return self.payload_mod

    def set_payload_mod(self, payload_mod):
        self.payload_mod = payload_mod

    def get_occupied_carriers(self):
        return self.occupied_carriers

    def set_occupied_carriers(self, occupied_carriers):
        self.occupied_carriers = occupied_carriers
        self.set_header_ph(digital.packet_header_ofdm(self.occupied_carriers, 1, self.len_tag_key, "frame_"+self.len_tag_key, bits_per_header_sym=header_mod.bits_per_symbol(), bits_per_payload_sym=payload_mod.bits_per_symbol(), scramble_header=False))

    def get_len_tag_key(self):
        return self.len_tag_key

    def set_len_tag_key(self, len_tag_key):
        self.len_tag_key = len_tag_key
        self.set_header_ph(digital.packet_header_ofdm(self.occupied_carriers, 1, self.len_tag_key, "frame_"+self.len_tag_key, bits_per_header_sym=header_mod.bits_per_symbol(), bits_per_payload_sym=payload_mod.bits_per_symbol(), scramble_header=False))

    def get_header_mod(self):
        return self.header_mod

    def set_header_mod(self, header_mod):
        self.header_mod = header_mod

    def get_fft_len(self):
        return self.fft_len

    def set_fft_len(self, fft_len):
        self.fft_len = fft_len
        self.set_CP_len(self.fft_len//4)

    def get_carrier_list_rf(self):
        return self.carrier_list_rf

    def set_carrier_list_rf(self, carrier_list_rf):
        self.carrier_list_rf = carrier_list_rf
        self.set_occupied_carriers_rf(((self.carrier_list_rf),))

    def get_sync_word2(self):
        return self.sync_word2

    def set_sync_word2(self, sync_word2):
        self.sync_word2 = sync_word2

    def get_sync_word1(self):
        return self.sync_word1

    def set_sync_word1(self, sync_word1):
        self.sync_word1 = sync_word1

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_rolloff(self):
        return self.rolloff

    def set_rolloff(self, rolloff):
        self.rolloff = rolloff

    def get_pilot_syms_rf(self):
        return self.pilot_syms_rf

    def set_pilot_syms_rf(self, pilot_syms_rf):
        self.pilot_syms_rf = pilot_syms_rf

    def get_pilot_symbols(self):
        return self.pilot_symbols

    def set_pilot_symbols(self, pilot_symbols):
        self.pilot_symbols = pilot_symbols

    def get_pilot_carriers_rf(self):
        return self.pilot_carriers_rf

    def set_pilot_carriers_rf(self, pilot_carriers_rf):
        self.pilot_carriers_rf = pilot_carriers_rf

    def get_pilot_carriers(self):
        return self.pilot_carriers

    def set_pilot_carriers(self, pilot_carriers):
        self.pilot_carriers = pilot_carriers

    def get_packet_len(self):
        return self.packet_len

    def set_packet_len(self, packet_len):
        self.packet_len = packet_len

    def get_occupied_carriers_rf(self):
        return self.occupied_carriers_rf

    def set_occupied_carriers_rf(self, occupied_carriers_rf):
        self.occupied_carriers_rf = occupied_carriers_rf

    def get_header_ph(self):
        return self.header_ph

    def set_header_ph(self, header_ph):
        self.header_ph = header_ph

    def get_filename(self):
        return self.filename

    def set_filename(self, filename):
        self.filename = filename
        self.blocks_file_sink_0_0.open(self.filename)

    def get_CP_len(self):
        return self.CP_len

    def set_CP_len(self, CP_len):
        self.CP_len = CP_len



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "-n", "--node-id", dest="node_id", type=str, default='100',
        help="Set Node Number [default=%(default)r]")
    return parser


def main(top_block_cls=Integrated_Comms_Rx, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(node_id=options.node_id)

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
