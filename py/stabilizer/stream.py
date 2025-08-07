#!/usr/bin/python3
"""Stabilizer streaming receiver and plotter using PyQtGraph and qasync"""

import argparse
import asyncio
import logging
import struct
import socket
import ipaddress
from collections import namedtuple
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import qasync
from scipy.signal import welch

import stabilizer

# Constants
DAC_VOLTS_PER_LSB = stabilizer.DAC_VOLTS_PER_LSB
SAMPLE_PERIOD = stabilizer.SAMPLE_PERIOD

logger = logging.getLogger(__name__)


def wrap(wide):
    """Wrap to 32 bit integer"""
    return wide & 0xFFFFFFFF


def get_local_ip(remote):
    """Get the local IP of a connection to a remote host."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote, 1883))
        return sock.getsockname()[0]
    finally:
        sock.close()


class AdcDac:
    """Stabilizer default streaming data format"""

    format_id = 1
    header = namedtuple("Header", "magic format_id batches sequence")
    header_fmt = struct.Struct("<HBBI")

    def __init__(self, header, body):
        self.header = header
        self.body = body

    def to_si(self):
        """Convert the raw data to SI units"""
        data = np.frombuffer(self.body, "<i2")
        data = data.reshape(self.header.batches, 4, -1)
        data = data.swapaxes(0, 1).reshape(4, -1)
        data[2:] ^= np.uint16(0x8000)
        return {
            "adc": data[:2] * DAC_VOLTS_PER_LSB,
            "dac": data[2:] * DAC_VOLTS_PER_LSB,
        }


class StabilizerStream(asyncio.DatagramProtocol):
    """Stabilizer streaming receiver protocol"""

    parsers = {AdcDac.format_id: AdcDac}

    @classmethod
    async def open(cls, addr, port, broker, maxsize=1):
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        if ipaddress.ip_address(addr).is_multicast:
            group = socket.inet_aton(addr)
            iface = socket.inet_aton(get_local_ip(broker))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, group + iface)
            sock.bind(("", port))
        else:
            sock.bind((addr, port))
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: cls(maxsize), sock=sock
        )
        return transport, protocol

    def __init__(self, maxsize):
        self.queue = asyncio.Queue(maxsize)

    def connection_made(self, _transport):
        logger.info("Connection made (listening)")

    def connection_lost(self, _exc):
        logger.info("Connection lost")

    def datagram_received(self, data, _addr):
        header = AdcDac.header._make(AdcDac.header_fmt.unpack_from(data))
        if header.magic != 0x057B:
            return
        frame = AdcDac(header, data[AdcDac.header_fmt.size :])
        if self.queue.full():
            self.queue.get_nowait()
        self.queue.put_nowait(frame)


class RealtimePlotter(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.fs = 1 / SAMPLE_PERIOD
        self.v_error_to_Ic_noise = 750 / (5 * args.N * 2020)
        self.transport = None
        self.stream = None
        self.streaming_enabled = False
        self.collection_duration = self.args.duration
        self.data_buffer = {"adc": np.array([[], []]), "dac": np.array([[], []])}
        self.samples_to_collect = int(args.psd_nperseg)

        self._setup_ui()
        self._configure_plots()

    def _setup_ui(self):
        self.setWindowTitle("Stabilizer Stream")
        self.resize(1200, 800)
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        control_layout = QtWidgets.QHBoxLayout()
        self.stream_button = QtWidgets.QPushButton("Start Stream")
        self.stream_button.setCheckable(True)
        self.stream_button.toggled.connect(self.toggle_stream)
        self.duration_input = QtWidgets.QDoubleSpinBox()
        self.duration_input.setSuffix(" s")
        self.duration_input.setRange(0.1, 60.0)
        self.duration_input.setValue(self.collection_duration)
        self.duration_input.valueChanged.connect(self.set_duration)
        control_layout.addWidget(QtWidgets.QLabel("Collection Time:"))
        control_layout.addWidget(self.duration_input)
        control_layout.addWidget(self.stream_button)
        control_layout.addStretch()
        layout.addLayout(control_layout)
        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win)
        self.plots = {
            "adc_t": self.win.addPlot(row=0, col=0),
            "psd_i": self.win.addPlot(row=0, col=1),
            "dac_t": self.win.addPlot(row=1, col=0),
            "psd_dac": self.win.addPlot(row=1, col=1),
        }
        self.curves = {name: plot.plot(pen="y") for name, plot in self.plots.items()}

    def _configure_plots(self):
        self.plots["adc_t"].setTitle("ADC0 Time Domain")
        self.plots["adc_t"].setLabel("left", "Voltage", units="V")
        self.plots["adc_t"].setYRange(-10.5, 10.5)
        self.plots["psd_i"].setTitle("PSD Current Noise")
        self.plots["psd_i"].setLabel("left", "PSD", units="A²/Hz")
        self.plots["psd_i"].setLogMode(x=True, y=True)
        self.plots["psd_i"].setYRange(1e-14, 1e-6)
        self.plots["dac_t"].setTitle("DAC0 Time Domain")
        self.plots["dac_t"].setLabel("left", "Voltage", units="V")
        self.plots["dac_t"].setYRange(-10.5, 10.5)
        self.plots["psd_dac"].setTitle("PSD DAC0")
        self.plots["psd_dac"].setLabel("left", "PSD", units="A²/Hz")
        self.plots["psd_dac"].setLogMode(x=True, y=True)
        self.plots["psd_dac"].setYRange(1e-14, 1e-6)

        # Calculate fixed frequency range
        min_freq_khz = (self.fs / self.args.psd_nperseg) / 1e3
        max_freq_khz = (self.fs / 2) / 1e3

        for p in self.plots.values():
            p.showGrid(x=True, y=True)
        for p in [self.plots["psd_i"], self.plots["psd_dac"]]:
            p.setLabel("bottom", "Frequency", units="kHz")
            p.setXRange(np.log10(min_freq_khz), np.log10(max_freq_khz))
            p.addItem(
                pg.InfiniteLine(pos=np.log10(15.625), angle=90, movable=False, pen="r")
            )

    def toggle_stream(self, checked):
        self.streaming_enabled = checked
        self.stream_button.setText("Stop Stream" if checked else "Start Stream")

    def set_duration(self, value):
        self.collection_duration = value

    async def update_loop(self):
        while self.isVisible():
            if not self.streaming_enabled:
                await asyncio.sleep(0.1)
                continue
            try:
                if not self.stream:
                    self.transport, self.stream = await StabilizerStream.open(
                        self.args.host,
                        self.args.port,
                        self.args.broker,
                        self.args.maxsize,
                    )
                await self.collect_and_plot()
            except (OSError, asyncio.CancelledError) as e:
                logger.error(f"Connection error: {e}")
                if self.transport:
                    self.transport.close()
                self.stream = None
                await asyncio.sleep(1)

    async def collect_and_plot(self):
        frames = []
        try:

            async def _collect():
                while True:
                    frames.append(await self.stream.queue.get())

            await asyncio.wait_for(_collect(), timeout=self.collection_duration)
        except asyncio.TimeoutError:
            pass
        if not frames:
            return
        all_si = [f.to_si() for f in frames]
        data = {
            key: np.concatenate([d[key] for d in all_si], axis=1) for key in all_si[0]
        }
        self.update_plots(data)

    def update_plots(self, data):
        adc1, dac1 = data["adc"][0], data["dac"][0]
        n_samples = adc1.size
        t_s = np.arange(n_samples) * SAMPLE_PERIOD

        freq_i, psd_i = welch(
            adc1 * self.v_error_to_Ic_noise, self.fs, nperseg=int(self.args.psd_nperseg)
        )
        freq_dac, psd_dac = welch(dac1, self.fs, nperseg=int(self.args.psd_nperseg))

        self.curves["adc_t"].setData(t_s, adc1)
        self.curves["psd_i"].setData(freq_i / 1e3, psd_i)
        self.curves["dac_t"].setData(t_s, dac1)
        self.curves["psd_dac"].setData(freq_dac / 1e3, psd_dac)

    def closeEvent(self, event):
        if self.transport:
            self.transport.close()
        for task in asyncio.all_tasks():
            task.cancel()
        event.accept()


async def main():
    parser = argparse.ArgumentParser(description="Stabilizer streaming demo")
    parser.add_argument(
        "--port", type=int, default=1234, help="Local port to listen on"
    )
    parser.add_argument("--host", default="0.0.0.0", help="Local address to listen on")
    parser.add_argument("--broker", default="mqtt", help="The MQTT broker address")
    parser.add_argument("--maxsize", type=int, default=10, help="Frame queue size")
    parser.add_argument(
        "--duration", type=float, default=1.0, help="Initial data collection time"
    )
    parser.add_argument("--psd-nperseg", default=2**12, help="Samples per PSD segment")
    parser.add_argument("--N", type=int, default=1, help="Number of windings")
    parser.add_argument(
        "--I", type=float, default=200.0, help="Current through coil (A)"
    )
    args = parser.parse_args()

    app = pg.mkQApp("Stabilizer Stream")
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    plotter = RealtimePlotter(args)
    plotter.show()

    try:
        await plotter.update_loop()
    except asyncio.CancelledError:
        logger.info("Async tasks cancelled, shutting down.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    try:
        qasync.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user")
