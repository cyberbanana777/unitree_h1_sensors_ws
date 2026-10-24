#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Вспомогательный процесс для узла odom_converter: включает одометрию робота через
Unitree SDK (OdomClient), держит её включённой и выключает при завершении.
Выделен в ОТДЕЛЬНЫЙ процесс намеренно: Unitree SDK и rmw_cyclonedds (ROS) не могут
создавать DDS-домен 0 в одном процессе ("Precondition Not Met" / segfault при
разных версиях CycloneDDS). Здесь нет ни одного импорта rclpy.

Протокол с родителем: после успешного включения одометрии процесс печатает в
stdout строку READY и блокируется на чтении stdin. Закрытие stdin (родитель
завершился или упал), SIGTERM или Ctrl+C приводят к DisableOdom и выходу.

ANNOTATION
Helper process for the odom_converter node: enables the robot odometry through
the Unitree SDK (OdomClient), keeps it enabled and disables it on exit. It is a
SEPARATE process on purpose: the Unitree SDK and rmw_cyclonedds (ROS) cannot
both create DDS domain 0 in one process ("Precondition Not Met" / segfault with
mismatched CycloneDDS versions). There is no rclpy import here.

Protocol with the parent: once odometry is enabled the process prints READY to
stdout and blocks reading stdin. Closing stdin (parent exited or crashed),
SIGTERM or Ctrl+C make it call DisableOdom and exit.
'''

import argparse
import select
import signal
import subprocess
import sys
import time

MODULE_NAME = 'odom_converter.odom_enabler'
READY_LINE = 'READY'
DEFAULT_CLIENT_TIMEOUT_SEC = 1.0
DEFAULT_START_TIMEOUT_SEC = 20.0
DEFAULT_STOP_TIMEOUT_SEC = 5.0


# ==============================================================================
# Parent side: used by the ROS node (stdlib only, no SDK, no rclpy)
# ==============================================================================

class OdomEnablerProcess:
    """Start/stop the helper process that owns the Unitree SDK."""

    def __init__(self, network_interface='',
                 client_timeout=DEFAULT_CLIENT_TIMEOUT_SEC,
                 start_timeout=DEFAULT_START_TIMEOUT_SEC,
                 stop_timeout=DEFAULT_STOP_TIMEOUT_SEC):
        self.network_interface = network_interface or ''
        self.client_timeout = float(client_timeout)
        self.start_timeout = float(start_timeout)
        self.stop_timeout = float(stop_timeout)
        self._proc = None

    def start(self):
        """Start the helper and wait until odometry is enabled.

        Raises:
            RuntimeError: the helper exited, or did not report READY in time.
                The helper's own error text is printed to this process' stderr.
        """
        cmd = [sys.executable, '-m', MODULE_NAME,
               '--client-timeout', str(self.client_timeout)]
        if self.network_interface:
            cmd += ['--network-interface', self.network_interface]

        self._proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            universal_newlines=True)  # stderr is inherited on purpose

        deadline = time.monotonic() + self.start_timeout
        while True:
            code = self._proc.poll()
            if code is not None:
                self._proc = None
                raise RuntimeError(
                    f'Odometry helper exited with code {code} before it was '
                    'ready (see its messages above)')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._kill()
                raise RuntimeError(
                    f'Odometry helper did not become ready in '
                    f'{self.start_timeout:.0f} s')
            ready, _, _ = select.select(
                [self._proc.stdout], [], [], min(0.2, remaining))
            if ready:
                line = self._proc.stdout.readline()
                if line.strip() == READY_LINE:
                    return

    def stop(self):
        """Disable odometry (closes the helper's stdin) and wait for it.

        Returns:
            True if the helper exited cleanly (odometry disabled), else False.
            Safe to call more than once.
        """
        proc, self._proc = self._proc, None
        if proc is None:
            return True
        try:
            proc.stdin.close()  # EOF -> helper calls DisableOdom and exits
        except (OSError, ValueError):
            pass
        try:
            return proc.wait(timeout=self.stop_timeout) == 0
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            return False

    def _kill(self):
        proc, self._proc = self._proc, None
        if proc is not None:
            proc.kill()
            proc.wait()


# ==============================================================================
# Child side: runs as `python3 -m odom_converter.odom_enabler`
# ==============================================================================

def main(argv=None):
    """Enable odometry, hold it until stdin closes, then disable it."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--network-interface', default='',
                        help='DDS network interface for the Unitree SDK '
                             '(empty = default)')
    parser.add_argument('--client-timeout', type=float,
                        default=DEFAULT_CLIENT_TIMEOUT_SEC)
    args = parser.parse_args(argv)

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from high_level_control import OdomClient
    except ImportError as import_error:
        print('Cannot import unitree_sdk2py / high_level_control '
              f'({import_error}). Install unitree_sdk2py and the '
              'unitree_h1_control_ws workspace (it provides '
              'high_level_control.OdomClient) and source them.',
              file=sys.stderr, flush=True)
        return 2

    # SIGTERM -> normal exit path, so that DisableOdom below still runs.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        if args.network_interface:
            ChannelFactoryInitialize(0, args.network_interface)
        else:
            ChannelFactoryInitialize(0)
        client = OdomClient()
        client.Init()
        client.SetTimeout(args.client_timeout)
        client.EnableOdom()
    except Exception as error:  # noqa: BLE001 - SDK raises various types
        print(f'Failed to enable odometry: {error!r}. Check the network '
              'interface and that the robot is powered and in a state that '
              'accepts the odometry service.', file=sys.stderr, flush=True)
        return 4

    print(READY_LINE, flush=True)
    try:
        sys.stdin.read()  # blocks until the parent closes the pipe or dies
    except (KeyboardInterrupt, SystemExit):
        pass

    try:
        client.DisableOdom()
    except Exception as error:  # noqa: BLE001 - SDK raises various types
        print(f'Failed to disable odometry: {error}',
              file=sys.stderr, flush=True)
        return 3
    return 0


if __name__ == '__main__':
    sys.exit(main())
