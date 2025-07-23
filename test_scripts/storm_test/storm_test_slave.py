"""An "echo slave" script for the data integrity storm test.

This script acts as the counterpart to `storm_test_master.py`. It has a
unique and complex behavior designed to rigorously test the master and the
communication protocol.

Key Behavior:
- **Dynamic Address Changing: ** Like the address range test slave, this script
  does not have a fixed address. It starts at `FIRST_ADDRESS` and simulates an
  entire bus of slaves by incrementing its own address as the test progresses.
- **Echo Server: ** For each address it listens on, it expects a series of
  payloads of varying lengths from the master. Its sole job is to "echo" each
  payload back to the master immediately upon receipt.
- **Nested Loop: ** It waits in a nested loop: the outer loop for the address,
  and the inner loop for each payload length.

Dependencies:
- This script is designed to work exclusively with `storm_test_master.py`.

Usage:
1. Run this script on a slave device connected to the bus.
2. Configure the `serial_port` in the `__init__` method below.
3. Run the corresponding `storm_test_master.py` on the master device.
"""

# ------------------------------------------------------------------------------
#  Last modified 23.07.2025, 16:57, simple485-remastered-micro                 -
# ------------------------------------------------------------------------------

import logging
import time
from machine import UART, Pin

from simple485_remastered_micro import Slave, ReceivedMessage

logger = logging.getLogger(__name__, level=logging.DEBUG)

# -- Transceiver Configuration ---
UART_ID = 0
BAUDRATE = 9600
UART_TX_PIN = 16
UART_RX_PIN = 17
TRANSCEIVER_PIN = None

# --- Test Configuration ---
FIRST_ADDRESS = 1
LAST_ADDRESS = 254
PAYLOAD_LENGTH_RANGE = (1, 256)
SIMULATED_FAILURES_COUNT = 0
ITERATIONS = 1


class StormTestSlave(Slave):
    """A concrete Slave implementation for the storm test.

    Its main role is to act as a dynamic "echo" server, changing its address
    to match the master's test progression.
    """

    def __init__(self):
        """Initializes the Slave and the serial port for communication."""
        interface = UART(UART_ID, baudrate=BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))

        self._current_address = FIRST_ADDRESS
        super().__init__(interface=interface, address=self._current_address)

        self._payload_received = False
        self._simulated_failures_count = SIMULATED_FAILURES_COUNT

    def _handle_unicast_message(self, message: ReceivedMessage) -> None:
        """Dispatches any unicast message to the main handler."""
        self.on_unicast_message(message)

    def _handle_broadcast_message(self, message: ReceivedMessage) -> None:
        """Dispatches any broadcast message to the main handler."""
        self.on_broadcast_message(message)

    def _on_payload_registered(self) -> None:
        """Helper method to signal that a payload has been processed."""
        self._payload_received = True

    def on_broadcast_message(self, _message: ReceivedMessage):
        """Handles a broadcast message by logging it and not responding."""
        logger.info("Received broadcast message. Not responding.")
        self._on_payload_registered()

    def on_unicast_message(self, message: ReceivedMessage):
        """The core "echo" logic of the slave.

        It takes the payload from the incoming message and immediately sends it
        back to the master using the `message.respond()` helper. It can also
        simulate failures.
        """
        if self._simulated_failures_count > 0:
            self._simulated_failures_count -= 1
            logger.warning("Simulating a failure by not responding.")
            self._on_payload_registered()
            return

        message.respond(message=message.payload)
        logger.debug(f"Received payload of length {len(message.payload)}, echoed it back.")

        self._on_payload_registered()
        # Reset failure counter for the next payload
        self._simulated_failures_count = SIMULATED_FAILURES_COUNT

    def run(self):
        """Runs the main test loop for the slave.

        It iterates through addresses and payload lengths, waiting for each
        message from the master and echoing it back.
        """
        for i in range(ITERATIONS):
            logger.info(f"--- Starting Iteration {i + 1}/{ITERATIONS} ---")
            while self._current_address <= LAST_ADDRESS:
                self._set_address(self._current_address)
                logger.info(f"--- Now listening on address: {self._current_address} ---")
                for payload_length in range(*PAYLOAD_LENGTH_RANGE):
                    self._payload_received = False
                    logger.debug(f"Waiting for payload of length {payload_length}...")

                    # Wait until a payload is received AND the echo response is fully sent.
                    # This prevents a race condition where the slave might expect the
                    # next payload before the master has received the previous response.
                    while not self._payload_received or self._pending_send():
                        self._loop()
                        time.sleep(0.0001)
                self._current_address += 1

            logger.info(
                f"Successfully tested {self._current_address - FIRST_ADDRESS} addresses from range "
                f"{FIRST_ADDRESS} - {self._current_address - 1}."
            )
            self._current_address = FIRST_ADDRESS  # Reset for next iteration
        logger.info("--- Storm Test Complete ---")


if __name__ == "__main__":
    # Script entry point
    storm_test_slave = StormTestSlave()
    storm_test_slave.run()
