# simple485_remastered_micro.py
# A MicroPython port of the simple485-remastered library for slave devices.

# ------------------------------------------------------------------------------
#  Last modified 22.08.2025, 18:38, simple485-remastered-micro                  -
# ------------------------------------------------------------------------------

import utime
import machine
import ubinascii

from micropython import const  # MicroPython specific optimization for compile time constants

# This port uses https://github.com/Me-Phew/micropython-logging,
# which is a fork of https://github.com/erikdelange/MicroPython-Logging.
# It should, however, be compatible with any logging library
# that uses the same interface as the standard Python logging library.
import logging

# --- Protocol definitions (see simple485_remastered/protocol.py) ---


MAX_MESSAGE_LEN = const(255)
LINE_READY_TIME_MS = const(10)
BITS_PER_BYTE = const(10)
PACKET_TIMEOUT_MS = const(500)
FIRST_NODE_ADDRESS = const(0)
MASTER_ADDRESS = const(FIRST_NODE_ADDRESS)
BROADCAST_ADDRESS = const(255)
LAST_NODE_ADDRESS = const(BROADCAST_ADDRESS - 1)


class ControlSequence:
    SOH = b"\x01"
    STX = b"\x02"
    ETX = b"\x03"
    EOT = b"\x04"
    LF = b"\x0a"
    NULL = b"\x00"


class ReceiverState:
    IDLE = const(0)
    SOH_RECEIVED = const(1)
    DEST_ADDRESS_RECEIVED = const(2)
    SRC_ADDRESS_RECEIVED = const(3)
    TRANSACTION_ID_RECEIVED = const(4)
    MESSAGE_LEN_RECEIVED = const(5)
    STX_RECEIVED = const(6)
    ETX_RECEIVED = const(7)
    CRC_OK = const(8)


def is_valid_node_address(address):
    return FIRST_NODE_ADDRESS <= address <= LAST_NODE_ADDRESS


def is_valid_slave_address(address):
    return is_valid_node_address(address) and address != MASTER_ADDRESS


# --- Utility functions (see simple485_remastered/utils.py) ---


def get_milliseconds():
    return utime.ticks_ms()


# --- Model definitions (see simple485_remastered/models.py) ---


class ReceivingMessage:
    def __init__(self, timestamp=None):
        self.timestamp = timestamp
        self.dst_address = None
        self.src_address = None
        self.transaction_id = None
        self.length = None
        self.crc = None
        self.is_first_nibble = True
        self.incoming = 0
        self.payload_buffer = b""


class ReceivedMessage:
    def __init__(self, *, src_address, dest_address, transaction_id, length, payload, originating_bus):
        self.src_address = src_address
        self.dest_address = dest_address
        self.transaction_id = transaction_id
        self.length = length
        self.payload = payload
        self._originating_bus = originating_bus

    def __repr__(self):
        payload_hex = ubinascii.hexlify(self.payload).decode("ascii")
        return "ReceivedMessage(src={}, dest={}, tid={}, len={}, payload={})".format(
            self.src_address, self.dest_address, self.transaction_id, self.length, payload_hex
        )

    def is_broadcast(self):
        return self.dest_address == BROADCAST_ADDRESS

    def respond(self, message, allow_broadcast=False):
        if self._originating_bus is None:
            raise ValueError("Originating bus is not set for this message.")

        if self.is_broadcast() and not allow_broadcast:
            raise ValueError("Cannot respond to a broadcast message.")

        return self._originating_bus.send_message(self.src_address, message, self.transaction_id)


# --- Core functionality (see simple485_remastered/core.py) ---

DEFAULT_TRANSCEIVER_TOGGLE_TIME_MS = 20


class Simple485Remastered:
    def __init__(
        self,
        *,
        interface,
        interface_baudrate,
        address,
        transmit_mode_pin,
        transceiver_toggle_time_ms=DEFAULT_TRANSCEIVER_TOGGLE_TIME_MS,
        log_level=logging.INFO,
    ):
        self._logger = logging.getLogger(self.__class__.__name__, level=log_level)

        self._interface = interface
        self._interface_baudrate = interface_baudrate

        if not is_valid_node_address(address):
            raise ValueError(f"Invalid address: {address}")

        self._address = address

        self._transceiver_toggle_time_ms = transceiver_toggle_time_ms
        if self._transceiver_toggle_time_ms <= 0:
            raise ValueError(
                f"Invalid transceiver toggle time: {self._transceiver_toggle_time_ms}. It must be a positive float representing milliseconds."
            )

        self._transmit_mode_pin = machine.Pin(transmit_mode_pin, machine.Pin.OUT)
        self._disable_transmit_mode()

        self._last_bus_activity = get_milliseconds()
        self._receiver_state = ReceiverState.IDLE
        self._receiving_message = None
        self._received_messages = []
        self._output_messages = []

        self._logger.debug(f"Initialized {self.__class__.__name__} with address {self._address}")

    def get_last_bus_activity(self):
        return self._last_bus_activity

    def get_address(self):
        return self._address

    def set_address(self, address):
        if not is_valid_node_address(address):
            raise ValueError(f"Invalid address: {address}")

        self._logger.info(f"Changing address from {self._address} to {address}")
        self._address = address

    def _enable_transmit_mode(self):
        self._transmit_mode_pin.on()
        utime.sleep_ms(self._transceiver_toggle_time_ms)

    def _disable_transmit_mode(self):
        self._transmit_mode_pin.off()
        utime.sleep_us(self._transceiver_toggle_time_ms)

    def loop(self):
        self._receive()
        self._transmit()

        if (
            self._receiver_state != ReceiverState.IDLE
            and utime.ticks_diff(get_milliseconds(), self._receiving_message.timestamp) > PACKET_TIMEOUT_MS
        ):
            self._logger.warning("Packet timeout, resetting receiver state.")
            self._receiver_state = ReceiverState.IDLE
            self._receiving_message = None

    def pending_send(self):
        self._logger.debug(f"Pending send: {len(self._output_messages)}")

        return len(self._output_messages) > 0

    def send_message(self, dst_address, payload, transaction_id=0):
        message_len = len(payload)

        if message_len == 0:
            raise ValueError("Cannot send an empty message.")

        if message_len > MAX_MESSAGE_LEN:
            raise ValueError(f"Message length exceeds maximum of {MAX_MESSAGE_LEN}.")

        text_buffer = (
            ControlSequence.LF * 3
            + ControlSequence.SOH
            + bytes([dst_address])
            + bytes([self._address])
            + bytes([transaction_id])
            + bytes([message_len])
            + ControlSequence.STX
        )
        crc = self._address ^ dst_address ^ message_len
        for i in range(message_len):
            crc ^= payload[i]
            byte = payload[i] & 240
            byte = byte | (~(byte >> 4) & 15)
            text_buffer += bytes([byte])
            byte = payload[i] & 15
            byte = byte | ((~byte << 4) & 240)
            text_buffer += bytes([byte])
        text_buffer += ControlSequence.ETX + bytes([crc]) + ControlSequence.EOT + ControlSequence.LF * 2

        self._logger.debug(f"Queuing message, buffer: {text_buffer.hex()}, dest_address: {dst_address}")
        self._output_messages.append(text_buffer)
        return True

    def available(self):
        return len(self._received_messages)

    def read(self):
        if len(self._received_messages) == 0:
            raise ValueError("No messages available to read.")
        return self._received_messages.pop(0)

    def _process_byte(self, byte):
        if self._receiver_state == ReceiverState.IDLE:
            if byte == ControlSequence.SOH:
                self._receiver_state = ReceiverState.SOH_RECEIVED

                self._receiving_message = ReceivingMessage(timestamp=get_milliseconds())

        elif self._receiver_state == ReceiverState.SOH_RECEIVED:
            self._receiving_message.dst_address = byte[0]

            if (
                self._receiving_message.dst_address != self._address
                and self._receiving_message.dst_address != BROADCAST_ADDRESS
            ):
                self._logger.info("Received message for another address. Ignoring.")
                self._receiver_state = ReceiverState.IDLE
                self._receiving_message = None
            else:
                self._receiver_state = ReceiverState.DEST_ADDRESS_RECEIVED

        elif self._receiver_state == ReceiverState.DEST_ADDRESS_RECEIVED:
            self._receiving_message.src_address = byte[0]
            self._receiver_state = ReceiverState.SRC_ADDRESS_RECEIVED

        elif self._receiver_state == ReceiverState.SRC_ADDRESS_RECEIVED:
            self._receiving_message.transaction_id = byte[0]
            self._receiver_state = ReceiverState.TRANSACTION_ID_RECEIVED

        elif self._receiver_state == ReceiverState.TRANSACTION_ID_RECEIVED:
            self._receiving_message.length = byte[0]

            if not (0 < self._receiving_message.length <= MAX_MESSAGE_LEN):
                self._logger.warning(f"Received invalid message length of: {self._receiving_message.length}. Dropping.")
                self._receiver_state = ReceiverState.IDLE
                self._receiving_message = None
            else:
                self._receiver_state = ReceiverState.MESSAGE_LEN_RECEIVED

        elif self._receiver_state == ReceiverState.MESSAGE_LEN_RECEIVED:
            if byte == ControlSequence.STX:
                self._receiving_message.crc = (
                    self._receiving_message.dst_address
                    ^ self._receiving_message.src_address
                    ^ self._receiving_message.length
                )
                self._receiver_state = ReceiverState.STX_RECEIVED
            else:
                self._logger.warning("Expected STX, but got other data. Dropping.")
                self._receiver_state = ReceiverState.IDLE
                self._receiving_message = None

        elif self._receiver_state == ReceiverState.STX_RECEIVED:
            is_valid_encoded_byte = (~(((byte[0] << 4) & 240) | ((byte[0] >> 4) & 15))) & 0xFF == byte[0]

            if is_valid_encoded_byte:
                if self._receiving_message.is_first_nibble:
                    self._receiving_message.incoming = byte[0] & 240
                    self._receiving_message.is_first_nibble = False
                else:
                    self._receiving_message.is_first_nibble = True
                    self._receiving_message.incoming |= byte[0] & 15
                    self._receiving_message.payload_buffer += bytes([self._receiving_message.incoming])
                    self._receiving_message.crc ^= self._receiving_message.incoming
                return

            if byte == ControlSequence.ETX:
                if len(self._receiving_message.payload_buffer) == self._receiving_message.length:
                    self._receiver_state = ReceiverState.ETX_RECEIVED
                else:
                    self._logger.warning("ETX received but payload length is incorrect. Dropping.")
                    self._receiver_state = ReceiverState.IDLE
                    self._receiving_message = None
                return

            self._logger.warning("Invalid data byte. Dropping.")
            self._receiver_state = ReceiverState.IDLE
            self._receiving_message = None

        elif self._receiver_state == ReceiverState.ETX_RECEIVED:
            if byte[0] == self._receiving_message.crc:
                self._receiver_state = ReceiverState.CRC_OK
            else:
                self._logger.warning("CRC mismatch. Dropping.")
                self._receiver_state = ReceiverState.IDLE
                self._receiving_message = None

        elif self._receiver_state == ReceiverState.CRC_OK:
            if byte == ControlSequence.EOT:
                message = ReceivedMessage(
                    src_address=self._receiving_message.src_address,
                    dest_address=self._receiving_message.dst_address,
                    transaction_id=self._receiving_message.transaction_id,
                    length=self._receiving_message.length,
                    payload=self._receiving_message.payload_buffer,
                    originating_bus=self,
                )
                self._received_messages.append(message)
                self._logger.info(f"Successfully received message: {message}")
            else:
                self._logger.warning("Expected EOT. Dropping packet.")

            self._receiver_state = ReceiverState.IDLE
            self._receiving_message = None

    def _receive(self):
        while self._interface.any() > 0:
            byte = self._interface.read(1)

            if byte is None or (byte == ControlSequence.NULL and self._receiver_state == ReceiverState.IDLE):
                continue

            self._last_bus_activity = get_milliseconds()
            self._logger.debug(f"Received byte: {byte.hex()} in state {self._receiver_state}")

            self._process_byte(byte)

    def _transmit(self):
        if not self._output_messages:
            return False

        if utime.ticks_diff(get_milliseconds(), self._last_bus_activity) < LINE_READY_TIME_MS:
            self._logger.debug("Line not ready for transmission, waiting.")
            return False

        message_to_send = self._output_messages[0]
        self._logger.debug(f"Attempting to transmit a message, buffer: {message_to_send.hex()}")

        try:
            self._enable_transmit_mode()
            self._interface.write(message_to_send)

            try:
                while not self._interface.txdone():
                    utime.sleep_us(10)
            except AttributeError:
                self._logger.warning("Interface does not support txdone. Falling back to using flush with manual timing calculation.")

                safety_margin_factor = 1.1

                try:
                    self._interface.flush()
                except AttributeError:
                    self._logger.warning("Interface does not support flush. Increasing safety margin factor for manual timing calculation.")
                    safety_margin_factor = 1.2

                transmission_time_s = (
                    (len(message_to_send) * BITS_PER_BYTE) / self._interface_baudrate
                ) * safety_margin_factor

                transmission_time_us = int(transmission_time_s * 1_000_000)

                self._logger.debug(f"Message transmission time: {transmission_time_s} s ({transmission_time_us} us)")

                utime.sleep_us(transmission_time_us)
        except OSError as e:
            self._logger.exception(e, f"Serial communication error: {e}. Message not sent. Will retry later.")
            return False
        except Exception as e:
            self._logger.exception(e, f"Unexpected error during transmission: {e}. Message not sent. Will retry later.")
            return False
        finally:
            self._disable_transmit_mode()

        self._last_bus_activity = get_milliseconds()
        self._output_messages.pop(0)
        self._logger.info("Message sent successfully, buffer: %s", message_to_send.hex())
        return True


# --- Node abstract class (see simple485_remastered/node.py) ---


class Node:
    def __init__(
        self,
        *,
        interface,
        interface_baudrate,
        address,
        transmit_mode_pin,
        transceiver_toggle_time_ms=DEFAULT_TRANSCEIVER_TOGGLE_TIME_MS,
        log_level=logging.INFO,
    ):
        self._logger = logging.getLogger(self.__class__.__name__, level=log_level)

        if not is_valid_node_address(address):
            raise ValueError(f"Invalid address for Node: {address}")

        self._address = address
        self._bus = Simple485Remastered(
            interface=interface,
            interface_baudrate=interface_baudrate,
            address=address,
            transmit_mode_pin=transmit_mode_pin,
            transceiver_toggle_time_ms=transceiver_toggle_time_ms,
            log_level=log_level,
        )

        self._logger.debug(f"Initialized {self.__class__.__name__} with address {self._address}")

    def _loop(self):
        self._bus.loop()

        while self._bus.available() > 0:
            try:
                message = self._bus.read()
                self._logger.info(f"Received a message: {message}")

                self._handle_incoming_message(message)
            except Exception as e:
                self._logger.exception(e, f"Error while handling incoming message: {e}")

    def _handle_incoming_message(self, message, elapsed_ms=None):
        raise NotImplementedError


# --- Slave abstract class (see simple485_remastered/slave.py) ---


class Slave(Node):
    def __init__(
        self,
        *,
        interface,
        interface_baudrate,
        address,
        transmit_mode_pin,
        transceiver_toggle_time_ms=DEFAULT_TRANSCEIVER_TOGGLE_TIME_MS,
        log_level=logging.INFO,
    ):
        if not is_valid_slave_address(address):
            msg = "Invalid address for Slave: {}. Address must be between {} and {}.".format(
                address, FIRST_NODE_ADDRESS + 1, LAST_NODE_ADDRESS
            )
            raise ValueError(msg)

        super(Slave, self).__init__(
            interface=interface,
            interface_baudrate=interface_baudrate,
            address=address,
            transmit_mode_pin=transmit_mode_pin,
            transceiver_toggle_time_ms=transceiver_toggle_time_ms,
            log_level=log_level,
        )

    def loop(self):
        self._loop()

    def _handle_incoming_message(self, message, elapsed_ms=None):
        if message.src_address != MASTER_ADDRESS:
            self._logger.warning(
                f"Received message from non-master address {message.src_address}. Slave only accepts messages from the master ({MASTER_ADDRESS})."
            )
            return None

        if message.is_broadcast():
            return self._handle_broadcast_message(message)
        else:
            return self._handle_unicast_message(message)

    def _handle_broadcast_message(self, message):
        raise NotImplementedError

    def _handle_unicast_message(self, message):
        raise NotImplementedError
