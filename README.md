# Simple 485 Remastered Micro

_A MicroPython port of the [Simple 485 Remastered](https://github.com/Me-Phew/simple485-remastered) library for slave devices._

[![License: GPL-3.0](https://img.shields.io/badge/License-GNU%20GPLv3-green.svg)](https://opensource.org/licenses/MIT)

## Requirements
This port uses https://github.com/Me-Phew/micropython-logging, which is a fork of https://github.com/erikdelange/MicroPython-Logging.
It should, however, be compatible with any logging library that uses the same interface as the standard Python logging library.

## Installation
Simply copy the simple485_remastered_micro.py file onto your microcontroller.

### Manual Hardware Tests (`/test_scripts`)

These scripts are designed to test the library's performance and robustness on real hardware. They are essential for verifying behavior in a real-world environment with physical RS485 transceivers and wiring.

More details on how to run these tests can be found in the [CPython version repo](https://github.com/Me-Phew/simple485-remastered/blob/main/test_scripts/README.md) 

## API at a Glance

There are only two main classes in this port that you need to know about for most use cases:

-   **`Slave`**: The abstract base class for creating all slave devices. You must subclass it and implement `_handle_unicast_message`.
-   **`ReceivedMessage`**: Represents a message received by a slave. It contains the sender's address, the message type, and the payload.

## Contributing

Contributions are welcome! If you find a bug or have a feature request, please open an issue. If you'd like to contribute code, please feel free to fork the repository and submit a pull request.

## License

This project is licensed under the MIT License—see the [LICENSE](LICENSE) file for details.

## Acknowledgements
This project is inspired by the original work of [rzeman9](https://github.com/rzeman9)