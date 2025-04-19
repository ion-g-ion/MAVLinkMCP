# MAVLink MCP Server

This repository contains a Python-based Model Context Protocol (MCP) server for interacting with MAVLink-enabled devices, such as drones running PX4 software. 

## Prerequisites

- Python 3.10 or higher

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/MAVLinkMCP.git
   cd MAVLinkMCP
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

Run the MCP server with the following command:
```bash
python src/server/mavlinkmcp.py
```

Alternatively, you can run the server using `uv run`:
```bash
uv run src/server/mavlinkmcp.py
```

## Example agent usage

An example client is implemented in `example_agent.py` using the `fastagent` library. This demonstrates how to create an AI agent that interacts with the MCP server and supports human input for controlling a drone.
Export the OpenAI key before running it.


## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
