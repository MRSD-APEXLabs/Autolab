# Autolab CLI Tool

The `autolab` command-line tool provides a unified interface for common development tasks in the Autolab project, including setup, installation, and container management.

## Installation

The `autolab` tool is included in the Autolab repository. To set it up:

1. Clone the Autolab repository:
   ```bash
   git clone https://github.com/arker123/Autolab
   cd Autolab
   ```

2. Run the setup command:
   ```bash
   ./autolab setup
   ```

This will add the `autolab` command to your PATH by modifying your shell profile (`.bashrc` or `.zshrc`).

## Basic Usage

```bash
autolab <command> [options]
```

To see all available commands:
```bash
autolab commands
```

To get help for a specific command:
```bash
autolab help <command>
```

## Core Commands

- `install`: Install dependencies (Docker Engine, Docker Compose, etc.)
- `setup`: Configure AutoLab settings and add to shell profile
- `up [service]`: Start services using Docker Compose
- `stop [service]`: Stop services
- `connect [container]`: Connect to a running container (supports partial name matching)
- `status`: Show status of all containers
- `logs [container]`: View logs for a container (supports partial name matching)

### Container Name Matching

The `connect` and `logs` commands support partial name matching for container names. This means you can:

1. Use just a portion of the container name (e.g., `autolab connect web` will match `autolab_web_1`)
2. If multiple containers match, you'll be shown a list and prompted to select one
3. The matching is case-insensitive and supports fuzzy matching

Example:
```bash
$ autolab connect web
[WARN] Multiple containers match 'web'. Please be more specific or select from the list below:
NUM     CONTAINER NAME  IMAGE   STATUS
1       autolab_web_1  nginx:latest    Up 2 hours
2       autolab_webapi_1       node:14 Up 2 hours

Options:
  1. Enter a number to select a container
  2. Type 'q' to quit
  3. Press Ctrl+C to cancel and try again with a more specific name

Your selection: 1
[INFO] Connecting to container: autolab_web_1
[INFO] Tip: Next time, you can directly use 'autolab connect autolab_web_1' for this container
```

## Development Commands

- `test`: Run tests
- `docs`: Build documentation
- `lint`: Lint code
- `format`: Format code

## Extending the Tool

The `autolab` tool is designed to be easily extensible. You can add new commands by creating module files in the `.autolab/modules/` directory.

### Creating a New Module

1. Create a new `.sh` file in the `.autolab/modules/` directory:
   ```bash
   touch .autolab/modules/mymodule.sh
   ```

2. Add your command functions and register them:
   ```bash
   #!/usr/bin/env bash

   # Function to implement your command
   function cmd_mymodule_mycommand {
       log_info "Running my command..."
       # Your command implementation here
   }

   # Register commands from this module
   function register_mymodule_commands {
       COMMANDS["mycommand"]="cmd_mymodule_mycommand"
       COMMAND_HELP["mycommand"]="Description of my command"
   }
   ```

3. Make the module executable:
   ```bash
   chmod +x .autolab/modules/mymodule.sh
   ```

Your new command will be automatically loaded and available as `autolab mycommand`.

### Available Helper Functions

When creating modules, you can use these helper functions:

- `log_info "message"`: Print an info message
- `log_warn "message"`: Print a warning message
- `log_error "message"`: Print an error message
- `check_docker`: Check if Docker is installed and running

## Configuration

The `autolab` tool stores its configuration in `~/.autolab.conf`. This file is created when you run `autolab setup`.

## License

This tool is part of the AutoLab project and is subject to the same license terms.