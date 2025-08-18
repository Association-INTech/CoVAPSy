# UV Dependency Management Guide

This project now uses [UV](https://github.com/astral-sh/uv) for fast, reliable Python package management.

## Quick Start

### Install UV
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.cargo/env
```

### Install Project Dependencies
```bash
# Install all runtime dependencies
uv sync

# Install with documentation dependencies
uv sync --extra docs

# Install with development dependencies  
uv sync --extra dev

# Install all optional dependencies
uv sync --all-extras
```

### Running Python Scripts and Commands
```bash
# Run a script using the project's virtual environment
uv run python src/HL/main.py

# Run a script directly (if defined in pyproject.toml)
uv run covapsy-main

# Run any command with the project's dependencies
uv run mkdocs serve

# Note: Always use 'uv run' prefix to ensure you're using the UV-managed environment
```

### Managing Dependencies

#### Add a new dependency
```bash
# Add runtime dependency
uv add numpy

# Add development dependency
uv add --dev pytest

# Add documentation dependency
uv add --optional docs mkdocs
```

#### Remove a dependency
```bash
uv remove package-name
```

#### Update dependencies
```bash
# Update all packages
uv lock --upgrade

# Update specific package
uv add package-name@latest
```

### Virtual Environment Management

UV automatically manages virtual environments. You can also:

```bash
# Activate the virtual environment manually
source .venv/bin/activate

# Or run commands directly
uv run python your_script.py
```

### Lock File

The `uv.lock` file pins exact versions for reproducible builds. Commit this file to version control.


### Old way (pip):
```bash
pip install -r requirements.txt
```

### New way (UV):
```bash
uv sync
```

## Benefits of UV

- **Fast**: 10-100x faster than pip
- **Reliable**: Deterministic resolution with lock files
- **Cross-platform**: Works on Linux, macOS, Windows
- **Drop-in replacement**: Compatible with existing Python tooling
- **Modern**: Built in Rust with modern dependency resolution

## Troubleshooting

### Common Issues

#### "Plugin not installed" or "Module not found" errors
**Problem**: Running commands directly (e.g., `mkdocs serve`) instead of through UV.
**Solution**: Always prefix commands with `uv run`:
```bash
# ❌ Wrong
mkdocs serve

# ✅ Correct
uv run mkdocs serve
```

#### UV installs unwanted dependencies
**Problem**: UV may incorrectly resolve optional dependencies due to dependency resolution bugs.
**Solution**: Remove problematic lock files and clear cache:
```bash
rm uv.lock
uv cache clean
uv sync --extra docs
```
