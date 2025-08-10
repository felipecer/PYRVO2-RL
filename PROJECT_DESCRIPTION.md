# PYRVO2-RL Project Description for Documentation Agent

## Agent Instructions
**Target**: Create comprehensive documentation for researchers using PYRVO2-RL in RL experiments
**Priority Order**: 1) RL extensions, 2) Usage examples, 3) Technical implementation
**Format**: Sphinx-based documentation with MyST-Parser (Markdown support)
**Output**: `docs/` folder structure compatible with ReadTheDocs
**Approach**: Read only files specified for each section to optimize resources
**CRITICAL**: ALL documentation work must be performed within the dedicated virtual environment

### Virtual Environment Requirements
**The agent MUST**:
1. **Create virtual environment** before any documentation work
2. **Activate environment** for every documentation command
3. **Never work outside** the virtual environment
4. **Verify environment** is active before each major step
5. **Clean up environment** only when all work is complete

## Documentation Infrastructure Setup

### Required Documentation Structure
```
docs/
├── source/
│   ├── conf.py              # Sphinx configuration
│   ├── index.md             # Main documentation entry
│   ├── installation.md     # Installation guide
│   ├── quickstart.md       # Quick start for researchers
│   ├── api/
│   │   ├── rl_extensions.md # RL API reference
│   │   ├── rvo2_core.md     # Core RVO2 API
│   │   └── utilities.md     # Utility functions
│   ├── examples/
│   │   ├── basic_usage.md   # Simple examples
│   │   ├── multi_agent.md   # Multi-agent scenarios
│   │   └── advanced.md      # Advanced configurations
│   └── technical/
│       ├── architecture.md  # Technical implementation
│       └── performance.md   # Performance considerations
├── requirements.txt         # Documentation dependencies
└── .readthedocs.yaml       # ReadTheDocs configuration
```

### Required Dependencies for Documentation
- `sphinx>=5.0.0`
- `myst-parser>=0.18.0`
- `sphinx-rtd-theme>=1.0.0`
- `sphinx-autodoc-typehints>=1.19.0`

---

## Section 1: Project Overview
**Files to read**: Already analyzed
**Content to document**:

### Basic Information
- **Project Name**: PYRVO2-RL 
- **Version**: 0.1.0
- **Author**: Felipe Cerda
- **Description**: Python bindings for RVO2 extended for reinforcement learning
- **Python Requirements**: >=3.10
- **Build System**: setuptools + pybind11>=2.10

### Architecture Overview
- **Module Structure**: Three-tier architecture
  - `rvo2_rl.rvo2`: Pure RVO2 simulator bindings (vanilla API)
  - `rvo2_rl.util`: Utility functions (OpenMP support)
  - `rvo2_rl.rl`: RL extensions (main focus for researchers)

### Key Dependencies
- **Core**: pybind11, OpenMP (platform-specific)
- **Build optimizations**: O3, march=native, ffast-math, C++17
- **Platform support**: Linux (primary), macOS (Homebrew LLVM)

---

## Section 2: RL Extensions (PRIORITY 1)
**Files to read for this section**:
- `src/bindings/rl_bindings.cpp`
- `src/impl/RVO2_RL_Wrapper.cpp` 
- `src/include/RVO2_RL_Wrapper.h`
- `src/include/structs.h` (already analyzed)

**Content to document**:

### Core RL Wrapper Class
- **Main Class**: `RVO2RLWrapper` in `rvo2_rl.rl` module
- **Purpose**: Extends RVO2 with RL-friendly observation and control interfaces

### Key RL Features Identified
1. **Observation Modes**: 
   - `ObsMode.Cartesian`: Standard x,y coordinates
   - `ObsMode.Polar`: Distance/angle representation

2. **Observation System**:
   - Neighbor observations with masking support
   - LiDAR-style sensing (360-degree configurable)
   - Batch processing for multi-agent scenarios

3. **Data Structures for RL**:
   - `BatchAgentData`: Agent state for batch processing
   - `NeighborDataObsCart`: Cartesian neighbor observations
   - `NeighborDataObsPolar`: Polar neighbor observations

4. **Step Management**:
   - Step counting for episode management
   - Preferred velocity setting for action application

### Constructor Parameters for Researchers
```python
RVO2RLWrapper(
    time_step=0.25,          # Simulation timestep
    neighbor_dist=15.0,      # Agent sensing range
    max_neighbors=10,        # Max neighbors to observe
    time_horizon=5.0,        # Planning horizon
    time_horizon_obst=5.0,   # Obstacle avoidance horizon
    radius=0.5,              # Agent radius
    max_speed=2.0,           # Maximum agent speed
    velocity=Vector2(),      # Initial velocity
    mode=ObsMode.Cartesian,  # Observation coordinate system
    use_obs_mask=False,      # Enable observation masking
    use_lidar=False,         # Enable LiDAR observations
    lidar_count=360,         # LiDAR ray count
    lidar_range=18.0,        # LiDAR sensing range
    max_step_count=256       # Episode length limit
)
```

---

## Section 3: Usage Examples (PRIORITY 2)
**Files to read for this section**:
- `tests/obs/` directory contents
- Focus on test file names and structure to understand usage patterns

**Content to document**:
- Practical examples from test cases
- Common RL experiment setups
- Observation space configuration examples
- Multi-agent scenario setup

---

## Section 4: Core RVO2 Integration (PRIORITY 3)
**Files to read for this section**:
- `src/rvo2/include/RVO.h`
- `src/rvo2/include/RVOSimulator.h`
- `src/rvo2/include/Vector2.h`
- `src/bindings/rvo2_bindings.cpp`

**Content to document**:
- How RVO2 original API is exposed
- Vector2 operations for researchers
- Direct simulator access methods
- Integration between vanilla RVO2 and RL extensions

---

## Section 5: Technical Implementation (LOWEST PRIORITY)
**Files to read for this section**:
- `src/impl/RVO2_RL_Wrapper.cpp`
- `src/impl/RayCastingEngine.cpp`
- `src/include/RayCastingEngine.h`
- `src/bindings/openmp_bindings.cpp`

**Content to document**:
- Performance optimizations
- OpenMP parallelization details
- Ray casting implementation for LiDAR
- Memory management and efficiency considerations

---

## Section 6: Documentation Infrastructure Setup
**Files to create**:
- `docs/source/conf.py` - Sphinx configuration
- `docs/requirements.txt` - Documentation dependencies  
- `docs/.readthedocs.yaml` - ReadTheDocs configuration
- `docs/source/index.md` - Main documentation entry point

**Key Configuration Details**:

### Sphinx conf.py essentials:
```python
extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon'
]
html_theme = 'sphinx_rtd_theme'
source_suffix = {'.md': None, '.rst': None}
```

### ReadTheDocs YAML:
```yaml
version: 2
build:
  os: ubuntu-22.04
  tools:
    python: "3.11"
python:
  install:
    - requirements: docs/requirements.txt
    - method: pip
      path: .
sphinx:
  configuration: docs/source/conf.py
```

**Content to document**:
- Complete setup instructions for contributors
- Local documentation building commands
- ReadTheDocs integration guide
- Maintenance procedures for keeping docs updated

---

## Section 7: Documentation Validation & Testing
**Automated validation steps the agent MUST perform**:

### Step 1: Create Documentation Infrastructure
1. Create `docs/` directory structure
2. Generate all configuration files (`conf.py`, `.readthedocs.yaml`, `requirements.txt`)
3. Verify file structure matches the required layout

### Step 2: Install Documentation Dependencies
```bash
# Agent should create isolated environment for documentation work
python -m venv docs_env
source docs_env/bin/activate  # On Windows: docs_env\Scripts\activate

# Install documentation dependencies
pip install --upgrade pip
pip install sphinx myst-parser sphinx-rtd-theme sphinx-autodoc-typehints

# Verify installation
python -c "import sphinx, myst_parser; print('✓ Documentation tools installed')"
```

### Step 3: Build Documentation Locally
```bash
# ENSURE VIRTUAL ENVIRONMENT IS ACTIVE
if [[ "$VIRTUAL_ENV" == "" ]]; then
    echo "✗ ERROR: Virtual environment not active!"
    echo "Run: source docs_env/bin/activate"
    exit 1
fi

# Navigate to docs directory
cd docs/

# Clean any previous builds
rm -rf build/

# Build HTML documentation (within virtual environment)
sphinx-build -b html source build/html

# Check for build errors (return code should be 0)
if [ $? -eq 0 ]; then
    echo "✓ Documentation built successfully in virtual environment"
else
    echo "✗ Build failed - check error messages above"
    exit 1
fi
```

### Step 4: Validation Checklist
**The agent must verify ALL of these automatically**:

#### File Existence Check:
- [ ] `docs/source/conf.py` exists and contains required extensions
- [ ] `docs/source/index.md` exists with proper toctree
- [ ] `docs/.readthedocs.yaml` exists with correct configuration
- [ ] `docs/requirements.txt` exists with all dependencies
- [ ] All section files exist in their respective directories

#### Build Validation:
- [ ] Sphinx build completes without errors (exit code 0)
- [ ] HTML files are generated in `build/html/`
- [ ] `build/html/index.html` exists and is valid HTML
- [ ] No broken internal links (check with `sphinx-build -b linkcheck`)
- [ ] All code examples have proper syntax highlighting

#### Content Validation:
- [ ] All API functions are documented with parameters and return types
- [ ] Code examples are complete and runnable
- [ ] Cross-references work correctly
- [ ] Search index is generated properly

### Step 5: Automated Testing Commands
**The agent should run these commands to validate everything**:

```bash
# VERIFY VIRTUAL ENVIRONMENT IS ACTIVE FOR ALL TESTS
check_venv() {
    if [[ "$VIRTUAL_ENV" == "" ]]; then
        echo "✗ ERROR: Virtual environment not active for testing!"
        echo "Run: source docs_env/bin/activate"
        exit 1
    fi
    echo "✓ Virtual environment active: $VIRTUAL_ENV"
}

# 1. Test Sphinx configuration (in virtual environment)
check_venv
python -c "
import sys
sys.path.insert(0, 'docs/source')
import conf
print('✓ Sphinx configuration loads successfully')
print(f'✓ Extensions: {conf.extensions}')
print(f'✓ Theme: {conf.html_theme}')
"

# 2. Test documentation build (in virtual environment)
check_venv
cd docs/
sphinx-build -W -b html source build/html
if [ $? -eq 0 ]; then
    echo "✓ Documentation builds successfully"
else
    echo "✗ Documentation build failed"
    exit 1
fi

# 3. Test link checking (in virtual environment)
check_venv
sphinx-build -b linkcheck source build/linkcheck
if [ $? -eq 0 ]; then
    echo "✓ No broken links found"
else
    echo "⚠ Some links may be broken (check build/linkcheck/output.txt)"
fi

# 4. Verify key files exist
required_files=(
    "build/html/index.html"
    "build/html/installation.html"
    "build/html/quickstart.html"
    "build/html/api/rl_extensions.html"
    "build/html/examples/basic_usage.html"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file exists"
    else
        echo "✗ Missing: $file"
        exit 1
    fi
done

# 5. Test that HTML is valid and contains expected content (in virtual environment)
check_venv
python -c "
import os
from pathlib import Path

html_file = Path('build/html/index.html')
if html_file.exists():
    content = html_file.read_text()
    if 'PYRVO2-RL' in content and 'RL extensions' in content:
        print('✓ Index page contains expected content')
    else:
        print('✗ Index page missing expected content')
        exit(1)
else:
    print('✗ Index HTML file not found')
    exit(1)
"

echo "✓ All documentation validation tests passed!"
```

### Step 6: ReadTheDocs Preview Test
**Final validation step**:
```bash
# Test that the build works in a clean environment (simulates ReadTheDocs)
deactivate 2>/dev/null || true  # Exit current venv if active
python -m venv test_readthedocs_env
source test_readthedocs_env/bin/activate
pip install --upgrade pip

# Install documentation dependencies
pip install -r docs/requirements.txt

# Try to install the package itself (may fail if build deps missing, that's OK for docs)
pip install . 2>/dev/null || echo "⚠ Package install failed, but docs may still build"

# Build documentation in clean environment
cd docs/
sphinx-build -b html source build/html

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo "✓ Documentation builds in clean environment (ReadTheDocs simulation)"
else
    echo "✗ Clean environment build failed"
    exit 1
fi

# Cleanup
deactivate
cd ..
rm -rf test_readthedocs_env

echo "✓ ReadTheDocs simulation complete"
```

### Failure Recovery Instructions
**If any validation fails, the agent should**:
1. Check build logs for specific error messages
2. Verify all required files are present
3. Ensure dependencies are correctly specified
4. Fix any syntax errors in configuration files
5. Re-run validation until all tests pass

**Success Criteria**: All validation commands must return exit code 0 and all checkboxes must be verified before considering the documentation complete.

---

## Section 8: Intermediate Goals & Progress Tracking
**The agent should work incrementally and validate progress at each stage**:

### Phase 1: Infrastructure Setup (MUST COMPLETE FIRST)
**Goal**: Create and validate basic documentation infrastructure
**Deliverables**:
- [ ] `docs/` directory structure created
- [ ] `docs/source/conf.py` with working Sphinx configuration
- [ ] `docs/requirements.txt` with all dependencies
- [ ] `docs/.readthedocs.yaml` with correct ReadTheDocs config
- [ ] Basic `docs/source/index.md` with project title and placeholder sections

**Validation Command**:
```bash
# MUST WORK IN VIRTUAL ENVIRONMENT
source docs_env/bin/activate  # Ensure environment is active
cd docs/
sphinx-build -b html source build/html
test -f build/html/index.html && echo "✓ Phase 1 Complete" || echo "✗ Phase 1 Failed"
```

### Phase 2: Core Content Structure (BUILD ON PHASE 1)
**Goal**: Create all content files with basic structure and navigation
**Deliverables**:
- [ ] All section files created (`installation.md`, `quickstart.md`, etc.)
- [ ] Proper `{toctree}` navigation in `index.md`
- [ ] Each file has basic headers and placeholders
- [ ] Cross-references between sections work

**Validation Command**:
```bash
# MUST WORK IN VIRTUAL ENVIRONMENT  
source docs_env/bin/activate  # Ensure environment is active
cd docs/
sphinx-build -b linkcheck source build/linkcheck
grep -q "Total" build/linkcheck/output.txt && echo "✓ Phase 2 Complete" || echo "✗ Phase 2 Failed"
```

### Phase 3: RL Extensions Documentation (PRIORITY CONTENT)
**Goal**: Complete documentation of RL-specific features
**Files to read**: `src/bindings/rl_bindings.cpp`, `src/include/RVO2_RL_Wrapper.h`
**Deliverables**:
- [ ] Complete API reference for `RVO2RLWrapper` class
- [ ] All observation modes documented with examples
- [ ] Data structures explained with usage patterns
- [ ] Constructor parameters fully documented

**Validation Command**:
```bash
# Check that RL API content exists (in virtual environment)
source docs_env/bin/activate  # Ensure environment is active
cd docs/build/html/
grep -q "RVO2RLWrapper" api/rl_extensions.html && \
grep -q "ObsMode" api/rl_extensions.html && \
echo "✓ Phase 3 Complete" || echo "✗ Phase 3 Failed"
```

### Phase 4: Usage Examples (PRACTICAL CONTENT)
**Goal**: Provide working examples for researchers
**Files to read**: `tests/obs/` directory contents
**Deliverables**:
- [ ] Basic usage example with minimal setup
- [ ] Multi-agent scenario example
- [ ] Observation space configuration examples
- [ ] All examples are tested and runnable

**Validation Command**:
```bash
# Extract and test code examples (in virtual environment)
source docs_env/bin/activate  # Ensure environment is active
cd docs/build/html/examples/
python -c "
import re
# Check that examples contain proper code blocks
with open('basic_usage.html', 'r') as f:
    content = f.read()
    if 'RVO2RLWrapper' in content and 'import' in content:
        print('✓ Phase 4 Complete')
    else:
        print('✗ Phase 4 Failed')
"
```

### Phase 5: Installation & Quick Start (USER ONBOARDING)
**Goal**: Enable researchers to get started quickly
**Deliverables**:
- [ ] Complete installation instructions for Linux/macOS
- [ ] Build dependency explanations
- [ ] 5-minute quickstart tutorial
- [ ] Troubleshooting section for common issues

**Validation Command**:
```bash
# Check installation content completeness (in virtual environment)
source docs_env/bin/activate  # Ensure environment is active
grep -q "pip install" docs/build/html/installation.html && \
grep -q "import rvo2_rl" docs/build/html/quickstart.html && \
echo "✓ Phase 5 Complete" || echo "✗ Phase 5 Failed"
```

### Phase 6: Technical Implementation (ADVANCED CONTENT)
**Goal**: Document advanced features and performance considerations
**Files to read**: `src/impl/RVO2_RL_Wrapper.cpp`, `src/impl/RayCastingEngine.cpp`
**Deliverables**:
- [ ] Architecture overview with diagrams
- [ ] Performance benchmarks and optimization notes
- [ ] OpenMP parallelization details
- [ ] Memory usage and scaling characteristics

**Validation Command**:
```bash
# Check technical content exists (in virtual environment)
source docs_env/bin/activate  # Ensure environment is active
grep -q "OpenMP" docs/build/html/technical/performance.html && \
grep -q "architecture" docs/build/html/technical/architecture.html && \
echo "✓ Phase 6 Complete" || echo "✗ Phase 6 Failed"
```

### Progress Tracking Requirements
**The agent MUST**:
1. **Complete phases sequentially** - don't skip ahead
2. **Validate each phase** before proceeding to the next
3. **Report progress** after each phase completion
4. **Fix failures** before moving forward
5. **Test incrementally** - build docs after each major section
6. **MAINTAIN VIRTUAL ENVIRONMENT** - never work outside the documentation environment

### Virtual Environment State Management
**For every major operation, the agent must**:

```bash
# Standard environment check function to use before ANY documentation command
ensure_docs_env() {
    if [[ "$VIRTUAL_ENV" == "" ]]; then
        echo "Activating documentation environment..."
        source docs_env/bin/activate
        if [[ "$VIRTUAL_ENV" == "" ]]; then
            echo "✗ Failed to activate virtual environment!"
            echo "Create environment first: python -m venv docs_env"
            exit 1
        fi
    fi
    echo "✓ Working in virtual environment: $(basename $VIRTUAL_ENV)"
}

# Use before every sphinx command, python command, or pip command
ensure_docs_env
```

**Examples of CORRECT usage**:
```bash
# CORRECT: Always check environment first
ensure_docs_env
sphinx-build -b html source build/html

# CORRECT: Check environment before Python commands
ensure_docs_env
python -c "import sphinx; print('Sphinx version:', sphinx.__version__)"

# WRONG: Running commands without environment check
sphinx-build -b html source build/html  # ✗ May use wrong Python/packages
```

### Failure Recovery Protocol
**If any phase fails**:
1. Run the specific validation command to identify the issue
2. Check build logs for specific error messages
3. Fix the identified problems
4. Re-run the phase validation
5. Only proceed when validation passes

**Phase Dependencies**:
- Phase 2 requires Phase 1 completion
- Phase 3-6 can run in parallel after Phase 2
- Final validation (Section 7) requires ALL phases complete

---

## Section 9: Critical Information Gaps & Requirements
**The agent must address these shortcomings in the current description**:

### Missing Technical Details
1. **Return Types & Parameters**: The current description lacks specific type information
   - **Action Required**: Read header files to document exact function signatures
   - **Example**: What does `get_observation()` return? NumPy array shape? Data type?

2. **Error Handling**: No mention of exceptions or error conditions
   - **Action Required**: Document what happens when invalid parameters are used
   - **Example**: What if `max_neighbors=0` or `lidar_range=-1`?

3. **Performance Characteristics**: Missing concrete performance data
   - **Action Required**: Document approximate agent counts, memory usage, timing
   - **Example**: "Supports up to X agents with Y GB RAM"

### Missing Research Context
1. **RL Integration Examples**: No connection to popular RL libraries
   - **Action Required**: Show integration with Gymnasium, Stable-Baselines3, etc.
   - **Example**: How to wrap as a Gymnasium environment?

2. **Observation Space Definitions**: Unclear what researchers actually receive
   - **Action Required**: Document exact observation space shapes and ranges
   - **Example**: `observation_space = Box(low=-1, high=1, shape=(N,))`

3. **Action Space Definition**: Missing action format specification
   - **Action Required**: Document how actions are applied to the simulation
   - **Example**: "Actions are 2D velocity vectors normalized to [-max_speed, max_speed]"

### Missing Practical Information
1. **Build Requirements**: Vague dependency information
   - **Action Required**: Specify exact Ubuntu/macOS package names
   - **Example**: `sudo apt-get install build-essential libomp-dev`

2. **Known Limitations**: No mention of current constraints
   - **Action Required**: Document what doesn't work yet
   - **Example**: "Obstacle support is limited to static polygons"

3. **Comparison with Alternatives**: Missing competitive analysis
   - **Action Required**: Brief comparison with other multi-agent simulators
   - **Example**: "Vs. SUMO: Better for RL, vs. Mesa: Better performance"

### Agent Research Tasks
**Before writing each section, the agent should**:

#### For RL Extensions Section:
```bash
# Agent should look for these specific patterns in the code:
grep -r "def get_" src/bindings/rl_bindings.cpp    # Find observation methods
grep -r "return" src/include/RVO2_RL_Wrapper.h     # Find return types  
grep -r "throw\|exception" src/impl/               # Find error handling
```

#### For Usage Examples Section:
```bash
# Agent should analyze test patterns:
find tests/ -name "*.py" -exec grep -l "RVO2RLWrapper" {} \;  # Find usage patterns
grep -r "import" tests/                                        # Find required imports
grep -r "assert" tests/                                        # Find expected behaviors
```

#### For Installation Section:
```bash
# Agent should check build requirements:
grep -r "cmake\|make\|gcc" .                       # Find build tools needed
grep -r "apt-get\|brew\|yum" .                     # Find system dependencies
```

### Content Quality Requirements
**Each section must include**:
1. **Complete examples** - No partial code snippets
2. **Expected outputs** - Show what users should see
3. **Error scenarios** - What goes wrong and how to fix it
4. **Performance notes** - Memory/CPU requirements for different scales
5. **Cross-references** - Links to related sections

### Research Context Requirements
**The documentation must explain**:
1. **Why use this over alternatives** - Unique value proposition
2. **When NOT to use this** - Limitations and better alternatives
3. **Integration patterns** - How it fits into RL workflows
4. **Scaling considerations** - From prototype to large experiments

---

## Section 10: Environment Management & Dependencies
**The agent must handle dependencies safely and reproducibly**:

### Virtual Environment Strategy
**Why virtual environments are essential**:
1. **Isolation**: Prevents conflicts with system packages
2. **Reproducibility**: Ensures consistent builds across different machines
3. **Safety**: Won't break user's existing Python setup
4. **ReadTheDocs compatibility**: Matches how ReadTheDocs builds documentation

### Environment Setup Protocol
**The agent should follow this exact sequence**:

```bash
# 1. Create documentation environment
python -m venv pyrvo2_docs_env
source pyrvo2_docs_env/bin/activate  # Linux/macOS
# pyrvo2_docs_env\Scripts\activate  # Windows (agent should detect OS)

# 2. Upgrade pip to latest version
pip install --upgrade pip

# 3. Install documentation dependencies
pip install sphinx>=5.0.0 myst-parser>=0.18.0 sphinx-rtd-theme>=1.0.0 sphinx-autodoc-typehints>=1.19.0

# 4. Verify installations
python -c "
import sphinx, myst_parser, sphinx_rtd_theme
print(f'✓ Sphinx: {sphinx.__version__}')
print(f'✓ MyST-Parser: {myst_parser.__version__}')
print('✓ All documentation dependencies installed successfully')
"
```

### Dependency Management Files
**The agent must create these files for reproducible builds**:

#### `docs/requirements.txt`
```txt
# Core Sphinx dependencies
sphinx>=5.0.0,<8.0.0
myst-parser>=0.18.0,<3.0.0
sphinx-rtd-theme>=1.0.0,<3.0.0
sphinx-autodoc-typehints>=1.19.0,<2.0.0

# Optional but recommended
sphinx-copybutton>=0.5.0
sphinx-design>=0.3.0
```

#### `docs/requirements-dev.txt` (for contributors)
```txt
# Include base requirements
-r requirements.txt

# Development tools
sphinx-autobuild>=2021.3.14
doc8>=0.11.0
rstcheck>=6.0.0
```

### Environment Validation Commands
**The agent should run these to ensure environment is working**:

```bash
# Test 1: Check Python version compatibility
python -c "
import sys
if sys.version_info >= (3, 10):
    print(f'✓ Python {sys.version} is compatible')
else:
    print(f'✗ Python {sys.version} is too old (need >=3.10)')
    exit(1)
"

# Test 2: Verify all documentation dependencies
python -c "
try:
    import sphinx
    import myst_parser
    import sphinx_rtd_theme
    print('✓ All required packages imported successfully')
except ImportError as e:
    print(f'✗ Missing dependency: {e}')
    exit(1)
"

# Test 3: Test basic Sphinx functionality
python -c "
from sphinx.application import Sphinx
from pathlib import Path
print('✓ Sphinx can be instantiated')
"
```

### Cleanup Protocol
**The agent should clean up environments after completion**:

```bash
# Function to cleanup documentation environment
cleanup_docs_env() {
    echo "Cleaning up documentation environment..."
    deactivate 2>/dev/null || true
    rm -rf pyrvo2_docs_env test_readthedocs_env
    echo "✓ Documentation environments cleaned up"
}

# Call cleanup on script exit
trap cleanup_docs_env EXIT
```

### Dependency Integration with Existing Project
**Key considerations for the agent**:

1. **Don't modify existing setup.py**: Documentation dependencies are separate from build dependencies
2. **Create separate requirements files**: Keep docs deps isolated in `docs/requirements.txt`
3. **Check for conflicts**: Ensure docs dependencies don't conflict with project dependencies
4. **Version pinning**: Use compatible version ranges, not exact pins

### Platform-Specific Considerations

#### Linux (Primary platform):
```bash
# Check if system packages are needed
python -c "
import subprocess
import sys
try:
    import sphinx
    print('✓ Can import sphinx on Linux')
except ImportError:
    print('Installing documentation dependencies...')
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'sphinx', 'myst-parser', 'sphinx-rtd-theme'])
"
```

#### macOS:
```bash
# Handle potential SSL certificate issues
python -c "
import ssl
import certifi
print(f'✓ SSL context available')
print(f'✓ Certificates: {certifi.where()}')
"
```

### Environment Troubleshooting
**Common issues the agent should handle**:

1. **Permission errors**: Use `--user` flag if venv creation fails
2. **Network issues**: Add timeout and retry logic for pip installs
3. **Version conflicts**: Clear pip cache if dependency resolution fails
4. **Path issues**: Use absolute paths for cross-platform compatibility

### ReadTheDocs Environment Simulation
**To ensure compatibility, the agent should test with ReadTheDocs-like constraints**:

```bash
# Simulate ReadTheDocs build environment
python -m venv rtd_test_env
source rtd_test_env/bin/activate

# Install only what ReadTheDocs would install
pip install --upgrade pip
pip install -r docs/requirements.txt

# Test build without installing the main package
cd docs/
sphinx-build -W -b html source build/html

# Verify critical files exist
test -f build/html/index.html || exit 1
test -f build/html/_static/css/theme.css || exit 1

echo "✓ ReadTheDocs environment simulation passed"
deactivate
rm -rf rtd_test_env
```

---

## Formatting Guidelines for Documentation Agent

### Sphinx Configuration Requirements
1. **conf.py Setup**: Configure MyST-Parser, theme, and extensions
2. **ReadTheDocs Integration**: YAML config for automatic building
3. **Cross-References**: Use proper Sphinx directives for linking
4. **Code Documentation**: Include docstring examples for API functions

### File Organization Standards
- **index.md**: Overview + navigation to all sections
- **installation.md**: Build instructions + dependency management
- **quickstart.md**: 5-minute tutorial for researchers
- **API sections**: Comprehensive reference with examples
- **Examples**: Practical tutorials with complete code
- **Technical**: Implementation details for advanced users

### Content Standards
- **Code Examples**: All examples must be runnable
- **API Documentation**: Include parameter types, return values, exceptions
- **Cross-Platform Notes**: Highlight Linux/macOS differences where relevant
- **Performance Metrics**: Include timing/memory usage where applicable

### ReadTheDocs Integration
- **Automatic Building**: Push to GitHub triggers docs rebuild
- **Version Management**: Support multiple versions (stable/latest)
- **Search Integration**: Sphinx search functionality
- **Mobile Responsive**: RTD theme provides mobile support

### Output Format Requirements
- Use MyST-Markdown syntax for all content files
- Include proper Sphinx directives: `{class}`, `{function}`, `{note}`, etc.
- Add table of contents with `{toctree}` directives
- Provide both conceptual explanations and practical code snippets
- Include downloadable example scripts in examples/ sections
