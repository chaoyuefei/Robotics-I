# Installation and Setup guide
This repository uses [Drake](https://drake.mit.edu/) primarily through its Python bindings. The easiest setup depends on your operating system:

* On **macOS Tahoe (26) on Apple Silicon / arm64**, use a Python virtual environment and install Drake with `pip`.

* On **Windows**, install Ubuntu via WSL and then use the Ubuntu instructions below.

* On **Ubuntu**, install Drake using the official APT packages.

## Recommended setup for this repository

For the tutorials in `tutorial_scripts/`, you only need:

* Python + Drake (`pydrake`)
* A few Python packages such as `numpy`, `matplotlib`, and `pydot`
* A browser for MeshCat visualization

You do **not** need to build Drake from source to start the tutorials.

## macOS Tahoe (26, arm64) setup

These instructions target **macOS Tahoe (26)** on Apple Silicon, following Drake's supported configuration.

### 1. Check your Python version

Drake supports `pip` installs on macOS with Python `3.13` or `3.14`, and the official supported configuration for Tahoe is Python `3.14`.

Check your Python version:

```bash
python3 --version
```

If your system Python is older, install a newer one first. A practical option is Homebrew Python:

```bash
brew install python@3.14
```

Then verify:

```bash
/opt/homebrew/bin/python3.14 --version
```

### 2. Clone the repository

```bash
cd ~
git clone https://github.com/chaoyuefei/Robotics-I.git
cd Robotics-I
```

### 3. Create and activate a virtual environment

Using a virtual environment avoids conflicts with other Python packages on your machine. We recommend either:

* a standard Python virtual environment (`venv`), or
* [`uv`](https://docs.astral.sh/uv/) if you already use it.

We do **not** recommend Conda as the default for this course material. Drake's documentation notes that Anaconda / Conda is not tested regularly, so compatibility issues are more likely.

#### Option A: `venv`

```bash
/opt/homebrew/bin/python3.14 -m venv .venv
source .venv/bin/activate
python --version
```

If `python3` already points to version `3.13` or `3.14`, you can also use:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

#### Option B: `uv`

Install `uv` first if needed:

```bash
brew install uv
```

Then create and activate the environment:

```bash
uv venv --python 3.14 .venv
source .venv/bin/activate
python --version
```

### 4. Install Drake and the Python dependencies used in this repo

Upgrade packaging tools first:

```bash
python -m pip install --upgrade pip setuptools wheel
```

Install Drake and the packages used by the tutorial scripts.

If you are using `venv`:

```bash
python -m pip install drake numpy matplotlib pydot ipython
```

If you are using `uv`:

```bash
uv pip install drake numpy matplotlib pydot ipython
```

Notes:

* `pydot` is used to generate some block-diagram images.
* If you also want PNG exports of system block diagrams, install Graphviz so the `dot` executable is available:

```bash
brew install graphviz
```

* `ipython` is imported by some scripts for display helpers.
* `tutorial_04_path_planner.py` also imports `ompl`, which is **optional** for the earlier tutorials and may require extra setup depending on your machine.

### 5. Run the sanity check

From the repository root:

```bash
cd tutorial_scripts
python tutorial_sanity_check.py
```

If Drake starts correctly, you should see output similar to:

```text
INFO:drake:Meshcat listening for connections at http://localhost:7000
```

Open the printed URL in your browser. You should see the robot appear in MeshCat.

### 6. Recommended first tutorials on macOS

Start with these first:

* `tutorial_sanity_check.py`
* `tutorial_02.py`
* `tutorial_03.py`

Leave `tutorial_04_path_planner.py` for later, because it adds the extra `ompl` dependency.

## Install Linux via WSL
Developers on Windows can run both Windows and Linux side-by-side using the Windows Subsystem for Linux (WSL).
This is the easiest way to get Ubuntu running without a full virtual machine.

> ⚠️ Requirements: Windows 10 (build 19041+) or Windows 11.
For older versions, see the [manual install page](https://learn.microsoft.com/en-us/windows/wsl/install-manual).

1) Open PowerShell as Administrator
Right-click PowerShell → Run as administrator. Then run:
   ```powershell
   wsl --install Ubuntu-22.04
   ``` 
   <!-- This installs WSL and sets up Ubuntu 22.04 as your Linux environment. -->
   If WSL doesn’t recognize the distro, try:

   ```powershell
   wsl --install -d Ubuntu-22.04
   ``` 
   then restart your machine.

2) Check your WSL version
   ```PowerShell
   wsl.exe --list --verbose
   ```
   If Ubuntu is running with `VERSION 1`, upgrade it:
   ```powershell
   wsl --set-version Ubuntu-22.04 2
   ```

3) The first time you start Ubuntu (from the Start menu), you’ll be asked to create a Linux username and password.
<div style="text-align: center;">
    <img src="images/ubuntUSER.png" alt="UbuntuUser">
</div>

4) Update your Linux packages
   ```bash
   sudo apt update && sudo apt upgrade
   ```
## Ubuntu: Install Drake
1. [Install the Drake Toolbox](https://drake.mit.edu/installation.html), preferably a [stable release](https://drake.mit.edu/apt.html#stable-releases), either locally or globally on your system. We recommend using the APT-based stable release:

   ```sh
   sudo apt-get update
   sudo apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget
   wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
   echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
   sudo apt-get update
   sudo apt-get install --no-install-recommends drake-dev
   ```
2. Environment variables: add the following to your ~/.bashrc (or ~/.bash_aliases):

   ```sh
   export PATH="/opt/drake/bin${PATH:+:${PATH}}"
   export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
   ```

3. Reload your shell
   ```bash
   source ~/.bashrc
   ```

## Ubuntu: Install Required Python Packages
Install additional Python dependencies:
```ssh
sudo apt-get install python3-pip
pip install numpy matplotlib pydot ipython
```

Optional for `tutorial_04_path_planner.py`:

```bash
pip install ompl
```

## Install Git and Clone the Repo
1. Install GIT :
```bash
sudo apt-get install git
```
<!-- 2. Set up SSH access to GitHub

   Since this repository is private, you need to configure SSH access with your GitHub account.
   Follow this step-by-step guide: 
   [👉 How to stup SSH keys for github cloning](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/github-clone-with-ssh-keys). -->

2. Clone this repository   
```sh
cd 
git clone https://github.com/Coryx99/Robotics-II.git
```
<!-- for more details in case the previous is not enough::::
https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-git -->


## Getting ready before starting
Navigate to the `tutorial_scripts/` folder.

- Run the sanity check script:
```sh 
cd ~/Robotics-II/tutorial_scripts 
python3 tutorial_sanity_check.py
```
Drake uses MeshCat for 3D visualization, therefore, when you run a script, you will see output like:
```sh 
INFO:drake:Meshcat listening for connections at http://localhost:7000
```
Open the printed link in your browser and you should see a robot visualized.

That’s it! You now have Ubuntu, Drake, and this repo ready to run the tutorials.
