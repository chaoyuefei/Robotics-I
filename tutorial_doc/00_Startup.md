# Installation and Setup Guide
This repository uses [Drake](https://drake.mit.edu/) primarily through its Python bindings. 
<!-- Choose the setup that matches your system: -->

<!-- * On **macOS Tahoe (26) on Apple Silicon / arm64**, use a Python virtual environment and install Drake with `pip`. -->

## 1. Choose your platform
| System | Setup |
|---|---|
| **macOS Tahoe (Apple Silicon)** | Python virtual environment + `pip install drake` |
| **Windows** | Install **Ubuntu via WSL**, then follow the Ubuntu setup below |
| **Ubuntu / WSL** | Install Drake via **APT** |

## ⚡ Quick Start (Recommended)

If you are on:

- **macOS** → follow Section 2 (macOS), then Section 3  
- **Windows** → install WSL, then follow Ubuntu setup, then Section 3  
- **Ubuntu** → install Drake, then Section 3  

Estimated setup time: 10–20 minutes.
<!-- You do **not** need to build Drake from source to start the tutorials. -->

# 2. OS-specific Setup
## 🟣 macOS (Apple Silicon)
### 2.1. Check your Python version
Drake supports `pip` installs on macOS with Python `3.13` or `3.14`. The recommended version for Tahoe is Python `3.14`.

Check your Python version:
```bash
python3 --version
```

If needed, install a newer version using Homebrew:

```bash
brew install python@3.14
```
Verify:

```bash
/opt/homebrew/bin/python3.14 --version
```

<!-- ### 2. Clone the repository

```bash
cd ~
git clone https://github.com/chaoyuefei/Robotics-I.git
cd Robotics-I
``` -->

### 2.2 Create and activate a virtual environment
Using a virtual environment avoids conflicts with other Python packages on your machine. We recommend either:

* a standard Python virtual environment (`venv`), or
* [`uv`](https://docs.astral.sh/uv/) if you already use it.

Conda **is not recommended** for this course.

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

### 2.3 Install Drake and dependencies
Upgrade packaging tools first:
```bash
python -m pip install --upgrade pip setuptools wheel
```

Using `venv`:
```bash
python -m pip install drake ompl numpy matplotlib pydot ipython
```

If you are using `uv`:

```bash
uv pip install drake ompl numpy matplotlib pydot ipython
```

Notes:
If you also want PNG exports of system block diagrams, install Graphviz so the `dot` executable is available:
```bash
brew install graphviz
```

## 🟢 Windows (WSL)
Developers on Windows can run both Windows and Linux side-by-side using the Windows Subsystem for Linux (WSL).
This is the easiest way to get Ubuntu running without a full virtual machine.

> ⚠️ Requirements: Windows 10 (build 19041+) or Windows 11.
For older versions, see the [manual install page](https://learn.microsoft.com/en-us/windows/wsl/install-manual).

### 2.1 Install WSL
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

### 2.2 Check WSL version
   ```PowerShell
   wsl.exe --list --verbose
   ```
   If Ubuntu is running with `VERSION 1`, upgrade it:
   ```powershell
   wsl --set-version Ubuntu-22.04 2
   ```

### 2.3 First-time setup
The first time you start Ubuntu (from the Start menu), you’ll be asked to create a Linux username and password.
<div style="text-align: center;">
    <img src="images/ubuntUSER.png" alt="UbuntuUser">
</div>

### 2.4 Update packages
```bash
sudo apt update && sudo apt upgrade
```
## 🔵 Ubuntu / WSL (Drake Installation)
### 2.5 Ubuntu / WSL (Drake Installation)
1. Install Drake (APT):
   ```sh
   sudo apt-get update
   sudo apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget
   wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
   echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
   sudo apt-get update
   sudo apt-get install --no-install-recommends drake-dev
   ```
2. Set environment variables: add the following to your ~/.bashrc (or ~/.bash_aliases):

   ```sh
   export PATH="/opt/drake/bin${PATH:+:${PATH}}"
   export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
   ```

3. Reload your shell
   ```bash
   source ~/.bashrc
   ```

### 2.6 Install Python dependencies
```bash
sudo apt-get install python3-pip
pip install ompl numpy matplotlib pydot ipython
```

# 3. Common Setup (All Systems)

## 3.1 Install Git
1. macOS:
```bash
brew install git
```
2. Ubuntu / WSL:
```bash
sudo apt-get install git
```
<!-- 2. Set up SSH access to GitHub

   Since this repository is private, you need to configure SSH access with your GitHub account.
   Follow this step-by-step guide: 
   [👉 How to stup SSH keys for github cloning](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/github-clone-with-ssh-keys). -->

## 3.2 Clone the repository
```sh
cd ~
git clone https://github.com/chaoyuefei/Robotics-I.git
cd Robotics-I
```
<!-- for more details in case the previous is not enough::::
https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-git -->


## 3.3 Run the sanity check script:
```sh 
cd ~/Robotics-I/tutorial_scripts 
python3 tutorial_sanity_check.py
```
Expected output:
```sh 
INFO:drake:Meshcat listening for connections at http://localhost:7000
```
Open the printed link in your browser and you should see a robot visualized.

## 3.4 Recommended tools
* [VS Code](https://code.visualstudio.com/) 
* [WSL extension (for Windows users)](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) extension. 

That’s it! You now have Drake and this repository ready to run the tutorials.