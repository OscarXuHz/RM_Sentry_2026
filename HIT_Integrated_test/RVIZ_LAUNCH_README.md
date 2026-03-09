# RViz Launch Guide for HIT Integration (macOS SSH to Ubuntu)

## Problem
Running `rviz` directly over SSH fails with:
```
qt.qpa.xcb: could not connect to display
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

This is because RViz requires an X11 display and OpenGL support, which SSH does not automatically provide.

## Solution: Use VNC (Recommended)

### 1. **Ubuntu Server Setup (One-Time)**

The VNC server is already running on your Ubuntu machine at port **5901** (display `:1`).

Verify it's running:
```bash
vncserver -list
```

Expected output:
```
TigerVNC server sessions:

X DISPLAY #     PROCESS ID
:1              <some-pid>
```

To stop/restart VNC (if needed):
```bash
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

### 2. **macOS VNC Client Setup**

Install a VNC viewer on your Mac:
- **Built-in**: Use macOS's built-in Screen Sharing (open Finder → Go → Connect to Server → `vnc://ubuntu-host:5901`)
- **Free alternatives**: [TightVNC](https://www.tightvnc.com/), [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/), [VNC Viewer from AppStore](https://apps.apple.com/us/app/vnc-viewer/id1568806081)

### 3. **Launch RViz with VNC**

**Step 1**: Connect to VNC from your Mac:
```
vnc://ubuntu-host:5901
```
(Replace `ubuntu-host` with your Ubuntu machine's IP or hostname)

**Step 2**: Once connected, open a terminal in the VNC desktop and run the HIT integration launch:
```bash
cd /home/sentry_train_test/AstarTraining/HIT_Integrated_test
roslaunch HIT_intergration_test.launch enable_rviz:=true enable_ser2msg:=false
```

**Step 3**: RViz should appear in the VNC desktop window.

---

## Alternative: X11 Forwarding (Less Reliable for OpenGL)

If you prefer X11 forwarding instead of VNC:

### macOS Setup:
1. Install [XQuartz](https://www.xquartz.org/) on your Mac
2. Start XQuartz from Applications

### Ubuntu Server Setup:
Ensure X11 forwarding is enabled in SSH config:
```bash
sudo nano /etc/ssh/sshd_config
# Find and uncomment/set:
#   X11Forwarding yes
# Save and restart SSH:
sudo systemctl restart ssh
```

### Connect and Test:
```bash
# On macOS terminal:
ssh -Y user@ubuntu-host

# On Ubuntu (after SSH-ing in):
echo $DISPLAY  # Should show something like localhost:10.0
xclock        # If XQuartz window opens on your Mac, X11 forwarding works

# Then run RViz:
cd /home/sentry_train_test/AstarTraining/HIT_Integrated_test
roslaunch HIT_intergration_test.launch enable_rviz:=true enable_ser2msg:=false
```

⚠️ **Note**: X11 forwarding can be slow or unstable for OpenGL apps like RViz. VNC is recommended.

---

## Launch Arguments Reference

When launching, you can override default args:

```bash
roslaunch HIT_intergration_test.launch \
  enable_rviz:=true \
  enable_ser2msg:=false \
  enable_dstarlite:=false \
  map_resolution:=0.05 \
  robot_radius:=0.35
```

- **enable_rviz**: Start RViz visualization (`true`/`false`, default: `false`)
- **enable_ser2msg**: Enable serial decision node (`true`/`false`, default: `true`)
  - Set to `false` if no serial hardware is connected
- **enable_real_robot_transform**: Enable TF transforms (`true`/`false`, default: `true`)
- **enable_dstarlite**: Enable legacy D* Lite planner (`true`/`false`, default: `false`)
  - Keep `false` to use HIT planners by default
- **map_resolution**: Grid cell size in meters (default: `0.05`)
- **robot_radius**: Robot footprint radius in meters (default: `0.35`)

---

## Troubleshooting

### VNC connection refused
- Check VNC server is running: `vncserver -list`
- Restart VNC: `vncserver -kill :1 && vncserver :1 -geometry 1920x1080 -depth 24`

### RViz still crashes in VNC
- Ensure you have a window manager running in the VNC desktop
- Try: `vncserver -kill :1` then restart with a specific window manager:
  ```bash
  vncserver :1 -geometry 1920x1080 -depth 24 -exec startxfce4
  ```

### No topics visible in RViz
- Check LIDAR driver is publishing: `rostopic list | grep livox`
- Check SLAM is running: `rosnode list | grep hdl`
- Verify filtered pointcloud: `rostopic hz /test_scan`

---

## Quick Start Command

```bash
# On your Mac:
ssh -l sentry_train_test ubuntu-host
# In VNC desktop or via SSH, run:
roslaunch /home/sentry_train_test/AstarTraining/HIT_Integrated_test/HIT_intergration_test.launch enable_rviz:=true enable_ser2msg:=false
```
