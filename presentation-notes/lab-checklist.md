# LAB CHECKLIST — custom lane follower on duck4

Follow top to bottom. Every command is copy-paste. Terminal names in [brackets].
"[VM]" = terminal in the Ubuntu VM. "[BOT]" = a terminal already SSH'd into the robot.

IMPORTANT: we are NOT starting indefinite_navigation. That demo is broken on this bot
(missing led_emitter_node). We run our OWN follower instead. Ignore the demo entirely.

---------------------------------------------------------------------------
## PHASE 0 — what you actually need to calibrate (read, saves time)

Our follower uses the RAW camera image + HSV colors. It does NOT use ground
projection or rectification. Therefore:

- Camera INTRINSIC calibration  -> NOT used by our follower. Skip.
- Camera EXTRINSIC calibration  -> NOT used by our follower. Skip.
- WHEEL / kinematics (trim)     -> YES, matters. Quick check in Phase 4.
- HSV color thresholds          -> YES, must tune for the lab's lighting (Phase 5).

(The camera calibrations are still saved on the SD card and are not lost. If you later
go back to the official demo/ground projection, you'd need them again.)

---------------------------------------------------------------------------
## PHASE 1 — power on and get on the network

1. Plug in / power on the bot. Wait ~2-3 min for it to fully boot.
2. Turn on the Windows Mobile Hotspot, then start the VM.
3. [VM] If the VM has no network:
       sudo ip link set enp0s3 up
       sudo dhclient enp0s3
   (If dhclient hangs forever -> hotspot is OFF or bridged to the wrong adapter.)
4. [VM] Confirm the bot is reachable:
       ping -c 3 duck4.local
   Must get replies before continuing. If not, fix network first.

---------------------------------------------------------------------------
## PHASE 2 — verify the core stack is running

5. [VM] SSH into the bot (keep this terminal open all session — it's your brake):
       ssh duckie@duck4.local

6. [BOT] Check the core containers are up:
       docker ps --format "table {{.Names}}\t{{.Status}}"

   You NEED these Up: ros, duckiebot-interface, car-interface.
   If any are missing/stopped, start them IN THIS ORDER (Portainer or command line),
   waiting ~30s between each:
       docker start ros
       docker start duckiebot-interface
       docker start car-interface

7. LENS CAP OFF. (With it on the camera sees black and nothing works.)

8. [VM] New terminal — open ROS tools and confirm the camera is publishing:
       dts start_gui_tools duck4
   then inside that shell:
       rostopic hz /duck4/camera_node/image/compressed
   Should print ~30 Hz. Ctrl-C to stop. If nothing prints, the camera isn't running.

---------------------------------------------------------------------------
## PHASE 3 — get the two scripts onto the bot

The files live on Windows at E:\Claude\fuckassbot. Easiest reliable way is to paste
them in over SSH with a heredoc.

9. [BOT] Create the follower (type the first line, paste the FULL contents of
   lane_follower.py, then type EOF on its own line):
       cat > /data/lane_follower.py << 'EOF'
       ...paste entire lane_follower.py here...
       EOF

10. [BOT] Same for the calibrator:
       cat > /data/hsv_calibrator.py << 'EOF'
       ...paste entire hsv_calibrator.py here...
       EOF

11. [BOT] Verify both landed and aren't empty:
       ls -l /data/*.py

(Alternative if you can get the files into the VM first: from [VM] run
     scp lane_follower.py hsv_calibrator.py duckie@duck4.local:/data/  )

---------------------------------------------------------------------------
## PHASE 4 — wheel trim check (2 minutes, only calibration that matters)

12. [VM] Read the current trim:
        dts start_gui_tools duck4
    inside:
        rosparam get /duck4/kinematics_node/trim

13. Put the bot on the floor with clear space ahead. [VM] new terminal:
        dts duckiebot keyboard_control duck4 --cli
    Keep THAT terminal focused. Press the UP arrow to drive forward a short distance.
    Watch which way it drifts. Then release / stop.

14. If it drifts:
      - drifts RIGHT -> DECREASE trim (e.g. -0.01 -> -0.03)
      - drifts LEFT  -> INCREASE trim (e.g. -0.01 -> 0.01)
    In the gui-tools shell (this SETS the value, it is not additive):
        rosparam set /duck4/kinematics_node/trim -0.03
        rosservice call /duck4/kinematics_node/save_calibration
    Re-test with the arrow key. Repeat until it goes roughly straight. Close enough is fine —
    the lane follower corrects continuously anyway.

15. Close keyboard_control when done (Ctrl-C).

---------------------------------------------------------------------------
## PHASE 5 — tune the HSV colors (the important one)

Do this ON THE TRACK, under the lab's actual lighting, with the bot sitting in a lane
where yellow centerline + white edge line are both visible.

16. IMPORTANT: `dts start_gui_tools` runs a container ON YOUR VM, not on the robot. It has
    its own filesystem and will NOT see the bot's /data. So EXPECT to paste the script in.

    [VM] Start it (needs a GUI window for the sliders):
        dts start_gui_tools duck4

    Inside that shell, create the file (press Enter after the first line, paste the WHOLE
    hsv_calibrator.py, then type EOF on its own line and press Enter):
        cat > /tmp/hsv_calibrator.py << 'EOF'
        ...paste entire hsv_calibrator.py here...
        EOF

    Check it landed (should be ~170 lines, not 0):
        wc -l /tmp/hsv_calibrator.py

    Set env vars -- the save path matters, /data does not exist here:
        export VEHICLE_NAME=duck4
        export HSV_SAVE_PATH=/tmp/hsv_thresholds.json

    Run it:
        python3 /tmp/hsv_calibrator.py

17. Two windows open: "roi" (what the follower sees) and "mask".
    Keys:  1 = tune YELLOW,  2 = tune WHITE,  3 = tune RED,  s = SAVE,  q = quit.
    For each color: drag the sliders until the MASK window shows ONLY that color as
    white and everything else black. A few stray dots are fine; big blobs are not.
    Do all three, then press 's'.

18. When you press 's' it SAVES to /tmp AND PRINTS the JSON to the terminal.
    Copy that printed JSON block -- you now have to move it to the robot.

    [BOT] in your SSH terminal, write it to the real location:
        cat > /data/hsv_thresholds.json << 'EOF'
        ...paste the printed JSON here...
        EOF

    Verify:
        cat /data/hsv_thresholds.json
    (lane_follower.py reads /data/hsv_thresholds.json on startup. If it's missing it falls
     back to default thresholds and will likely NOT follow the lane properly.)

---------------------------------------------------------------------------
## PHASE 6 — run the lane follower

19. FIRST verify the command topic exists (one-time sanity check).
    [VM] in gui-tools:
        rostopic info /duck4/car_cmd_switch_node/cmd
    Look for kinematics_node listed as a Subscriber. If the topic doesn't exist at all,
    see TROUBLESHOOTING below.

20. Put the bot in a lane. Keep your [BOT] SSH terminal handy.

21. [BOT] Run the follower on the robot:
        docker exec -it duckiebot-interface bash
    inside the container:
        source /environment.sh
        export VEHICLE_NAME=duck4
        python3 /data/lane_follower.py

    It should print "running. Ctrl-C to stop." and the bot should start driving.

22. BRAKE = Ctrl-C in that terminal. It publishes zero velocity on shutdown.
    If it ever runs away: Ctrl-C, or physically pick it up / cut power.

23. Tune the gains by editing the constants at the top of /data/lane_follower.py
    (use `nano /data/lane_follower.py`, edit, Ctrl-O Enter Ctrl-X to save):
      - wanders, reacts too weakly       -> raise KP (4.0 -> 5.0 -> 6.0)
      - snakes / oscillates left-right   -> lower KP, or raise KD
      - too fast to control              -> lower V_BAR (0.12 -> 0.08)
      - "lane lost" spam / phantom lanes -> re-tune HSV (Phase 5)
    Re-run after each edit. Change ONE thing at a time.

---------------------------------------------------------------------------
## TROUBLESHOOTING

- "No module named cv2" or "No module named duckietown_msgs" inside the container:
  you're in the wrong container. Try instead:
      docker exec -it car-interface bash
  or run the follower from gui-tools in the [VM] (slower video, but works):
      dts start_gui_tools duck4 ; export VEHICLE_NAME=duck4 ; python3 lane_follower.py

- Bot doesn't move at all, but the script prints no errors:
  the command isn't reaching the wheels. Check in gui-tools:
      rostopic echo /duck4/car_cmd_switch_node/cmd
  If values ARE being published but wheels don't move, switch the follower to publish
  wheel commands directly: publish WheelsCmdStamped to /duck4/wheels_driver_node/wheels_cmd
  (loses trim/gain calibration but bypasses everything). Ask Claude to make that edit.

- Bot drives but ignores the lane: HSV thresholds are wrong for this lighting. Redo Phase 5.

- ROS commands hang / nodes vanish: hotspot dropped. Re-check Phase 1.

- Do NOT run `dts duckiebot demo ...` — that demo is broken on this bot (missing
  led_emitter_node) and will just crash. We are not using it.
