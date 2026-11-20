Official Kai0 AgileX ROS1 DAgger collector: local SSH compatibility copy

Source: https://github.com/OpenDriveLab/kai0.git
Commit: 9d93078c757840f50e75248c5c5a94ab7b41e13a
Upstream directory: train_deploy_alignment/dagger/agilex

These two Python files are isolated copies of the official collector. The
original /Users/kl/kai0 checkout has not been modified. upstream-manifest.json
records exact original and patched SHA-256 hashes. compatibility.patch is the
complete code delta, with paths relative to this directory.

Explicit runtime and I/O boundary fixes (official policy, control, and data
schema preserved):

The optional run_dagger_pedal.sh entry wraps the unchanged original Bash flow.
Its USB reader uses private pipe tokens, never global OS key injection. The same
collector action handler handles keyboard and pedal actions; no pipe environment
means the original keyboard flow. A readiness pipe prevents queued startup pedal
presses. Left chooses d or an existing post-save recording request; single presses
during save confirmation only confirm; the delete token works only in that
window. Double-middle deletes only in confirmation; double-right stop is handled
by the outer owner through the original foreground Ctrl+C path, never as an r
command. Middle/right singles wait a 0.5s double-press window; doubles never
first emit a single action. Calibration verifies three complete press/releases
per pedal, without requiring a hold duration.

Both normal Bash entries now enable arms whose complete normal native feedback
shows disabled/partial-enabled motors during preparation. The startup helper
preloads the current measured six-joint pose, restores CAN1 there when needed,
sends EnableArm(7), and verifies the actual six enabled bits using the existing
feedback criteria. It does not disable, reset, zero or command a gripper. Already
enabled CAN1 arms remain passive; --check/--stop and Ctrl+C do not auto-enable.
This startup change does not alter the collector's policy, recording or handoff.

The formal keyboard and pedal entries now default to releasing both master
grippers at the FC-to-FA boundary. run_dagger_gripper_handoff_test.sh remains a
compatible alias (--pedal selects the pedal entry). The legacy
DAGGER_TEST_MASTER_GRIPPER_RELEASE selector defaults to both when unset; explicit
right still selects only the right side, and explicit empty/other values retain
the old ordinary release request. The default path sends one release per master gripper at that side's
existing prepared native actual width, after the normal FA phase is armed and before its role
request. It adds no gripper acknowledgement wait, extra feedback gate, second
follower check, FC pause flag or additional recovery branch. Existing controls
and failure handling remain. r adds no gripper restoration write; the next
alignment uses the original gripper position control. Promotion changes the
default selection only; it does not establish physical firmware behavior.
Native FA commands are not actual gripper feedback. Save does not
switch the masters to a fixed holding state.

Saving with s or the middle pedal now stops correction capture and pauses native
FA target publication on both masters before export or its confirmation prompt.
Followers retain their last accepted joint/gripper position targets; master
motion no longer supplies new targets while paused. Confirmation, deletion and
an empty save request keep following paused. Existing in-flight targets may
finish. This uses pause_following/resume_following on the existing master config
topics, with no extra SDK write, mode switch, enable/disable, ACK wait, timeout or
pose gate. Space or the left pedal resumes publication only when a new recording
actually starts. FC priming and r/policy output keep their existing paths, and a
new native phase starts with publication enabled.

1. Honor --dataset_dir when creating both inference and DAgger output paths.
   Both still use <dataset_name>_{inference,dagger}_hdf5/aloha_mobile_dummy.
2. Use the existing terminal keyboard_monitor_thread for the combined collector.
   Remove its redundant desktop pynput import/listener, which otherwise fails
   during SSH import or races with the terminal Space key behavior.
   collect_data.py imports pynput only inside its standalone keyboard_listener;
   its save_data and video functions remain importable without DISPLAY/pynput.
3. On d, keep the existing inference capture worker running for an extra1.0s
   after the publisher loop has entered takeover. Use monotonic wall duration,
   then set its stop flag before join(timeout=5 seconds). These are actual new
   samples in the inference segment, not a delayed write of an already stopped
   buffer or additional policy execution. Ordinary stop/finally defaults to no
   tail wait; shutdown cancels the tail and prevents subsequent master commands.
   No fixed frame count, timestamp fabrication or transition-stage relabeling is
   added. If the worker is stuck or failed, raise before enabling or aligning masters.
   Explicitly reject empty capture rather than reporting DAgger entry success.
4. On r, stop correction capture, clear its pending collection flag, then start
   a fresh inference recording with new timesteps/actions/result and stop=False.
   This permits repeated d/r corrections; save_data consumes the old lists.
5. Only mark DAgger mode entered after successful upstream alignment/entry.
   A failed entry raises instead of continuing in a falsely active mode.
6. Stop/join the inference capture worker when model_inference exits, then save
   its unsaved real inference observations/actions and camera videos. No exit
   tail wait or additional robot command is sent. d and exit use the same
   inference writer and episode numbering; completed saves are not repeated.
   Empty/actionless segments are skipped and correction buffers are untouched.
   A normal ROS shutdown interrupt preserves the sampled prefix. Saving uses
   shallow list copies so an interrupted write can be retried at the same path.
   The launcher stops the arm drivers before signalling the collector, drains
   its PTY during export, and lets the first Ctrl+C wait for completion; a second
   Ctrl+C requests forceful exit. This applies to keyboard and pedal entries.
7. Normalize all 12 ROS color-image bridge calls to bgr8, matching the official
   inference BGR-to-RGB conversion. This accepts either rgb8 or bgr8 ROS input,
   including the ROS1 RealSense driver 2.3.2 rgb8 output. The nine depth-image
   bridge calls remain passthrough; depth values are not color-converted.
8. Convert normalized BGR color frames to RGB immediately before video export,
   matching the existing av.VideoFrame(..., format="rgb24") writer. Upstream
   passed 3-channel frames unchanged, which would swap red/blue in the recording.
   HDF5 array names/shapes and companion MP4 layout/codecs remain unchanged.
9. Require fresh dual-master mode acknowledgements before completing d or r.
   d waits for /teach/master_control_status_{left,right} Twist with angular.y=250
   and linear.z=1. r remains a pending request with inference paused, publishes
   config=slave to both masters, and waits for fresh angular.y=252 / linear.z=0
   plus 0.25s with no messages on either follower command topic. r retains its15s
   deadline; d waits without a fixed handoff deadline and Ctrl+C can stop it.
   Teaching entry requires
   both command topics observed after this request, not a continuous sub0.25s
   packet interval. A stale/missing mode ack or
   continuing publisher leaves policy paused. An r handoff timeout is reported
   within the existing session, retaining collected data and allowing a later
   explicit r retry, without exiting the round. Repeated r keys do not queue
   transitions while a handoff is pending/in progress. This also rejects
   a mode_master=True node whose unconditional command publishing never stops.
10. After successful handoff, clear cached observation/actions, last stream
    action and filter state. A generation guard under a shared lock prevents an
    inflight pre-intervention policy reply from repopulating the cleared stream.
    Both stream and blocking modes wait for fresh post-resume inference actions;
    interpolation starts from the new observation rather than the old action.
11. Before the official initial pose, wait up to15s for fresh three-camera and
    two-follower joint inputs. Before every initial interpolation step and actual
    policy joint/eef/base publish, require latest color headers no older than0.5s
    and follower joint headers no older than0.25s. Timestamp/age must be finite;
    future headers beyond0.05s are rejected. Independent callback timestamp
    storage survives destructive deque reads. Failure raises RuntimeError on the
    collector main path and stops collector output; it sends no enable, disable,
    stop or hold command. Human DAgger control, master alignment and saving remain
    outside this guard. The isolated ROS driver also has the teaching-entry
    follower-readiness interlocks described below.
12. d retains the intermediate-pose/current-follower alignment sequence. After
    final arrival verification, concurrently keep publishing that final target
    with fresh actual six-joint checks, and prepare following on a separate
    prepare-only worker. The worker probes fixed follower poses, checks actual
    subscriptions/raw CAN, sends prepare_master and waits for fresh dual prepared
    status. Drivers actively forward native FC actual feedback and get dual ACKs.
    Only three continuous enabled/stable seconds AND currently live preparation
    permit the main thread to send release_master. Wait is max(3s, preparation),
    not their sum; overall deadline15s. A worker never releases FA, even if it
    completes after cancellation. A transient unready state keeps holding, while
    fault/timeout/cancel prevents release. FA remains the physical release point.
    Alignment/prepared-target checks cover the six joints only. Gripper position
    differences do not reject entry; gripper following remains active and retains
    its WIDTH, native-frame freshness and control-limit validation.
    A receive-only native listener and independent forwarding loop are armed
    first; post-FA frames forward immediately with their original CAN timestamps.
    Managed FA entry has no repeated Enable/teach command or six-second sleep.
    FA acknowledgement waits for native publication and its follower receipt.
    Both masters wait for the first complete fresh FA group and both follower
    source acknowledgements without fixed deadlines. The added2s first-group and
    2.5s pair-ACK deadlines, and the collector's15s FA handoff timeout, are removed.
    Forward immediately when data arrives; existing shutdown/manual cancellation
    can end the wait. FC priming retains its0.25s missing-data failure. FA silence
    after the first complete group, including before handoff acknowledgement,
    waits in the same phase;0.25s only triggers a waiting notice. Component
    freshness and full-group validation remain unchanged. Additional1-degree
    recovery/actual-priming and2-degree first-FA/gap target gates are removed.
    FA already releases the master, so this wait does not keep it position-held
    or guarantee following while the first group has not yet arrived.
    After successful FA acknowledgement, native silence is recoverable waiting,
    never a0.25s permanent gate. Keep the same receive phase and do not republish
    old targets or refresh source timestamps. Resume only with complete fresh
    physical RX, without an added2-degree target-difference gate after a gap.
    Invalid frames, listener faults and manual cancellation still stop forwarding.
    Existing mode-status text reports waiting/resumed input without CAN writes.
    Do not send a separate concurrent early teach command. After both master mode
    acknowledgements and fresh post-request commands from both master publishers,
    automatically start correction recording, prepend the preceding one second
    of real camera/follower samples, and set collection_active. Space is not needed for this first
    segment; it remains optional to restart recording after saving a segment.

The one-second DAgger prefix is relative to the original FA-confirmed recording
start, not one second before FA release and not the entire handoff wait. After
inference capture has stopped/joined/saved, a separate data-only worker samples
non-destructive snapshots of the three cameras and both actual follower streams.
It keeps a rolling one-second buffer. At the original ready point, use the same
ROS clock as the source headers to select [ready-1s, ready], stop/join the worker,
then run the original start_collection and add_frame sequence with those samples.
The live path skips already prepended source frames. FIRST/MID and next-actual-
state action pairing, camera order and BGR encoding remain unchanged. This adds
no fixed wait, hardware command, ACK, timeout or pose check. Failed/interrupted
entry stops and clears this worker; save/restart does not replay an old prefix.
Missing samples are not padded. The separate inference tail remains one second.

Runtime recording checks existing fresh FA/teach/enabled mode acknowledgements.
An invalid/expired mode pauses appending without clearing the segment, starting
a replacement segment, exiting DAgger or resuming policy. Quiet control topics
alone are not a fault; synchronized camera/follower data can still record a held
pose. The unused master queues no longer block correction frame synchronization;
no master data or timestamps are fabricated to fill them.

Follower preparation checks actual fresh enabled feedback and transport, followed
by a command-source execution probe. ROS node-name prefixes are not a readiness
gate; stale cameras do not block that mechanical holding probe. Policy publishers
still require their normal fresh camera/follower inputs.

Keyboard controls are the official terminal controls and require an SSH TTY:
  d: pause/save policy, align masters, hold final pose3s, enter teaching and record
  Space: resume following and restart correction recording after saving in DAgger
  s: pause following, stop capture and save the correction episode
  w: discard the last saved correction during its confirmation prompt
  any other key at that prompt: confirm the correction
  r: request exclusive policy handoff, then begin a new inference recording

No CLI options were added. Explicitly pass a writable --dataset_dir, the desired
--dataset_name, and the official policy server --host/--port. Preserve video
export: upstream save_data asserts args.export_video. Do not use a dummy pynput
backend; the combined collector no longer invokes pynput.

Runtime dependencies remain ROS1 rospy/messages/cv_bridge/piper_msgs, numpy<2,
OpenCV, torch, h5py, dm_env, av, and the official openpi_client package. CPU torch
is sufficient for collector-side inference_mode/seeding. The official client
also needs Pillow, msgpack, typing_extensions.override, and a websockets release
supporting sync.client.connect(additional_headers=...); websockets 15.0.1 matches
that API. Its pyproject permits Python >=3.7 and numpy>=1.22.4,<2.0.0. No removed
NumPy scalar aliases (np.int/np.float/np.bool/etc.) appear in these collectors or
the checked-in official packages/openpi-client sources.

Hardware-free validation:
  python3 test_compatibility.py -v

22 tests passed locally. The tests compile/extract actual source methods with
AST and fake dependencies, so importing the test cannot initialize ROS, open
CAN/cameras, or move arms. They check the dataset root, stop-before-bounded-join,
timeouts/errors/empty capture preventing entry, duplicate-worker rejection,
fresh data/result buffers, two d/r capture cycles with separate episode numbers,
the model loop's r transition and failed-entry branch, lazy pynput import, and
the untouched upstream hashes, all color/depth boundaries, and synthetic rgb8 /
bgr8 color sentinels through actual frame getters and the policy/video conversion
expressions. Both copied Python files compile successfully.
Additional checks execute the real TTY r handler to prove inference remains
paused, require fresh mode feedback and command silence, reject continuing master
publishers, verify handoff before capture restart, and reject an old generation
reply while accepting a fresh policy reply through the actual background loop.
Freshness checks cover missing/stale/future/nonfinite data on every input with
zero output commands, fresh data after deque consumption, startup refusing all
initial motion without fresh input, and camera loss between interpolation steps
preventing the next step.

The separate negative integration runner is:
  ../validation/validate_stale_policy_input.py --collector ./agilex_openpi_dagger_collect.py

Run it under the sourced ROS environment, separately from the positive harness
(both reserve loopback ports11321/11322). It uses the actual collector and policy
server with synthetic ROS inputs, suppresses only the right camera after two
model replies/policy output, and requires the specific freshness error plus
collector exit within2s and no policy commands after exit. Other fake sensors
continue publishing. Its report marks hardware_io=False / validation_only=True;
it never opens CAN/cameras or starts hardware nodes. It was syntax-checked locally
and must be executed in the remote ROS environment before claiming integration
validation.

An additional real-library test is available for the ROS environment:
  python3 test_color_io.py -v

It constructs rgb8 and bgr8 sensor_msgs/Image messages with CvBridge, runs actual
extracted getters and inference_fn, checks the model CHW RGB payload, then uses
the actual save_data/PyAV writer and decodes each MP4 as rgb24 to check red/green/
blue sentinels. It also checks unchanged 16UC1 depth values and HDF5 action shape.
It does not initialize a ROS node, create an interface, open CAN, or make a model
server request. Network infer/resize are local stand-ins; color conversion and
video encoding/decoding are real. This test skips if its optional ROS and media
libraries are unavailable (as on the local macOS test host).

These tests do not establish real ROS topic/image encodings, camera/CAN health,
master alignment, or motion behavior. The official startup motion and alignment
sequence remain unchanged; importing/calling main is not a hardware-free test.
No remote deployment or hardware command was performed to produce this copy.
