# Robot Cell Control Center - Bug Audit

Audit date: 2026-06-10

## Fixed in this audit

### Critical / high impact

1. **Simulated AMR mode sent real AMR movement commands**
   - The box-filled path always published `PICK_FILLED_BOX`, even when
     `simulate_amr` was enabled.
   - Fixed: simulated mode now generates AMR events without publishing a real
     AMR movement command.

2. **Monitor-only mode still commanded the automatic box workflow**
   - `auto_coordinate` was checked by the legacy package path but ignored by
     the newer box-filled and arm-ready paths.
   - Fixed: automatic AMR and arm commands are blocked while monitor-only mode
     is selected.

3. **Box-filled events could start duplicate AMR jobs**
   - The coordinator published its own box-filled event, received it again
     through `cell/#`, and processed it a second time.
   - Fixed: box-filled events are deduplicated by event, job, color, and box.

4. **Configured arm delay was ignored**
   - `delayBeforeArmMs` was displayed and saved but not applied.
   - The intended helper referenced an undefined `meta` variable and
     recursively scheduled itself forever.
   - Fixed: arm start is delayed correctly and stale delayed starts are
     cancelled.

5. **STOP/reset did not cancel delayed or simulated actions**
   - A delayed AMR command or simulated AMR thread could continue after STOP
     or reset. Late AMR events or MQTT reconnects could also clear STOPPED.
   - Fixed: operations use a generation token, STOP is latched across late
     events/reconnects, and reset/box reset cancels stale actions.

6. **AMR acknowledgements could clear unrelated pending commands**
   - Any matching event name resolved every pending command expecting that
     event.
   - Fixed: acknowledgement resolution now matches `commandId` and/or `jobId`
     when provided.

7. **Late AMR/arm events could act on the wrong job**
   - Ready/DONE events were not checked against the active job.
   - Fixed: stale job events are consumed and ignored.

8. **Stop All always reported success and did not command the conveyor**
   - Fixed: STOP is sent to AMR and conveyor, arm abort is checked, and the API
     reports partial failure accurately.

### Runtime and dashboard defects

9. **Automatic MQTT startup called undefined `connect_mqtt()`**
   - Fixed to call `mqtt_connect()`.

10. **Network discovery crashed because `re` was not imported**
   - Fixed and added subnet octet validation.

11. **HTTP 4xx responses were treated as a connected subsystem**
    - Fixed: subsystem HTTP checks now require a 2xx or 3xx response.

12. **Failed AMR MQTT publish left a fake pending acknowledgement**
    - Fixed: the pending item is removed and the mission enters FAULT.

13. **Manual arm start could continue after `/job/wait-amr` failed**
    - Fixed: `/job/amr-arrived` is called only after a successful wait step.

14. **External `cell/conveyor/cube` messages were ignored by counters**
    - Fixed with loop prevention for the coordinator's own cube messages.

15. **Dashboard never showed the active mission**
    - JavaScript used `active_box_name`, while the API sends `activeBoxName`.
    - Fixed.

16. **Dashboard logs omitted the newest entries after the buffer grew**
    - Fixed to display the newest 120 entries.

17. **Dashboard contained repeated function definitions**
    - Four versions of save/clear functions existed; only the last silently
      won.
    - Fixed by keeping one implementation.

18. **Clearing/static-setting IPs left stale live node information**
    - Fixed by synchronizing the live node IP/state fields.

19. **Work-order API always reported no active mission**
    - The endpoint read snake-case fields, while the state snapshot exposes
      camel-case fields.
   - Fixed.

20. **Preflight checks did not match the selected operating mode**
    - Simulated AMR preflight still published a real AMR STATUS command.
    - Conveyor preflight did not require a running integrated OpenCV pipeline.
    - Arm preflight could pass without both required arm endpoints.
    - Fixed.

21. **Mosquitto Windows service was reachable only from localhost**
    - Mosquitto 2 used its default localhost-only mode because
      `C:\Program Files\mosquitto\mosquitto.conf` had no active listener.
    - The dashboard connected to `127.0.0.1:1883`, but ESP32 devices timed out
      on `192.168.137.1:1883`.
    - Added `SETUP_MQTT_BROKER_ADMIN.ps1`, improved broker diagnostics, and
      configured the live broker/firewall for `192.168.137.0/24`.

## Verification completed

```text
python -m py_compile cell_control_center.py conveyor_cv_bridge.py
python -m unittest -v test_control_center.py
```

Result: 15 tests passed.

The focused tests cover:

- Monitor-only command blocking.
- Simulated AMR isolation.
- Box-filled deduplication.
- Arm delay scheduling.
- ACK/job matching.
- Failed MQTT publish handling.
- STOP latching.
- Late-event and MQTT-reconnect STOP preservation.
- Discovery subnet validation.
- External cube ingestion.
- Active work-order reporting.
- Stale ready/DONE event rejection.
- Simulated AMR preflight isolation.

## Remaining known risks and limitations

1. **Only one active mission is supported**
   - A second box becoming full can replace the current active mission.
   - There is no mission queue.

2. **Simulated AMR can still start a real arm**
   - This is intentional in the current design: only the AMR is simulated.
   - Turn Auto coordinator OFF or disconnect the arm for a completely
     non-physical simulation.

3. **The dashboard is not a safety system**
   - STOP depends on Wi-Fi, HTTP, MQTT, and subsystem firmware.
   - Use a hardware emergency-stop circuit.

4. **No authentication or encryption**
   - Flask controls and Mosquitto anonymous access are exposed on the local
     network.
   - Use only on an isolated trusted network.

5. **State is not persisted**
   - Box counts, pending commands, completed jobs, and mission state reset when
     the Python process restarts.

6. **Integrated CV settings are hardcoded**
   - The main program uses stream port 81, servo port 82, minimum area 500,
     stable count 8, and missing-frame unlock count 6.
   - `conveyor_bridge_config.json` configures only the legacy standalone
     bridge, not the integrated pipeline.

7. **BLACK is not detected**
   - The dashboard says BLACK/BLUE, but OpenCV currently detects RED, GREEN,
     and BLUE only.

8. **Legacy standalone bridge uses a different workflow**
   - It publishes `cell/conveyor/package` directly instead of filling box
     counters.
   - Do not run it with the integrated camera pipeline.

9. **No physical subsystem was available during this audit**
   - HTTP endpoint names, JSON fields, MQTT events, motor stop behavior, arm
     recipes, and AMR routes must be verified against the real firmware.

10. **Dependencies and broker are external**
    - Python packages come from `requirements.txt`.
    - Mosquitto must be installed separately.

11. **Static typing is incomplete**
    - `mypy` still reports type-annotation issues, mainly around heterogeneous
      configuration dictionaries and OpenCV frame storage.
    - No remaining undefined-name error was reported after the fixes.

## Recommended next hardware test

Test one subsystem at a time with machinery in a safe state:

1. Verify broker and status-only messages.
2. Verify camera and servo without AMR/arm automation.
3. Verify AMR STATUS and STOP.
4. Verify one AMR route with no payload.
5. Verify arm startup, home, load, wait, start, and abort.
6. Run one RED box mission.
7. Confirm physical STOP behavior at every stage.
