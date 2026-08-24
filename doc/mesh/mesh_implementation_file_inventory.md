# Broadcast Mesh Implementation File Inventory

This note records the files retained by the production broadcast-mesh refactor. Rejected UART batching and other temporary experiments are excluded.

| Area | Files | Retained purpose |
| --- | --- | --- |
| Build and wire schema | `Makefile.ac`, `conf/messages_mesh_new.xml` (`conf/messages.xml` selects it) | Add `ALIVE_REQ` ID 195 and regenerate the active protocol header before aircraft MD5 generation. |
| Airborne datalink | `sw/airborne/modules/datalink/datalink.c`, `datalink.h`, `downlink.c` | Split identity from health, retain targeted PING evidence for automatic modes, and preserve uplink-age semantics. |
| Mesh and TCAS | `sw/airborne/modules/multi/traffic_info.c`, `traffic_info.h`, `traffic_info_mesh_clock.h`, `traffic_info_mesh_mode.h`, `tcas.c`, `tcas_policy.h` | Provide self-organising slots, GPS holdover and fallback, automatic telemetry modes, checked traffic snapshots, and fail-closed TCAS policy. |
| Ground link | `sw/ground_segment/tmtc/link.ml`, `parse_messages_v1.ml` | Provide serialized identity recovery, durable snapshot reconciliation, monotonic completed RTT, restart-safe serial handling, and unchanged uplink-age forwarding. |
| Serial library | `sw/lib/ocaml/serial.ml`, `serial.mli`, `cserial.c` | Add 460800 baud, bounds checks, monotonic time, and exclusive radio ownership. |
| Cockpit | `sw/ground_segment/cockpit/src/widgets/link_status.cpp` | Present the raw counter accurately as `Uplink age [s]`. |
| Configuration | `conf/modules/traffic_info.xml`, `conf/telemetry/OPENUAS/openuas_fixedwing_mesh.xml`, `openuas_rotorcraft_mesh.xml`, relevant OPENUAS airframes and fleet configurations | Centralize TDMA constants and deploy automatic `mesh`, `mesh_manifold`, and `mesh_solo` profiles. |
| Analysis and provisioning | `sw/tools/mesh/e52_provision.py`, `mesh_phase_optimizer.py`, `mesh_slot_sim.py`, `mesh_gps_denied_sim.py`, `mesh_link_budget.py`, `mesh_link_sim.py` | Provide reproducible legal reference-radio setup and executable RF, airtime, cache, churn, clock-loss, and mobility gates. |
| Focused tests | `tests/utils/test_mesh_clock.c`, `test_mesh_mode_policy.c`, `test_tcas_policy.c`, plus `sw/tools/mesh/test_mesh_phase_optimizer.py`, `test_mesh_slot_sim.py`, `test_mesh_gps_denied_sim.py` | Lock down clock, mode, TCAS, capacity, slot convergence, and fallback behavior. |
| Documentation | `doc/mesh/mesh_network_design.md`, `doc/mesh/asynchronous_coded_mesh.md`, `doc/sphinx/source/modules/traffic_info.xml` | Describe the flighted design, keep coded-mesh research separate, and expose module settings. |
