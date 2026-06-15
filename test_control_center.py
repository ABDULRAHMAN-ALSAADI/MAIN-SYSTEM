import re
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import cell_control_center as control


def filled_payload(job_id: str = "JOB_TEST_RED"):
    return {
        "event": "BoxRed_filled",
        "jobId": job_id,
        "color": "RED",
        "box": "BoxRed",
        "recipe": "RED_BOX",
    }


class VisionClassifierTests(unittest.TestCase):
    @staticmethod
    def target_frame(color, size=180):
        frame = control.np.zeros((480, 640, 3), dtype=control.np.uint8)
        control.cv2.rectangle(frame, (220, 150), (220 + size, 150 + size), color, -1)
        return frame

    def test_strict_classifier_accepts_red_green_and_blue_targets(self):
        expected_by_bgr = {
            (0, 0, 255): "RED",
            (0, 255, 0): "GREEN",
            (255, 0, 0): "BLUE",
        }

        for bgr, expected in expected_by_bgr.items():
            with self.subTest(expected=expected):
                _, result = control.cv_detect_color(self.target_frame(bgr))
                self.assertEqual(result["name"], expected)
                self.assertGreater(result["coverage"], 0.90)

    def test_strict_classifier_rejects_yellow_and_gray_targets(self):
        for bgr in ((0, 255, 255), (140, 140, 140)):
            with self.subTest(bgr=bgr):
                _, result = control.cv_detect_color(self.target_frame(bgr))
                self.assertEqual(result["name"], "NO COLOR")
                self.assertEqual(result["code"], "")

    def test_strict_classifier_accepts_dark_green_target(self):
        _, result = control.cv_detect_color(self.target_frame((12, 55, 18)))
        self.assertEqual(result["name"], "GREEN")

    def test_strict_classifier_rejects_dark_gray_target(self):
        _, result = control.cv_detect_color(self.target_frame((35, 38, 35)))
        self.assertEqual(result["name"], "NO COLOR")

    def test_strict_classifier_rejects_black_with_green_camera_tint(self):
        _, result = control.cv_detect_color(self.target_frame((18, 42, 28)))
        self.assertEqual(result["name"], "NO COLOR")

    def test_strict_classifier_rejects_green_tinted_border_shadow(self):
        frame = control.np.full((480, 640, 3), 100, dtype=control.np.uint8)
        control.cv2.rectangle(frame, (0, 100), (140, 479), (15, 60, 25), -1)
        _, result = control.cv_detect_color(frame)
        self.assertEqual(result["name"], "NO COLOR")

    def test_calibration_teaches_previously_rejected_muted_green(self):
        frame = self.target_frame((32, 50, 38))
        _, before = control.cv_detect_color(frame)
        self.assertEqual(before["name"], "NO COLOR")

        profile = control.cv_create_color_profile(frame, "GREEN")
        with patch.object(control, "cv_saved_color_profiles", return_value={"GREEN": profile}):
            _, after = control.cv_detect_color(frame)

        self.assertEqual(after["name"], "GREEN")
        self.assertTrue(after["learnedProfile"])

    def test_calibration_refuses_black_green_tinted_sample(self):
        frame = self.target_frame((18, 42, 28))
        with self.assertRaisesRegex(ValueError, "too dark|not colorful enough"):
            control.cv_create_color_profile(frame, "GREEN")

    def test_strict_classifier_rejects_small_colored_regions(self):
        _, result = control.cv_detect_color(self.target_frame((0, 0, 255), size=20))
        self.assertEqual(result["name"], "NO COLOR")


class SorterCycleTests(unittest.TestCase):
    def test_sorter_sends_one_color_command_and_never_auto_centers(self):
        state = control.cv_new_sorter_cycle_state()

        first = control.cv_sorter_cycle_step(state, "R", "RED")
        confirmed = control.cv_sorter_cycle_step(state, "R", "RED")
        flicker = control.cv_sorter_cycle_step(state, "G", "GREEN")
        empty = control.cv_sorter_cycle_step(state, "", "NO COLOR")

        self.assertEqual(first["sortCode"], "")
        self.assertEqual(confirmed["sortCode"], "R")
        self.assertEqual(flicker["sortCode"], "")
        self.assertEqual(flicker["displayCode"], "R")
        self.assertEqual(empty["sortCode"], "")
        self.assertNotIn("sendCenter", flicker)
        self.assertEqual(state["positionCode"], "R")

    def test_sorter_stays_at_red_until_next_confirmed_color(self):
        state = control.cv_new_sorter_cycle_state()
        control.cv_sorter_cycle_step(state, "R", "RED")
        control.cv_sorter_cycle_step(state, "R", "RED")

        rearmed = False
        for index in range(control.CV_REARM_FRAMES):
            action = control.cv_sorter_cycle_step(state, "", "NO COLOR")
            rearmed = rearmed or action["rearmed"]

        waiting = control.cv_sorter_cycle_step(state, "", "NO COLOR")
        green_first = control.cv_sorter_cycle_step(state, "G", "GREEN")
        green_confirmed = control.cv_sorter_cycle_step(state, "G", "GREEN")

        self.assertTrue(rearmed)
        self.assertEqual(waiting["sorterState"], "READY AT RED")
        self.assertEqual(green_first["sortCode"], "")
        self.assertEqual(green_confirmed["sortCode"], "G")
        self.assertEqual(state["positionCode"], "G")

    def test_one_missing_frame_does_not_restart_color_confirmation(self):
        state = control.cv_new_sorter_cycle_state()

        first = control.cv_sorter_cycle_step(state, "R", "RED")
        missed = control.cv_sorter_cycle_step(state, "", "NO COLOR")
        confirmed = control.cv_sorter_cycle_step(state, "R", "RED")

        self.assertEqual(first["sorterState"], "VERIFYING")
        self.assertEqual(missed["sorterState"], "VERIFYING")
        self.assertEqual(confirmed["sortCode"], "R")

    def test_same_color_next_cube_still_creates_one_new_sort_command(self):
        state = control.cv_new_sorter_cycle_state()
        control.cv_sorter_cycle_step(state, "R", "RED")
        first_cube = control.cv_sorter_cycle_step(state, "R", "RED")
        for _ in range(control.CV_REARM_FRAMES):
            control.cv_sorter_cycle_step(state, "", "NO COLOR")
        control.cv_sorter_cycle_step(state, "R", "RED")
        second_cube = control.cv_sorter_cycle_step(state, "R", "RED")

        self.assertEqual(first_cube["sortCode"], "R")
        self.assertEqual(second_cube["sortCode"], "R")


class DashboardIntegrityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = Path(__file__).with_name("index.html").read_text(encoding="utf-8")

    def test_dashboard_has_no_duplicate_element_ids(self):
        ids = re.findall(r'\bid="([^"]+)"', self.html)
        duplicates = sorted({element_id for element_id in ids if ids.count(element_id) > 1})
        self.assertEqual(duplicates, [])

    def test_dashboard_literal_api_routes_exist(self):
        referenced = set(re.findall(r"""['"](/(?:api|cv)/[^'"]+)['"]""", self.html))
        routes = {rule.rule for rule in control.app.url_map.iter_rules()}
        missing = sorted(path for path in referenced if not path.endswith("/") and path not in routes)
        self.assertEqual(missing, [])

    def test_dashboard_literal_element_references_exist(self):
        ids = set(re.findall(r'\bid="([^"]+)"', self.html))
        referenced = set(re.findall(r"""\$\(['"]([^'"]+)['"]\)""", self.html))
        self.assertEqual(sorted(referenced - ids), [])


class ControlCenterTests(unittest.TestCase):
    def setUp(self):
        self.original_state = control.S
        control.S = control.AppState()
        control.S.recipes = [
            {"index": 0, "name": "RED_BOX", "steps": []},
            {"index": 1, "name": "GREEN_BOX_A1", "steps": []},
            {"index": 2, "name": "BLUE_BOX", "steps": []},
        ]

    def tearDown(self):
        control.S = self.original_state

    def test_monitor_only_does_not_schedule_amr(self):
        control.S.config["auto_coordinate"] = False
        with patch.object(control, "select_recipe_by_name") as select_recipe, \
                patch.object(control, "schedule_delayed") as schedule, \
                patch.object(control.threading, "Thread") as thread:
            control.handle_conveyor_box_filled(filled_payload())

        select_recipe.assert_not_called()
        schedule.assert_not_called()
        thread.assert_not_called()
        self.assertEqual(control.S.mission_state, "BOX_FILLED_MONITOR_ONLY")

    def test_default_color_recipe_mapping_matches_arm_recipes(self):
        recipes = {rule["color"]: rule["recipe"] for rule in control.default_workflow_rules()}
        self.assertEqual(recipes, {
            "RED": "RED_BOX",
            "GREEN": "GREEN_BOX_A1",
            "BLUE": "BLUE_BOX",
        })

    def test_legacy_workflow_recipe_names_migrate_to_current_arm_recipes(self):
        rules, changed = control.migrate_manual_arm_workflow_rules([
            {"color": "RED", "box": "BoxRed", "recipe": "RED_SQUARE_TO_SHELF_A", "deliveredEvent": "BoxRed_delivered_to_target"},
            {"color": "GREEN", "box": "BoxGreen", "recipe": "GREEN_SQUARE_TO_SHELF_C", "deliveredEvent": "BoxGreen_delivered_to_target"},
            {"color": "BLUE", "box": "BoxBlue", "recipe": "BLUE_SQUARE_TO_SHELF_B", "deliveredEvent": "BoxBlue_delivered_to_target"},
        ])

        self.assertTrue(changed)
        self.assertEqual([rule["recipe"] for rule in rules], ["RED_BOX", "GREEN_BOX_A1", "BLUE_BOX"])
        self.assertEqual([rule["deliveredEvent"] for rule in rules], [
            "BoxRed_finished_manual_delivery",
            "BoxGreen_finished_manual_delivery",
            "BoxBlue_finished_manual_delivery",
        ])

    def test_stale_conveyor_recipe_payload_cannot_override_color_mapping(self):
        control.S.config["auto_coordinate"] = True
        payload = filled_payload("JOB_STALE_RECIPE")
        payload["recipe"] = "RED_SQUARE_TO_SHELF_A"
        with patch.object(control, "select_recipe_by_name", return_value=True) as select_recipe, \
                patch.object(control, "schedule_delayed") as schedule:
            control.handle_conveyor_box_filled(payload)

        select_recipe.assert_called_once_with("RED_BOX")
        schedule.assert_called_once_with(
            0,
            control.execute_arm_recipe_start,
            "RED",
            "RED_BOX",
            "JOB_STALE_RECIPE",
            control.S.operation_generation,
        )

    def test_retry_active_arm_job_refreshes_recipes_and_schedules_current_box(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_SQUARE_TO_SHELF_A"
        control.S.active_box_job = "JOB_RETRY"
        control.S.current_job = "JOB_RETRY"
        control.S.mission_state = "ARM_RECIPE_MISSING"
        with patch.object(control, "refresh_arm_recipes", return_value=True) as refresh, \
                patch.object(control, "select_recipe_by_name", return_value=True) as select_recipe, \
                patch.object(control, "schedule_delayed") as schedule:
            response = control.app.test_client().post("/api/arm/retry-active-box")

        body = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(body["recipe"], "RED_BOX")
        self.assertEqual(control.S.active_box_recipe, "RED_BOX")
        refresh.assert_called_once_with()
        select_recipe.assert_called_once_with("RED_BOX")
        schedule.assert_called_once_with(
            0,
            control.execute_arm_recipe_start,
            "RED",
            "RED_BOX",
            "JOB_RETRY",
            control.S.operation_generation,
        )

    def test_retry_active_arm_job_refuses_when_arm_is_unreachable(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_job = "JOB_RETRY_OFFLINE"
        with patch.object(control, "refresh_arm_recipes", return_value=False), \
                patch.object(control, "select_recipe_by_name") as select_recipe, \
                patch.object(control, "schedule_delayed") as schedule:
            response = control.app.test_client().post("/api/arm/retry-active-box")

        self.assertEqual(response.status_code, 503)
        self.assertIn("did not answer /config", response.get_json()["err"])
        select_recipe.assert_not_called()
        schedule.assert_not_called()

    def test_second_arm_job_clears_previous_done_result_before_loading(self):
        control.S.active_box_color = "GREEN"
        control.S.active_box_name = "BoxGreen"
        control.S.active_box_recipe = "GREEN_BOX_A1"
        control.S.active_box_job = "JOB_GREEN"
        control.S.current_job = "JOB_GREEN"
        control.S.operation_generation = 7

        responses = {
            "/job/status": (200, {"ok": True, "state": "DONE", "jobId": "JOB_RED"}),
            "/job/clear": (200, {"ok": True, "msg": "Arm ready."}),
            "/job/load?recipe=1&jobId=JOB_GREEN&packageClass=GREEN_BOX_A1": (200, {"ok": True, "msg": "Job loaded."}),
            "/job/wait-amr": (200, {"ok": True, "msg": "Waiting for AMR."}),
            "/job/amr-arrived": (200, {"ok": True, "msg": "Job running."}),
        }

        with patch.object(control, "arm_get", side_effect=lambda path, timeout=5: responses[path]) as arm_get:
            control.execute_arm_recipe_start("GREEN", "GREEN_BOX_A1", "JOB_GREEN", 7)

        self.assertEqual([call.args[0] for call in arm_get.call_args_list], [
            "/job/status",
            "/job/clear",
            "/job/load?recipe=1&jobId=JOB_GREEN&packageClass=GREEN_BOX_A1",
            "/job/wait-amr",
            "/job/amr-arrived",
        ])
        self.assertEqual(control.S.mission_state, "ARM_LOADING_AMR")
        self.assertEqual(control.S.box_states["GREEN"], "ARM_LOADING_AMR")

    def test_new_arm_job_does_not_clear_ready_arm(self):
        with patch.object(control, "arm_get", return_value=(200, {"ok": True, "state": "READY"})) as arm_get:
            result = control.arm_prepare_for_new_job()

        self.assertTrue(result["ok"])
        arm_get.assert_called_once_with("/job/status", timeout=5)

    def test_done_clear_failure_reports_exact_arm_response_and_does_not_load(self):
        control.S.active_box_color = "GREEN"
        control.S.active_box_name = "BoxGreen"
        control.S.active_box_recipe = "GREEN_BOX_A1"
        control.S.active_box_job = "JOB_GREEN"
        control.S.current_job = "JOB_GREEN"
        control.S.operation_generation = 8

        responses = {
            "/job/status": (200, {"ok": True, "state": "DONE", "jobId": "JOB_RED"}),
            "/job/clear": (409, {"ok": False, "err": "HOME the arm before returning to READY."}),
        }
        with patch.object(control, "arm_get", side_effect=lambda path, timeout=5: responses[path]) as arm_get:
            control.execute_arm_recipe_start("GREEN", "GREEN_BOX_A1", "JOB_GREEN", 8)

        self.assertEqual([call.args[0] for call in arm_get.call_args_list], ["/job/status", "/job/clear"])
        self.assertEqual(control.S.system, "ARM PREPARE FAILED")
        self.assertIn("HOME the arm before returning to READY.", control.S.next_action)

    def test_state_exposes_active_box_job(self):
        control.S.active_box_job = "JOB_VISIBLE"

        response = control.app.test_client().get("/api/state")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["activeBoxJob"], "JOB_VISIBLE")

    def test_config_endpoint_migrates_legacy_recipe_names(self):
        with patch.object(control.S, "save_config"):
            response = control.app.test_client().post("/api/config", json={
                "workflow_rules": [{
                    "color": "RED",
                    "box": "BoxRed",
                    "recipe": "RED_SQUARE_TO_SHELF_A",
                    "deliveredEvent": "BoxRed_delivered_to_target",
                }],
            })

        self.assertEqual(response.status_code, 200)
        saved_rule = control.S.config["workflow_rules"][0]
        self.assertEqual(saved_rule["recipe"], "RED_BOX")
        self.assertEqual(saved_rule["deliveredEvent"], "BoxRed_finished_manual_delivery")

    def test_simulated_amr_still_starts_arm_first(self):
        control.S.config["auto_coordinate"] = True
        control.S.config["simulate_amr"] = True
        with patch.object(control, "select_recipe_by_name", return_value=True), \
                 patch.object(control, "schedule_delayed") as schedule, \
                 patch.object(control.threading, "Thread") as thread:
            control.handle_conveyor_box_filled(filled_payload())

        schedule.assert_called_once_with(
            0,
            control.execute_arm_recipe_start,
            "RED",
            "RED_BOX",
            "JOB_TEST_RED",
            1,
        )
        thread.assert_not_called()

    def test_real_amr_box_filled_event_is_deduplicated(self):
        control.S.config["auto_coordinate"] = True
        control.S.config["simulate_amr"] = False
        with patch.object(control, "select_recipe_by_name", return_value=True), \
                patch.object(control, "schedule_delayed") as schedule:
            payload = filled_payload()
            control.handle_conveyor_box_filled(payload)
            control.handle_conveyor_box_filled(payload)

        schedule.assert_called_once()

    def test_box_filled_starts_arm_using_configured_delay(self):
        control.S.config["auto_coordinate"] = True
        control.S.config["workflow_rules"][0]["delayBeforeArmMs"] = 750
        with patch.object(control, "select_recipe_by_name", return_value=True), \
                patch.object(control, "schedule_delayed") as schedule:
            control.handle_conveyor_box_filled(filled_payload("JOB_ARM_FIRST"))

        schedule.assert_called_once_with(
            750,
            control.execute_arm_recipe_start,
            "RED",
            "RED_BOX",
            "JOB_ARM_FIRST",
            1,
        )

    def test_ack_resolves_only_matching_job(self):
        first = control.S.add_pending_ack(
            "amr", "DELIVER_BOX_TO_TARGET", "AMR_DELIVERY_COMMAND_ACCEPTED", 8, {"jobId": "JOB_1"}, "cell/amr/cmd"
        )
        second = control.S.add_pending_ack(
            "amr", "DELIVER_BOX_TO_TARGET", "AMR_DELIVERY_COMMAND_ACCEPTED", 8, {"jobId": "JOB_2"}, "cell/amr/cmd"
        )

        resolved = control.S.resolve_ack("AMR_DELIVERY_COMMAND_ACCEPTED", {"jobId": "JOB_2"})

        self.assertTrue(resolved)
        self.assertIn(first, control.S.pending_acks)
        self.assertNotIn(second, control.S.pending_acks)

    def test_failed_amr_publish_removes_pending_ack(self):
        control.S.active_box_job = "JOB_FAIL"
        control.S.active_box_color = "RED"
        control.S.operation_generation = 3
        with patch.object(control, "publish", return_value=False):
            control.publish_amr_delivery_command("RED", filled_payload("JOB_FAIL"), 3)

        self.assertEqual(control.S.pending_acks, {})
        self.assertEqual(control.S.mission_state, "FAULT")

    def test_stop_latch_blocks_new_box_flow(self):
        control.S.mission_state = "STOPPED"
        with patch.object(control, "select_recipe_by_name") as select_recipe, \
                patch.object(control, "schedule_delayed") as schedule:
            control.handle_conveyor_box_filled(filled_payload())

        select_recipe.assert_not_called()
        schedule.assert_not_called()
        self.assertEqual(control.S.mission_state, "STOPPED")

    def test_stop_latch_ignores_late_amr_event(self):
        control.S.mission_state = "STOPPED"
        control.handle_amr_event({
            "event": "Done_Picking_RedBox",
            "jobId": "JOB_LATE",
            "color": "RED",
            "box": "BoxRed",
        })
        self.assertEqual(control.S.mission_state, "STOPPED")

    def test_mqtt_reconnect_does_not_clear_stop_latch(self):
        control.S.mission_state = "STOPPED"
        client = MagicMock()
        with patch.object(control, "publish", return_value=True):
            control.on_mqtt_connect(client, None, None, 0)
        self.assertEqual(control.S.mission_state, "STOPPED")

    def test_legacy_amr_ready_event_does_not_start_arm(self):
        control.S.active_box_job = "JOB_ACTIVE"
        with patch.object(control, "schedule_delayed") as schedule:
            control.handle_amr_event({
                "event": "RedBox_ready",
                "jobId": "JOB_ACTIVE",
                "color": "RED",
                "recipe": "RED_BOX",
            })
        schedule.assert_not_called()
        self.assertEqual(control.S.active_box_job, "JOB_ACTIVE")

    def test_stopped_arm_done_is_consumed_without_completing_job(self):
        control.S.mission_state = "STOPPED"
        with patch.object(control, "arm_get", return_value=(200, {"state": "DONE", "jobId": "JOB_STOPPED"})), \
                patch.object(control, "publish", return_value=True), \
                patch.object(control, "handle_arm_loaded_on_amr") as loaded:
            result = control.poll_arm_status()
        self.assertTrue(result)
        self.assertEqual(control.S.mission_state, "STOPPED")
        self.assertEqual(control.S.last_arm_event_key, "DONE:JOB_STOPPED")
        loaded.assert_not_called()

    def test_arm_done_waits_for_operator_without_commanding_amr(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.active_box_job = "JOB_LOAD"
        control.S.current_job = "JOB_LOAD"
        control.S.operation_generation = 4
        with patch.object(control, "publish", return_value=True) as publish, \
                patch.object(control, "schedule_delayed") as schedule:
            control.handle_arm_loaded_on_amr({"state": "DONE", "jobId": "JOB_LOAD"})

        schedule.assert_not_called()
        self.assertFalse(any(call.args and call.args[0] == control.TOPIC["amr_cmd"] for call in publish.call_args_list))
        self.assertEqual(control.S.mission_state, "WAITING_OPERATOR_FINISH")
        self.assertEqual(control.S.box_states["RED"], "ON_AMR_WAITING_OPERATOR")
        self.assertEqual(len(control.S.done_jobs), 0)
        self.assertEqual(control.S.active_box_job, "JOB_LOAD")

    def test_amr_delivery_event_does_not_replace_operator_confirmation(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.active_box_job = "JOB_DELIVER"
        control.S.current_job = "JOB_DELIVER"
        control.S.mission_state = "WAITING_OPERATOR_FINISH"
        control.S.box_counts["RED"] = 4
        with patch.object(control, "publish", return_value=True):
            control.handle_amr_event({
                "event": "DELIVERY_COMPLETE",
                "jobId": "JOB_DELIVER",
                "color": "RED",
                "box": "BoxRed",
                "destination": "TARGET_RED",
            })

        self.assertEqual(control.S.mission_state, "WAITING_OPERATOR_FINISH")
        self.assertEqual(control.S.box_counts["RED"], 4)
        self.assertEqual(control.S.active_box_job, "JOB_DELIVER")
        self.assertEqual(len(control.S.done_jobs), 0)

    def test_finished_job_button_completes_active_job(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.active_box_job = "JOB_DELIVER"
        control.S.current_job = "JOB_DELIVER"
        control.S.mission_state = "WAITING_OPERATOR_FINISH"
        control.S.box_counts["RED"] = 4
        with patch.object(control, "publish", return_value=True):
            response = control.app.test_client().post("/api/amr/finished-job")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(control.S.mission_state, "JOB_DONE")
        self.assertEqual(control.S.box_counts["RED"], 0)
        self.assertEqual(control.S.active_box_job, "")
        self.assertEqual(len(control.S.done_jobs), 1)

    def test_finished_job_button_rejects_job_before_arm_done(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.active_box_job = "JOB_RUNNING"
        control.S.mission_state = "ARM_LOADING_AMR"

        response = control.app.test_client().post("/api/amr/finished-job")

        self.assertEqual(response.status_code, 409)
        self.assertEqual(control.S.active_box_job, "JOB_RUNNING")

    def test_second_filled_box_queues_until_finished_job(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.active_box_job = "JOB_RED"
        control.S.current_job = "JOB_RED"
        control.S.mission_state = "WAITING_OPERATOR_FINISH"
        control.S.box_counts["RED"] = 4
        green = {
            "event": "BoxGreen_filled",
            "jobId": "JOB_GREEN",
            "color": "GREEN",
            "box": "BoxGreen",
            "recipe": "GREEN_BOX_A1",
        }
        with patch.object(control, "schedule_delayed") as schedule:
            control.handle_conveyor_box_filled(green)

        schedule.assert_not_called()
        self.assertEqual(len(control.S.box_queue), 1)
        self.assertEqual(control.S.box_states["GREEN"], "QUEUED_WAITING_ARM")

        with patch.object(control, "publish", return_value=True), \
                patch.object(control, "select_recipe_by_name", return_value=True), \
                patch.object(control, "schedule_delayed") as schedule:
            response = control.app.test_client().post("/api/amr/finished-job")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(control.S.active_box_job, "JOB_GREEN")
        self.assertEqual(control.S.active_box_recipe, "GREEN_BOX_A1")
        self.assertEqual(len(control.S.box_queue), 0)
        schedule.assert_called_once_with(
            0,
            control.execute_arm_recipe_start,
            "GREEN",
            "GREEN_BOX_A1",
            "JOB_GREEN",
            control.S.operation_generation,
        )

    def test_amr_delivery_command_uses_target_route(self):
        control.S.active_box_color = "RED"
        control.S.active_box_job = "JOB_ROUTE"
        control.S.operation_generation = 2
        with patch.object(control, "publish", return_value=True) as publish:
            control.publish_amr_delivery_command("RED", {"jobId": "JOB_ROUTE"}, 2)

        payload = publish.call_args.args[1]
        self.assertEqual(payload["cmd"], "DELIVER_BOX_TO_TARGET")
        self.assertEqual(payload["pickup"], "ARM_STATION")
        self.assertEqual(payload["destination"], "TARGET_RED")
        self.assertEqual(control.S.mission_state, "WAITING_AMR_ACK")

    def test_invalid_discovery_subnet_returns_400(self):
        response = control.app.test_client().post("/api/discovery/scan", json={"subnet": "999.1.2"})
        self.assertEqual(response.status_code, 400)

    def test_simulated_amr_check_does_not_command_real_amr(self):
        control.S.config["simulate_amr"] = True
        with patch.object(control, "publish") as publish:
            response = control.app.test_client().post("/api/check/amr")
        body = response.get_json()
        self.assertTrue(body["ok"])
        self.assertEqual(body["mode"], "SIMULATED")
        publish.assert_not_called()

    def test_external_cube_event_is_ingested_once(self):
        message = MagicMock()
        message.topic = control.TOPIC["conveyor_cube"]
        message.payload = b'{"event":"CUBE_COUNTED","color":"RED","source":"external_conveyor"}'
        with patch.object(control, "record_conveyor_cube") as record:
            control.on_mqtt_message(None, None, message)
        record.assert_called_once_with("RED")

    def test_simulate_cube_web_endpoint_counts_selected_color(self):
        with patch.object(control, "publish", return_value=True):
            response = control.app.test_client().post("/api/sim/conveyor", json={"color": "GREEN"})

        body = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(body["mode"], "CUBE_DETECTION")
        self.assertEqual(body["status"]["color"], "GREEN")
        self.assertEqual(control.S.box_counts["GREEN"], 1)

    def test_servo_command_verifies_esp32_response_and_angle(self):
        response = MagicMock()
        response.ok = True
        response.status_code = 200
        response.json.return_value = {"ok": True, "colorCode": "R", "servoAngle": 30}

        with patch.object(control.requests, "get", return_value=response) as get:
            result = control.cv_send_servo("R")

        self.assertTrue(result["ok"])
        self.assertEqual(result["data"]["servoAngle"], 30)
        self.assertEqual(control.CV["lastServo"], "R -> 30 deg")
        get.assert_called_once_with("http://192.168.137.172:82/servo-test?angle=30", timeout=1.0)

    def test_servo_command_uses_saved_color_angle(self):
        control.S.config["sorter_servo_angles"]["R"] = 42
        response = MagicMock()
        response.ok = True
        response.status_code = 200
        response.json.return_value = {"ok": True, "servoAngle": 42}

        with patch.object(control.requests, "get", return_value=response) as get:
            result = control.cv_send_servo("RED")

        self.assertTrue(result["ok"])
        self.assertEqual(result["requestedAngle"], 42)
        get.assert_called_once_with("http://192.168.137.172:82/servo-test?angle=42", timeout=1.0)

    def test_servo_command_reports_http_failure_instead_of_false_sent_status(self):
        response = MagicMock()
        response.ok = False
        response.status_code = 404
        response.json.return_value = {"ok": False, "err": "Not found"}

        with patch.object(control.requests, "get", return_value=response):
            result = control.cv_send_servo("R")

        self.assertFalse(result["ok"])
        self.assertEqual(control.CV["lastServo"], "R HTTP 404")

    def test_manual_servo_test_endpoint_accepts_center_and_rejects_bad_code(self):
        with patch.object(control, "cv_send_servo", return_value={"ok": True, "code": "CENTER"}) as send:
            center = control.app.test_client().post("/api/conveyor/servo-test", json={"code": "CENTER"})
            bad = control.app.test_client().post("/api/conveyor/servo-test", json={"code": "YELLOW"})

        self.assertEqual(center.status_code, 200)
        self.assertEqual(bad.status_code, 400)
        send.assert_called_once_with("CENTER", allow_center=True)

    def test_automatic_center_command_is_hard_blocked(self):
        with patch.object(control.requests, "get") as get:
            result = control.cv_send_servo("CENTER")

        self.assertFalse(result["ok"])
        self.assertIn("disabled", result["err"])
        get.assert_not_called()

    def test_servo_save_endpoint_persists_named_position(self):
        with patch.object(control.S, "save_config") as save:
            response = control.app.test_client().post(
                "/api/conveyor/servo-save",
                json={"target": "GREEN", "angle": 117},
            )

        body = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertEqual(body["target"], "G")
        self.assertEqual(body["angles"]["G"], 117)
        self.assertEqual(control.S.config["sorter_servo_angles"]["G"], 117)
        save.assert_called_once()

    def test_manual_servo_angle_endpoint_moves_and_clamps_angle(self):
        esp_response = MagicMock()
        esp_response.ok = True
        esp_response.status_code = 200
        esp_response.json.return_value = {"ok": True, "servoAngle": 180}
        with patch.object(control.requests, "get", return_value=esp_response) as get:
            response = control.app.test_client().post("/api/conveyor/servo-angle", json={"angle": 220})

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["requestedAngle"], 180)
        get.assert_called_once_with("http://192.168.137.172:82/servo-test?angle=180", timeout=1.0)

    def test_servo_calibration_endpoint_returns_saved_positions(self):
        control.S.config["sorter_servo_angles"]["B"] = 133
        response = control.app.test_client().get("/api/conveyor/servo-calibration")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["angles"]["B"], 133)

    def test_calibration_endpoint_saves_learned_green_profile(self):
        frame = control.np.zeros((480, 640, 3), dtype=control.np.uint8)
        control.cv2.rectangle(frame, (220, 140), (420, 340), (32, 50, 38), -1)
        with control.CV_LOCK:
            previous_frame = control.CV.get("latestFrame")
            control.CV["latestFrame"] = frame
        try:
            with patch.object(control.S, "save_config"):
                response = control.app.test_client().post("/api/cv/calibrate", json={"color": "GREEN"})
        finally:
            with control.CV_LOCK:
                control.CV["latestFrame"] = previous_frame

        body = response.get_json()
        self.assertEqual(response.status_code, 200)
        self.assertTrue(body["ok"])
        self.assertIn("GREEN", control.S.config["cv_color_profiles"])

    def test_config_endpoint_returns_and_applies_new_robot_ips(self):
        with patch.object(control.S, "save_config"):
            response = control.app.test_client().post("/api/config", json={
                "arm_ip": "192.168.137.10",
                "amr_ip": "192.168.137.11",
                "conveyor_ip": "192.168.137.12",
            })

        body = response.get_json()
        self.assertEqual(body["config"]["arm_ip"], "192.168.137.10")
        self.assertEqual(control.S.nodes["amr"]["ip"], "192.168.137.11")
        self.assertEqual(control.S.nodes["conveyor"]["ip"], "192.168.137.12")

    def test_work_order_uses_active_mission_fields(self):
        control.S.active_box_color = "RED"
        control.S.active_box_name = "BoxRed"
        control.S.active_box_recipe = "RED_BOX"
        control.S.mission_state = "AMR_DELIVERING"

        response = control.app.test_client().get("/api/work-order")
        body = response.get_json()

        self.assertEqual(body["box"], "BoxRed")
        self.assertEqual(body["armRecipe"], "RED_BOX")


if __name__ == "__main__":
    unittest.main()
