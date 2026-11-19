#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "ServoCalibration/servo_calibration.h"
#include "JointMapping/joint_mapping.h"
#include "Configuration/servo_configuration.h"
#include "Configuration/robot_model.h"
#include "Configuration/chosen_configurations.h"
#include "ForwardKinematics/forward_kinematics.h"

//Example table. Not assigned to any installed joint.
//These values are desktop test data, not assembled movement limits.
static const ServoCalibrationPoint example_points[] = {
    //a_j [deg], V_j [V], D_j [%]
    {  0, 0.04,  1.8},
    { 10, 0.20,  2.3},
    { 20, 0.32,  2.8},
    { 30, 0.48,  3.3},
    { 40, 0.60,  3.8},
    { 50, 0.77,  4.4},
    { 60, 0.92,  5.0},
    { 70, 1.11,  5.5},
    { 80, 1.27,  6.1},
    { 90, 1.41,  6.6},
    {100, 1.59,  7.2},
    {110, 1.75,  7.8},
    {120, 1.89,  8.3},
    {130, 2.09,  8.9},
    {140, 2.22,  9.4},
    {150, 2.37, 10.0},
    {160, 2.53, 10.5},
    {170, 2.67, 11.1},
    {180, 2.81, 11.6}
};

static const ServoCalibration example_calibration = {
    example_points,
    (int)(sizeof(example_points) / sizeof(example_points[0])),
    50.0
};

static int checks = 0;
static int failures = 0;

static void check(int condition, const char *description)
{
    checks++;
    if (!condition) {
        failures++;
        printf("FAIL: %s\n", description);
    }
}

//Numerical rounding tolerance only, not physical servo accuracy.
static int close_to(double actual, double expected)
{
    return isfinite(actual) && fabs(actual - expected) < 1e-9;
}

//PASS: exact table rows are recovered. FAIL: inspect the named row/lookup or pulse units.
static void test_table_entries(void)
{
    const double expected_pulse_us[] = {
        360, 460, 560, 660, 760, 880, 1000, 1100, 1220, 1320,
        1440, 1560, 1660, 1780, 1880, 2000, 2100, 2220, 2320
    };

    for (int j = 0; j < example_calibration.point_count; j++) {
        double a = NAN, V = NAN, pulse = NAN;
        int angle_ok = servo_angle_from_feedback(&example_calibration, example_points[j].feedback_V, &a);
        int voltage_ok = servo_feedback_from_angle(&example_calibration, example_points[j].servo_angle_deg, &V);
        int pulse_ok = servo_pulse_from_angle(&example_calibration, example_points[j].servo_angle_deg, &pulse);
        char description[80];

        snprintf(description, sizeof(description), "table row %d: V -> a", j);
        check(angle_ok && close_to(a, example_points[j].servo_angle_deg), description);
        snprintf(description, sizeof(description), "table row %d: a -> V", j);
        check(voltage_ok && close_to(V, example_points[j].feedback_V), description);
        snprintf(description, sizeof(description), "table row %d: a -> pulse", j);
        check(pulse_ok && close_to(pulse, expected_pulse_us[j]), description);
    }
}

//PASS: interpolation agrees with hand answers. FAIL: inspect interval selection and fraction.
static void test_between_entries(void)
{
    double a = NAN, V = NAN, pulse = NAN;
    int ok = servo_angle_from_feedback(&example_calibration, 0.685, &a);
    check(ok && close_to(a, 45.0), "0.685 V -> 45 deg");
    printf("Feedback: 0.685 V -> %.3f deg (expected 45.000)\n", a);

    ok = servo_feedback_from_angle(&example_calibration, 45.0, &V);
    check(ok && close_to(V, 0.685), "45 deg -> 0.685 V");
    printf("Reverse:  45.000 deg -> %.3f V (expected 0.685)\n", V);

    ok = servo_pulse_from_angle(&example_calibration, 45.0, &pulse);
    check(ok && close_to(pulse, 820.0), "45 deg -> 820 us");
    printf("Pulse:    45.000 deg -> %.3f us (expected 820.000 at 50 Hz)\n", pulse);

    //A quarter of the 70-to-80-degree interval, not just its midpoint.
    check(servo_angle_from_feedback(&example_calibration, 1.15, &a)
          && close_to(a, 72.5), "1.15 V -> 72.5 deg");
    check(servo_feedback_from_angle(&example_calibration, 72.5, &V)
          && close_to(V, 1.15), "72.5 deg -> 1.15 V");
    check(servo_pulse_from_angle(&example_calibration, 72.5, &pulse)
          && close_to(pulse, 1130), "72.5 deg -> 1130 us");
}

//All three functions must reject invalid tables without replacing the output.
static void check_invalid_table(const ServoCalibration *calibration, const char *description)
{
    double result = -123.0;
    int a_ok = servo_angle_from_feedback(calibration, 0.5, &result);
    int V_ok = servo_feedback_from_angle(calibration, 45, &result);
    int pulse_ok = servo_pulse_from_angle(calibration, 45, &result);
    check(!a_ok && !V_ok && !pulse_ok && result == -123.0, description);
}

//PASS means refusing invalid data without changing outputs. FAIL: inspect the named guard.
static void test_rejections(void)
{
    const double invalid_voltages[] = {0.039, 2.811, NAN, INFINITY};
    const double invalid_angles[] = {-0.01, 180.01, NAN, INFINITY};
    for (int j = 0; j < 4; j++) {
        double result = -123.0;
        check(!servo_angle_from_feedback(&example_calibration, invalid_voltages[j], &result)
              && result == -123.0, "reject invalid voltage; keep output");
        check(!servo_feedback_from_angle(&example_calibration, invalid_angles[j], &result)
              && result == -123.0, "reject invalid angle for voltage; keep output");
        check(!servo_pulse_from_angle(&example_calibration, invalid_angles[j], &result)
              && result == -123.0, "reject invalid angle for pulse; keep output");
    }

    check(!servo_angle_from_feedback(&example_calibration, 0.5, NULL), "reject null angle output");
    check(!servo_feedback_from_angle(&example_calibration, 45, NULL), "reject null voltage output");
    check(!servo_pulse_from_angle(&example_calibration, 45, NULL), "reject null pulse output");
    check_invalid_table(NULL, "reject null calibration");

    ServoCalibration bad = example_calibration;
    bad.points = NULL;
    check_invalid_table(&bad, "reject missing points");
    bad = example_calibration;
    bad.point_count = 1;
    check_invalid_table(&bad, "reject fewer than two points");
    bad = example_calibration;
    bad.pwm_frequency_hz = 0;
    check_invalid_table(&bad, "reject zero frequency");
    bad.pwm_frequency_hz = NAN;
    check_invalid_table(&bad, "reject nonfinite frequency");

    ServoCalibrationPoint points[] = {{0, 0.1, 5}, {90, 1.5, 7}, {180, 3.0, 10}};
    bad.points = points;
    bad.point_count = 3;
    bad.pwm_frequency_hz = 50;
    points[1].servo_angle_deg = 0;
    check_invalid_table(&bad, "reject duplicate angles");
    points[1].servo_angle_deg = -1;
    check_invalid_table(&bad, "reject descending angle rows");
    points[1].servo_angle_deg = 90;
    points[1].feedback_V = 0.1;
    check_invalid_table(&bad, "reject duplicate voltages");
    points[1].feedback_V = 3.1;
    check_invalid_table(&bad, "reject voltage direction reversal");
    points[1].feedback_V = NAN;
    check_invalid_table(&bad, "reject nonfinite table voltage");
    points[1].feedback_V = 1.5;
    points[1].duty_percent = 100;
    check_invalid_table(&bad, "reject invalid duty percentage");
}

//PASS: no assumption of rising voltage or equal row spacing. FAIL: inspect the named interval.
static void test_other_tables(void)
{
    //Synthetic data: unequal angle spacing and decreasing feedback.
    const ServoCalibrationPoint points[] = {{20, 2.5, 4}, {60, 1.5, 6}, {150, 0.3, 10}};
    ServoCalibration calibration = {points, 3, 50};
    double result = NAN;
    check(servo_angle_from_feedback(&calibration, 1.9, &result)
          && close_to(result, 44), "decreasing feedback -> angle");
    check(servo_feedback_from_angle(&calibration, 44, &result)
          && close_to(result, 1.9), "angle -> decreasing feedback");
    check(servo_angle_from_feedback(&calibration, 0.9, &result)
          && close_to(result, 105), "unequal interval width");
    check(servo_pulse_from_angle(&calibration, 105, &result)
          && close_to(result, 1600), "unequal interval -> pulse");
    check(servo_angle_from_feedback(&calibration, 2.5, &result)
          && close_to(result, 20), "decreasing feedback first endpoint");
    check(servo_angle_from_feedback(&calibration, 0.3, &result)
          && close_to(result, 150), "decreasing feedback last endpoint");
    result = -123;
    check(!servo_angle_from_feedback(&calibration, 2.6, &result)
          && result == -123, "reject above decreasing feedback range");
    check(!servo_angle_from_feedback(&calibration, 0.2, &result)
          && result == -123, "reject below decreasing feedback range");

    //Only a unit-conversion test; never change a real table's measured frequency.
    calibration.pwm_frequency_hz = 100;
    check(servo_pulse_from_angle(&calibration, 105, &result)
          && close_to(result, 800), "pulse conversion uses the supplied frequency");
}

//Remember only test counts, so each group can explain its own result.
static int group_start_checks = 0;
static int group_start_failures = 0;

static void begin_group(const char *title)
{
    group_start_checks = checks;
    group_start_failures = failures;
    printf("\n%s\n", title);
}

static void end_group(const char *pass_meaning, const char *failure_hint)
{
    int group_failures = failures - group_start_failures;
    int group_checks = checks - group_start_checks;
    if (group_failures == 0) {
        printf("PASS (%d checks): %s\n", group_checks, pass_meaning);
    } else {
        printf("FAIL (%d of %d checks): %s\n", group_failures, group_checks, failure_hint);
    }
}

//Example references only, not verified values for an installed joint.
static const double degrees_to_radians = 3.141592653589793 / 180.0;
static const double radians_to_degrees = 180.0 / 3.141592653589793;
static const JointMapping example_mapping = {
    30.0 * degrees_to_radians, //q_0 [rad]
    30.0,                     //a_0 [deg]
    -1                        //sigma: servo angle increases, model angle decreases
};

//Print actual and expected numbers on failure to help locate a unit/sign error.
static void check_value(int ok, double actual, double expected, const char *description)
{
    int passed = ok && close_to(actual, expected);
    check(passed, description);
    if (!passed) {
        printf("  Expected success=1, value=%.12g; got success=%d, value=%.12g\n",
               expected, ok, actual);
    }
}

//PASS: home offsets, both directions and degree/radian conversion match hand answers.
//FAIL: inspect the named equation, sigma or units; these tests use no robot readings.
static void test_mapping_references_and_directions(void)
{
    double q = NAN, a = NAN;
    int ok = joint_angle_from_servo(&example_mapping, 30, &q);
    check_value(ok, q, 0.5235987755982988, "home: a=30 deg -> q=30 deg in radians");
    ok = servo_angle_from_joint(&example_mapping, 0.5235987755982988, &a);
    check_value(ok, a, 30, "home: q=q_0 -> a=a_0");

    //30 - (45 - 30) = 15 degrees; expected radians are written independently.
    ok = joint_angle_from_servo(&example_mapping, 45, &q);
    check_value(ok, q, 0.2617993877991494, "sigma=-1: a=45 deg -> q=15 deg in radians");
    ok = servo_angle_from_joint(&example_mapping, 0.2617993877991494, &a);
    check_value(ok, a, 45, "sigma=-1: q=15 deg -> a=45 deg");

    //The measured other endpoint need not have a known CAD angle: calculate it.
    ok = joint_angle_from_servo(&example_mapping, 80, &q);
    check_value(ok, q, -0.3490658503988659, "away endpoint: a=80 deg -> q=-20 deg in radians");
    ok = servo_angle_from_joint(&example_mapping, -0.3490658503988659, &a);
    check_value(ok, a, 80, "reverse endpoint: q=-20 deg -> a=80 deg");

    //Unequal home angles prevent an accidental assumption that q_0 equals a_0.
    const JointMapping positive_mapping = {-20.0 * degrees_to_radians, 80.0, 1};
    ok = joint_angle_from_servo(&positive_mapping, 80, &q);
    check_value(ok, q, -0.3490658503988659, "unequal references: a_0=80 deg -> q_0=-20 deg");
    ok = servo_angle_from_joint(&positive_mapping, -0.3490658503988659, &a);
    check_value(ok, a, 80, "unequal references: q_0=-20 deg -> a_0=80 deg");
    ok = joint_angle_from_servo(&positive_mapping, 100, &q);
    check_value(ok, q, 0, "sigma=+1: -20 + (100-80) = 0 deg");
    ok = joint_angle_from_servo(&positive_mapping, 130, &q);
    check_value(ok, q, 0.5235987755982988, "sigma=+1: a=130 deg -> q=30 deg in radians");
    ok = servo_angle_from_joint(&positive_mapping, 0.5235987755982988, &a);
    check_value(ok, a, 130, "sigma=+1: q=30 deg -> a=130 deg");
}

//PASS: both physical endpoints become correctly ordered model limits.
//FAIL: inspect endpoint conversion and min/max sorting, especially for sigma=-1.
//This tests limit calculation, not collision clearance or command enforcement.
static void test_mapping_limits(void)
{
    double q_min = NAN, q_max = NAN, a = NAN;
    int ok = joint_limits_from_servo_limits(&example_mapping, 35, 95, &q_min, &q_max);
    check_value(ok, q_min, -0.6108652381980153, "limits sigma=-1: q_min=-35 deg in radians");
    check_value(ok, q_max, 0.4363323129985824, "limits sigma=-1: q_max=25 deg in radians");
    printf("  Example usable limits: servo [35, 95] deg -> model [%.3f, %.3f] deg\n",
           q_min * radians_to_degrees, q_max * radians_to_degrees);

    ok = servo_angle_from_joint(&example_mapping, q_min, &a);
    check_value(ok, a, 95, "negative sign: model minimum corresponds to servo maximum");
    ok = servo_angle_from_joint(&example_mapping, q_max, &a);
    check_value(ok, a, 35, "negative sign: model maximum corresponds to servo minimum");

    const JointMapping positive_mapping = {-20.0 * degrees_to_radians, 80.0, 1};
    ok = joint_limits_from_servo_limits(&positive_mapping, 35, 95, &q_min, &q_max);
    check_value(ok, q_min, -1.1344640137963142, "limits sigma=+1: q_min=-65 deg in radians");
    check_value(ok, q_max, -0.08726646259971647, "limits sigma=+1: q_max=-5 deg in radians");
}

//Invalid references must be rejected by every function without writing outputs.
static void check_invalid_mapping(const JointMapping *mapping, const char *reason)
{
    double q = -123, a = -456, q_min = -789, q_max = -987;
    char description[150];
    int ok = joint_angle_from_servo(mapping, 45, &q);
    snprintf(description, sizeof(description), "%s: reject servo -> joint; keep output", reason);
    check(!ok && q == -123, description);
    ok = servo_angle_from_joint(mapping, 0.2, &a);
    snprintf(description, sizeof(description), "%s: reject joint -> servo; keep output", reason);
    check(!ok && a == -456, description);
    ok = joint_limits_from_servo_limits(mapping, 35, 95, &q_min, &q_max);
    snprintf(description, sizeof(description), "%s: reject limits; keep both outputs", reason);
    check(!ok && q_min == -789 && q_max == -987, description);
}

//PASS here means refusing bad data, not successfully calculating an angle.
//FAIL means the named input guard or unchanged-output promise needs inspection.
static void test_mapping_rejections(void)
{
    check_invalid_mapping(NULL, "missing mapping");
    const JointMapping bad_mappings[] = {
        {NAN, 30, -1}, {INFINITY, 30, -1},
        {0, NAN, -1}, {0, INFINITY, -1},
        {0, 30, 0}, {0, 30, 2}, {0, 30, -2}
    };
    const char *reasons[] = {
        "q_0 is NaN", "q_0 is infinite", "a_0 is NaN", "a_0 is infinite",
        "sigma=0 (unconfigured)", "sigma=2", "sigma=-2"
    };
    for (int j = 0; j < 7; j++) check_invalid_mapping(&bad_mappings[j], reasons[j]);

    const double invalid_angles[] = {NAN, INFINITY, -INFINITY};
    for (int j = 0; j < 3; j++) {
        double result = -123;
        check(!joint_angle_from_servo(&example_mapping, invalid_angles[j], &result)
              && result == -123, "reject nonfinite servo angle; keep output");
        check(!servo_angle_from_joint(&example_mapping, invalid_angles[j], &result)
              && result == -123, "reject nonfinite model angle; keep output");
    }

    check(!joint_angle_from_servo(&example_mapping, 45, NULL), "reject null model-angle output");
    check(!servo_angle_from_joint(&example_mapping, 0.2, NULL), "reject null servo-angle output");

    const double bad_limits[][2] = {{95, 35}, {35, 35}, {NAN, 95}, {35, INFINITY}};
    const char *limit_reasons[] = {"reversed limits", "equal limits", "NaN endpoint", "infinite endpoint"};
    for (int j = 0; j < 4; j++) {
        double q_min = -123, q_max = -456;
        int ok = joint_limits_from_servo_limits(&example_mapping, bad_limits[j][0], bad_limits[j][1], &q_min, &q_max);
        check(!ok && q_min == -123 && q_max == -456, limit_reasons[j]);
    }

    double result = -123;
    check(!joint_limits_from_servo_limits(&example_mapping, 35, 95, NULL, &result)
          && result == -123, "reject missing minimum output; keep maximum");
    check(!joint_limits_from_servo_limits(&example_mapping, 35, 95, &result, NULL)
          && result == -123, "reject missing maximum output; keep minimum");
    check(!joint_limits_from_servo_limits(&example_mapping, 35, 95, &result, &result)
          && result == -123, "minimum and maximum need separate output variables");

    //Deliberately enormous inputs: a finite input can still overflow during maths.
    const JointMapping extreme_mapping = {0, -DBL_MAX, 1};
    check(!joint_angle_from_servo(&extreme_mapping, DBL_MAX, &result)
          && result == -123, "reject overflow in servo-angle difference");
    check(!servo_angle_from_joint(&example_mapping, DBL_MAX, &result)
          && result == -123, "reject overflow in radians-to-degrees conversion");
    double q_min = -123, q_max = -456;
    check(!joint_limits_from_servo_limits(&extreme_mapping, -DBL_MAX, DBL_MAX, &q_min, &q_max)
          && q_min == -123 && q_max == -456, "second endpoint failure must not write the first limit");
}

//PASS: both modules connect with the agreed units and independently known answers.
//FAIL: start with the FIRST failed stage; later failures may follow from its input.
//Passing does not verify these example home references on a real installed servo.
static void test_calibration_and_mapping_together(void)
{
    double a = NAN, q = NAN;
    int ok = servo_angle_from_feedback(&example_calibration, 0.685, &a);
    check_value(ok, a, 45, "feedback chain stage 1: 0.685 V -> servo 45 deg");
    ok = ok && joint_angle_from_servo(&example_mapping, a, &q);
    check_value(ok, q, 0.2617993877991494, "feedback chain stage 2: servo 45 deg -> model 15 deg in radians");
    printf("  Feedback chain: 0.685 V -> servo %.3f deg -> model %.6f rad (%.3f deg)\n",
           a, q, q * radians_to_degrees);

    //Start the command test from an independent known q, not the previous result.
    double commanded_a = NAN, pulse = NAN;
    ok = servo_angle_from_joint(&example_mapping, 0.2617993877991494, &commanded_a);
    check_value(ok, commanded_a, 45, "command chain stage 1: model 15 deg -> servo 45 deg");
    ok = ok && servo_pulse_from_angle(&example_calibration, commanded_a, &pulse);
    check_value(ok, pulse, 820, "command chain stage 2: servo 45 deg -> pulse 820 us at 50 Hz");
    printf("  Command chain: model 15.000 deg -> servo %.3f deg -> pulse %.3f us\n",
           commanded_a, pulse);

    //Mapping can produce an angle outside the table. Calibration must reject it.
    pulse = -123;
    ok = servo_angle_from_joint(&example_mapping, -180.0 * degrees_to_radians, &commanded_a);
    check_value(ok, commanded_a, 240, "mapping itself does not clamp a request to the table");
    check(!servo_pulse_from_angle(&example_calibration, commanded_a, &pulse) && pulse == -123,
          "reject pulse request for servo 240 deg, outside the calibration table");
}

//PASS: all stored cells match a frozen copy extracted independently from the XLSX.
//The reference CSV belongs to desktop tests only; firmware does not read a file.
//FAIL: the message identifies the servo, spreadsheet row and field to compare.
static void test_imported_servo_tables(const char *reference_path)
{
    FILE *file = fopen(reference_path, "r");
    check(file != NULL, "open calibration reference CSV (run the provided VS Code task)");
    if (file == NULL) return;

    char line[180], description[160];
    int seen[SERVO_COUNT][19] = {{0}};
    int row_count = 0;
    int header_ok = fgets(line, sizeof(line), file) != NULL
        && strcmp(line, "servo_id,spreadsheet_row,servo_angle_deg,feedback_V,duty_percent\n") == 0;
    check(header_ok, "reference CSV has the expected five columns");
    if (!header_ok) { fclose(file); return; }

    while (fgets(line, sizeof(line), file) != NULL) {
        int id = 0, source_row = 0;
        double expected_a = NAN, expected_V = NAN, expected_D = NAN;
        int fields = sscanf(line, "%d,%d,%lf,%lf,%lf", &id, &source_row, &expected_a, &expected_V, &expected_D);
        int row_ok = fields == 5 && id >= 1 && id <= SERVO_COUNT && source_row >= 3 && source_row <= 21;
        check(row_ok, "reference row contains a valid ID, sheet row and three numbers");
        if (!row_ok) continue;

        int j = source_row - 3; //Spreadsheet row 3 is points[0].
        seen[id - 1][j]++;
        row_count++;
        const ServoCalibration *calibration = servo_calibration_for_id(id);
        snprintf(description, sizeof(description), "S%02d: table exists with 19 rows", id);
        int table_ok = calibration != NULL && calibration->points != NULL && calibration->point_count == 19;
        check(table_ok, description);
        if (!table_ok) continue;

        //Read expected values from the reference, not from the production array.
        snprintf(description, sizeof(description), "S%02d sheet row %d: stored angle [deg]", id, source_row);
        check_value(1, calibration->points[j].servo_angle_deg, expected_a, description);
        snprintf(description, sizeof(description), "S%02d sheet row %d: stored feedback [V]", id, source_row);
        check_value(1, calibration->points[j].feedback_V, expected_V, description);
        snprintf(description, sizeof(description), "S%02d sheet row %d: stored duty [percent]", id, source_row);
        check_value(1, calibration->points[j].duty_percent, expected_D, description);

        double a = NAN, V = NAN, pulse = NAN;
        int ok = servo_angle_from_feedback(calibration, expected_V, &a);
        snprintf(description, sizeof(description), "S%02d sheet row %d: feedback -> angle [deg]", id, source_row);
        check_value(ok, a, expected_a, description);
        ok = servo_feedback_from_angle(calibration, expected_a, &V);
        snprintf(description, sizeof(description), "S%02d sheet row %d: angle -> feedback [V]", id, source_row);
        check_value(ok, V, expected_V, description);
        ok = servo_pulse_from_angle(calibration, expected_a, &pulse);
        snprintf(description, sizeof(description), "S%02d sheet row %d: angle -> pulse [us] at 50 Hz", id, source_row);
        check_value(ok, pulse, expected_D * 200.0, description);
    }

    int read_ok = !ferror(file);
    check(fclose(file) == 0 && read_ok, "read and close the complete reference CSV");
    check(row_count == 228, "reference contains 12 servos x 19 rows");
    for (int id = 1; id <= SERVO_COUNT; id++) {
        int all_rows_once = 1;
        for (int j = 0; j < 19; j++) if (seen[id - 1][j] != 1) all_rows_once = 0;
        snprintf(description, sizeof(description), "S%02d: every source row occurs exactly once", id);
        check(all_rows_once, description);
        const ServoCalibration *calibration = servo_calibration_for_id(id);
        snprintf(description, sizeof(description), "S%02d: user-confirmed calibration frequency is 50 Hz", id);
        check_value(calibration != NULL, calibration != NULL ? calibration->pwm_frequency_hz : NAN, 50.0, description);
    }
}

//PASS: each real table interpolates a selected between-row example correctly.
//FAIL: inspect the named servo's rows around 40/50 degrees and conversion units.
static void test_configured_interpolation_and_ranges(void)
{
    //Independent source calculations: 42.5 degrees is one quarter from 40 to 50.
    //Columns: ID, expected voltage [V], expected pulse [us] at 50 Hz.
    const double examples[12][3] = {
        {1, 0.71700, 830.000}, //S01
        {2, 0.6475, 785.000}, //S02
        {3, 0.6825, 785.000}, //S03
        {4, 0.6800, 805.000}, //S04
        {5, 0.6875, 825.000}, //S05
        {6, 0.6800, 805.000}, //S06
        {7, 0.6800, 825.000}, //S07
        {8, 0.6475, 790.000}, //S08
        {9, 0.6425, 790.000}, //S09
        {10, 0.6400, 785.000}, //S10
        {11, 0.6800, 805.000}, //S11
        {12, 0.6800, 810.000}, //S12
    };

    for (int j = 0; j < 12; j++) {
        int id = (int)examples[j][0];
        const ServoCalibration *calibration = servo_calibration_for_id(id);
        char description[160];
        snprintf(description, sizeof(description), "S%02d: calibration available for interpolation", id);
        int table_ok = calibration != NULL && calibration->points != NULL && calibration->point_count >= 2;
        check(table_ok, description);
        if (!table_ok) continue;

        double a = NAN, V = NAN, pulse = NAN;
        int ok = servo_angle_from_feedback(calibration, examples[j][1], &a);
        snprintf(description, sizeof(description), "S%02d: between-row feedback -> 42.5 deg", id);
        check_value(ok, a, 42.5, description);
        ok = servo_feedback_from_angle(calibration, 42.5, &V);
        snprintf(description, sizeof(description), "S%02d: 42.5 deg -> expected feedback [V]", id);
        check_value(ok, V, examples[j][1], description);
        ok = servo_pulse_from_angle(calibration, 42.5, &pulse);
        snprintf(description, sizeof(description), "S%02d: 42.5 deg -> expected pulse [us]", id);
        check_value(ok, pulse, examples[j][2], description);

        //These are table coverage tests, not tests of assembled mechanical limits.
        const ServoCalibrationPoint *first = &calibration->points[0];
        const ServoCalibrationPoint *last = &calibration->points[calibration->point_count - 1];
        const double bad_voltages[] = {fmin(first->feedback_V, last->feedback_V) - 0.001,
                                       fmax(first->feedback_V, last->feedback_V) + 0.001};
        const double bad_angles[] = {first->servo_angle_deg - 0.01, last->servo_angle_deg + 0.01};
        for (int side = 0; side < 2; side++) {
            double result = -123;
            snprintf(description, sizeof(description), "S%02d: reject %s-table voltage; keep output", id, side == 0 ? "below" : "above");
            check(!servo_angle_from_feedback(calibration, bad_voltages[side], &result) && result == -123, description);
            snprintf(description, sizeof(description), "S%02d: reject %s-table angle for voltage; keep output", id, side == 0 ? "below" : "above");
            check(!servo_feedback_from_angle(calibration, bad_angles[side], &result) && result == -123, description);
            snprintf(description, sizeof(description), "S%02d: reject %s-table angle for pulse; keep output", id, side == 0 ? "below" : "above");
            check(!servo_pulse_from_angle(calibration, bad_angles[side], &result) && result == -123, description);
        }
    }
}

//PASS: assignments match your list and every ID occurs once. FAIL: check row/column/ID.
//Expected IDs below come from your message, not from the production assignment array.
static void test_servo_installation_assignments(void)
{
    const int expected_ids[4][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}};
    const char *leg_names[4] = {"FL", "FR", "RL", "RR"};
    const char *joint_names[3] = {"hip roll", "hip pitch", "knee pitch"};
    int uses[SERVO_COUNT] = {0};
    char description[160];

    check(SERVO_LEG_FL == 0 && SERVO_LEG_FR == 1 && SERVO_LEG_RL == 2 && SERVO_LEG_RR == 3,
          "leg constants retain FL, FR, RL, RR order");
    check(SERVO_JOINT_HIP_ROLL == 0 && SERVO_JOINT_HIP_PITCH == 1 && SERVO_JOINT_KNEE_PITCH == 2,
          "joint constants retain roll, pitch, knee pitch order");
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            int id = servo_id_for_joint(leg, joint);
            snprintf(description, sizeof(description), "%s %s: expected S%02d, got ID %d",
                     leg_names[leg], joint_names[joint], expected_ids[leg][joint], id);
            check(id == expected_ids[leg][joint], description);
            if (id >= 1 && id <= SERVO_COUNT) uses[id - 1]++;

            const ServoCalibration *calibration = servo_calibration_for_id(id);
            snprintf(description, sizeof(description), "%s %s: retrieves its expected calibration object", leg_names[leg], joint_names[joint]);
            check(calibration != NULL && calibration == servo_calibration_for_id(expected_ids[leg][joint]), description);
            printf("  %s %-10s -> S%02d\n", leg_names[leg], joint_names[joint], id);
        }
    }
    for (int id = 1; id <= SERVO_COUNT; id++) {
        snprintf(description, sizeof(description), "S%02d: assigned exactly once", id);
        check(uses[id - 1] == 1, description);
    }
}

//PASS means bad requests are refused, rather than silently choosing another motor.
static void test_servo_lookup_rejections(void)
{
    const int bad_ids[] = {-1, 0, 13, 999};
    const int bad_legs[] = {-1, 4, 999};
    const int bad_joints[] = {-1, 3, 999};
    char description[120];
    for (int j = 0; j < 4; j++) {
        snprintf(description, sizeof(description), "invalid/unassigned servo ID %d -> NULL", bad_ids[j]);
        check(servo_calibration_for_id(bad_ids[j]) == NULL, description);
    }
    for (int j = 0; j < 3; j++) {
        snprintf(description, sizeof(description), "invalid leg index %d -> SERVO_UNASSIGNED", bad_legs[j]);
        check(servo_id_for_joint(bad_legs[j], SERVO_JOINT_HIP_ROLL) == SERVO_UNASSIGNED, description);
        snprintf(description, sizeof(description), "invalid joint index %d -> SERVO_UNASSIGNED", bad_joints[j]);
        check(servo_id_for_joint(SERVO_LEG_FL, bad_joints[j]) == SERVO_UNASSIGNED, description);
    }
}

//PASS: all 12 q_0/s_q values match the recorded CAD inspection, with correct units.
//FAIL: check the named leg/joint, radians conversion or copied reference value.
static void test_cad_joint_references(const char *reference_path)
{
    FILE *file = fopen(reference_path, "r");
    check(file != NULL, "open CAD joint reference CSV");
    if (file == NULL) return;

    char line[200], description[160];
    int seen[4][3] = {{0}};
    int row_count = 0;
    int header_ok = fgets(line, sizeof(line), file) != NULL
        && strcmp(line, "leg,joint,servo_id,q_0_deg,s_q\n") == 0;
    check(header_ok, "CAD reference columns: leg, joint, servo ID, degrees, away sign");
    if (!header_ok) { fclose(file); return; }

    while (fgets(line, sizeof(line), file) != NULL) {
        int leg = -1, joint = -1, id = 0, s_q = 0;
        double q_0_deg = NAN;
        int fields = sscanf(line, "%d,%d,%d,%lf,%d", &leg, &joint, &id, &q_0_deg, &s_q);
        int row_ok = fields == 5 && leg >= 0 && leg < 4 && joint >= 0 && joint < 3
                  && id >= 1 && id <= 12 && isfinite(q_0_deg) && (s_q == 1 || s_q == -1);
        check(row_ok, "CAD reference row has valid indices, angle and sign");
        if (!row_ok) continue;
        seen[leg][joint]++;
        row_count++;

        const JointCadReference *reference = joint_cad_reference_for_joint(leg, joint);
        snprintf(description, sizeof(description), "leg %d joint %d: CAD reference exists", leg, joint);
        check(reference != NULL, description);
        if (reference == NULL) continue;
        snprintf(description, sizeof(description), "S%02d: CAD row identifies its installed joint", id);
        check(servo_id_for_joint(leg, joint) == id, description);
        snprintf(description, sizeof(description), "S%02d: q_0 [rad] agrees with CAD degrees", id);
        check_value(1, reference->model_home_rad, q_0_deg * degrees_to_radians, description);
        snprintf(description, sizeof(description), "S%02d: s_q agrees with the CAD away direction", id);
        check(reference->model_away_sign == s_q, description);
        printf("  S%02d: q_0 = %+.6f deg, s_q = %+d\n",
               id, reference->model_home_rad * radians_to_degrees, reference->model_away_sign);
    }
    int read_ok = !ferror(file);
    check(fclose(file) == 0 && read_ok, "read and close the CAD joint reference");
    check(row_count == 12, "CAD reference contains all 12 joints");
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            snprintf(description, sizeof(description), "leg %d joint %d: exactly one CAD row", leg, joint);
            check(seen[leg][joint] == 1, description);
        }
    }
    const int bad_indices[] = {-1, 4, 999};
    for (int j = 0; j < 3; j++) {
        check(joint_cad_reference_for_joint(bad_indices[j], 0) == NULL, "reject invalid CAD leg index");
        check(joint_cad_reference_for_joint(0, bad_indices[j]) == NULL, "reject invalid CAD joint index");
    }
    check(joint_cad_reference_for_joint(0, 3) == NULL, "reject first out-of-range CAD joint index");
}

//PASS: real CAD references work with either measured servo direction.
//a_0=90 and s_a=+/-1 are fictional test inputs, never installed calibration values.
static void test_cad_references_with_mapping(void)
{
    char description[160];
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            const JointCadReference *reference = joint_cad_reference_for_joint(leg, joint);
            check(reference != NULL, "CAD reference available for mapping test");
            if (reference == NULL) continue;
            for (int s_a = -1; s_a <= 1; s_a += 2) {
                JointMapping mapping = {reference->model_home_rad, 90.0, reference->model_away_sign * s_a};
                double q = NAN, a = NAN;
                int ok = joint_angle_from_servo(&mapping, 90.0, &q);
                snprintf(description, sizeof(description), "leg %d joint %d s_a=%+d: a_0 -> q_0", leg, joint, s_a);
                check_value(ok, q, reference->model_home_rad, description);
                ok = servo_angle_from_joint(&mapping, reference->model_home_rad, &a);
                snprintf(description, sizeof(description), "leg %d joint %d s_a=%+d: q_0 -> a_0", leg, joint, s_a);
                check_value(ok, a, 90.0, description);

                //Move 10 degrees away: servo sign is s_a; mathematical sign is s_q.
                double servo_away = 90.0 + s_a * 10.0;
                double expected_q = reference->model_home_rad + reference->model_away_sign * 10.0 * degrees_to_radians;
                ok = joint_angle_from_servo(&mapping, servo_away, &q);
                snprintf(description, sizeof(description), "leg %d joint %d s_a=%+d: 10 deg away follows s_q", leg, joint, s_a);
                check_value(ok, q, expected_q, description);
                ok = servo_angle_from_joint(&mapping, expected_q, &a);
                snprintf(description, sizeof(description), "leg %d joint %d s_a=%+d: reverse away conversion", leg, joint, s_a);
                check_value(ok, a, servo_away, description);
            }
        }
    }
}

//PASS: configured q_0 and geometry reconstruct independent CAD P, K and F points.
//FAIL: check units, leg offsets, L2 or angle convention; do not alter the CAD fixture.
static void test_cad_home_forward_kinematics(const char *reference_path)
{
    FILE *file = fopen(reference_path, "r");
    check(file != NULL, "open CAD home point CSV");
    if (file == NULL) return;
    char line[200], description[160];
    int seen[4][3] = {{0}};
    int row_count = 0;
    int header_ok = fgets(line, sizeof(line), file) != NULL
        && strcmp(line, "leg,point,x_H_mm,y_H_mm,z_H_mm\n") == 0;
    check(header_ok, "CAD point columns use the fixed hip frame H and millimetres");
    if (!header_ok) { fclose(file); return; }

    while (fgets(line, sizeof(line), file) != NULL) {
        int leg = -1, point = 0;
        double expected_mm[3] = {NAN, NAN, NAN};
        int fields = sscanf(line, "%d,%d,%lf,%lf,%lf", &leg, &point,
                            &expected_mm[0], &expected_mm[1], &expected_mm[2]);
        int row_ok = fields == 5 && leg >= 0 && leg < 4 && point >= 1 && point <= 3;
        check(row_ok, "CAD point row identifies a leg and P=1, K=2 or F=3");
        if (!row_ok) continue;
        seen[leg][point - 1]++;
        row_count++;
        const JointCadReference *roll = joint_cad_reference_for_joint(leg, 0);
        const JointCadReference *pitch = joint_cad_reference_for_joint(leg, 1);
        const JointCadReference *knee = joint_cad_reference_for_joint(leg, 2);
        int references_ok = roll != NULL && pitch != NULL && knee != NULL;
        check(references_ok, "all three CAD references available for FK");
        if (!references_ok) continue;

        LegJointAngles q = {roll->model_home_rad, pitch->model_home_rad, knee->model_home_rad};
        LegPointsHip points = leg_forward_kinematics(q, ik_config.upper_link_length_m,
            ik_config.lower_link_length_m, ik_config.hip_roll_to_pitch_m[leg],
            ik_config.upper_lateral_offset_m[leg], ik_config.lower_lateral_offset_m[leg]);
        const double *actual = point == 1 ? points.P : (point == 2 ? points.K : points.F);
        for (int axis = 0; axis < 3; axis++) {
            snprintf(description, sizeof(description), "leg %d point %d axis %d: FK matches CAD [m]", leg, point, axis);
            check_value(1, actual[axis], expected_mm[axis] / 1000.0, description);
        }
    }
    int read_ok = !ferror(file);
    check(fclose(file) == 0 && read_ok, "read and close CAD point reference");
    check(row_count == 12, "CAD points contain P, K and F for all four legs");
    for (int leg = 0; leg < 4; leg++) {
        for (int point = 0; point < 3; point++) check(seen[leg][point] == 1, "each CAD leg/point appears once");
    }
}

//This checks the current desktop home-start example, not every possible starting pose.
//PASS: supplied angles equal q_0 and the chosen roll gives one height for all feet.
static void test_home_starting_configuration(void)
{
    double R_WB[3][3];
    body_rotation_matrix(initial_body_pose.yaw_rad, initial_command.body_pitch_rad,
                         initial_command.body_roll_rad, R_WB);
    double heights[4], H0 = 0;
    char description[160];
    for (int leg = 0; leg < 4; leg++) {
        LegJointAngles q = initial_joint_angles[leg];
        const double angles[3] = {q.alpha_rad, q.theta_1_rad, q.theta_2_rad};
        for (int joint = 0; joint < 3; joint++) {
            const JointCadReference *reference = joint_cad_reference_for_joint(leg, joint);
            snprintf(description, sizeof(description), "leg %d joint %d: desktop initial angle equals q_0", leg, joint);
            check_value(reference != NULL, angles[joint], reference != NULL ? reference->model_home_rad : NAN, description);
        }
        LegPointsHip points = leg_forward_kinematics(q, ik_config.upper_link_length_m,
            ik_config.lower_link_length_m, ik_config.hip_roll_to_pitch_m[leg],
            ik_config.upper_lateral_offset_m[leg], ik_config.lower_lateral_offset_m[leg]);
        //H_i = R_f - [R_WB * (h_i + F_H)]_z, the same grounded-height equation.
        double rotated_z = 0;
        for (int axis = 0; axis < 3; axis++) {
            rotated_z += R_WB[2][axis] * (robot_hip_position_B_m[leg][axis] + points.F[axis]);
        }
        heights[leg] = robot_ball_radius_m - rotated_z;
        H0 += heights[leg] / 4.0;
    }
    for (int leg = 0; leg < 4; leg++) {
        snprintf(description, sizeof(description), "leg %d: home foot agrees with common body height", leg);
        check_value(1, heights[leg], H0, description);
    }
    check_value(1, H0, 0.0811961020301, "CAD-home example starts at H0=0.081196102 m");
    printf("  Home-start H0 = %.9f m; body roll = %.6f deg\n", H0, initial_command.body_roll_rad * radians_to_degrees);
}

//Each group reports what passed and where to investigate a failure.
int main(int argc, char *argv[])
{
    const char *reference_path = argc > 1 ? argv[1] : "tests/data/servo_calibration_reference.csv";
    const char *cad_reference_path = argc > 2 ? argv[2] : "tests/data/cad_joint_references.csv";
    const char *cad_points_path = argc > 3 ? argv[3] : "tests/data/cad_home_points.csv";
    printf("ServoCalibration + JointMapping + configuration desktop tests - no robot or PWM output\n");
    printf("Groups 1-8 use examples. Groups 9-12 use your imported tables and assignments.\n");
    printf("Groups 13-16 check the CAD references, mapping, FK and current home-start example.\n");

    begin_group("1. Calibration: stored table entries");
    test_table_entries();
    end_group("Every stored row produces its expected angle, voltage and pulse width.",
              "Check the named row, table lookup and percent-to-microsecond conversion.");

    begin_group("2. Calibration: between-entry interpolation");
    test_between_entries();
    end_group("The tested between-row values match hand-calculated answers.",
              "Check which two rows were selected and the interpolation fraction.");

    begin_group("3. Calibration: invalid inputs");
    test_rejections();
    end_group("The tested bad inputs/tables are rejected and outputs are preserved.",
              "Check the named validation rule; rejection is the expected result here.");

    begin_group("4. Calibration: other table shapes");
    test_other_tables();
    end_group("Decreasing voltages, uneven intervals and the frequency conversion work.",
              "Check interval direction, interval width or the supplied frequency.");

    begin_group("5. Mapping: home references, signs and units");
    test_mapping_references_and_directions();
    end_group("Both signs and unequal home references match independent angle answers.",
              "Check q_0, a_0, sigma and the degree/radian factors in the named function.");

    begin_group("6. Mapping: usable endpoint conversion");
    test_mapping_limits();
    end_group("Servo endpoints produce correctly sorted model limits for both signs.",
              "Check endpoint mapping and min/max sorting; sigma=-1 reverses the order.");

    begin_group("7. Mapping: invalid inputs");
    test_mapping_rejections();
    end_group("The tested invalid references/arguments are rejected without changing outputs.",
              "Check the named guard or output write; rejection is expected here.");

    begin_group("8. Calibration + mapping: complete conversion chains");
    test_calibration_and_mapping_together();
    end_group("Voltage -> servo -> model and model -> servo -> pulse match known answers.",
              "Inspect the first failed stage and its input units before later stages.");

    begin_group("9. Configuration: all spreadsheet values and table entries");
    test_imported_servo_tables(reference_path);
    end_group("All 228 rows match the source reference and convert correctly at 50 Hz.",
              "Compare the named servo/sheet row with the source; do not change expected data just to pass.");

    begin_group("10. Configuration: interpolation and table coverage for all 12 servos");
    test_configured_interpolation_and_ranges();
    end_group("Selected between-row answers match the source; out-of-table inputs are rejected.",
              "Check the named servo's selected interval or range check.");

    begin_group("11. Configuration: leg/joint assignments");
    test_servo_installation_assignments();
    end_group("All 12 assignments match your list; each physical servo ID occurs once.",
              "Check FL/FR/RL/RR row order, roll/pitch/knee columns and the named servo ID.");

    begin_group("12. Configuration: invalid lookup requests");
    test_servo_lookup_rejections();
    end_group("Invalid IDs and leg/joint indices do not select a different motor.",
              "Check the ID or index guard in the named lookup.");

    begin_group("13. CAD configuration: q_0 and away direction for all 12 joints");
    test_cad_joint_references(cad_reference_path);
    end_group("All home angles and away signs match the CAD inspection; invalid indices are rejected.",
              "Check the named joint, stored units and reference row from the CAD report.");

    begin_group("14. CAD + mapping: both possible measured servo directions");
    test_cad_references_with_mapping();
    end_group("Home and away conversions work for either s_a; a_0 and s_a here are fictional test inputs.",
              "Check sigma=s_q*s_a and the degree/radian conversions.");

    begin_group("15. CAD + FK: reconstruct the measured home geometry");
    test_cad_home_forward_kinematics(cad_points_path);
    end_group("Configured home angles and geometry reproduce all four legs' CAD P, K and F points.",
              "Check the named coordinate, L2, offsets and q_0; expected data comes from CAD.");

    begin_group("16. Desktop example: exact CAD-home starting angles");
    test_home_starting_configuration();
    end_group("The current home-start angles and small roll put all four feet on the same floor.",
              "Check the starting angles and body roll; revise this example test if choosing a different start.");

    printf("\n%d checks: %d passed, %d failed.\n", checks, checks - failures, failures);
    if (failures == 0) {
        printf("PASS: ServoCalibration + JointMapping + configuration desktop tests.\n");
        printf("Conclusion: calibration, joint references, mapping and CAD-home FK pass desktop checks.\n");
    } else {
        printf("FAIL: investigate the labelled checks above before using these results.\n");
    }
    printf("Still unmeasured: installed a_0/s_a, sensor accuracy and physical joint limits. No motor was moved.\n");
    return failures == 0 ? 0 : 1;
}
