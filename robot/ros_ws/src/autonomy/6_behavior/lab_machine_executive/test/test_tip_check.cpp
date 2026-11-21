// Tip check after each OT-2 aspirate: the client of the d3:4b tip-check service and the step that
// runs it. All HTTP is faked; nothing here reaches the OT-2 or the camera service.
#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <lab_machine_executive/tip_check.hpp>

using nlohmann::json;
using lab_machine::HttpClient;
using lab_machine::HttpResponse;
using lab_machine::TipCheckClient;
using lab_machine::TipCheckConfig;

namespace {

const std::string BASE = "http://tipcheck.test:8110";

HttpResponse ok(const json& body) { return {true, 200, body.dump(), ""}; }

// Answers each URL from its own queue (the last answer repeats) and records every request.
class FakeHttp : public HttpClient {
   public:
    std::map<std::string, std::deque<HttpResponse>> answers;
    std::vector<std::pair<std::string, std::string>> posts;  // (url, body)
    std::vector<std::string> gets;

    void answer(const std::string& path, HttpResponse r) { answers[BASE + path].push_back(r); }

    HttpResponse post(const std::string& url, const std::string& body) override {
        posts.emplace_back(url, body);
        return next(url);
    }
    HttpResponse get(const std::string& url) override {
        gets.push_back(url);
        return next(url);
    }

   private:
    HttpResponse next(const std::string& url) {
        auto it = answers.find(url);
        if (it == answers.end() || it->second.empty()) return {false, 0, "", "connection refused"};
        HttpResponse r = it->second.front();
        if (it->second.size() > 1) it->second.pop_front();
        return r;
    }
};

json done_job(double p, const std::string& clip = "clip") {
    return {{"clip_id", clip}, {"status", "done"}, {"p_tip_empty", p}, {"p_tip_visible", 0.9},
            {"threshold", 0.5}, {"model", "d3:4b"},
            {"files", {{"crop", "/runs/" + clip + "/crop.jpg"},
                       {"preview", "/runs/" + clip + "/preview.jpg"}}},
            {"timings_s", {{"total", 2.0}}}};
}

struct Clock {
    double t = 0;
    std::vector<double> sleeps;
};

TipCheckClient make_client(FakeHttp& http, Clock& clock, int passes = 5, double timeout_s = 900) {
    TipCheckConfig cfg;
    cfg.base_url = BASE;
    cfg.passes = passes;
    cfg.timeout_s = timeout_s;
    cfg.poll_s = 1.0;
    return TipCheckClient(
        http, cfg, [&clock](double s) { clock.sleeps.push_back(s); clock.t += s; },
        [&clock]() { return clock.t; });
}

}  // namespace

// ── One verdict from five checks ───────────────────────────────────────────────

TEST(TipCheck, FiveCompletedChecksAreAveraged) {
    FakeHttp http;
    Clock clock;
    for (double p : {0.6, 0.7, 0.8, 0.9, 1.0}) http.answer("/check", ok(done_job(p)));
    json r = make_client(http, clock).run("aspirate tube_rack A1");

    ASSERT_EQ(http.posts.size(), 5u);
    EXPECT_EQ(http.posts[0].first, BASE + "/check");
    EXPECT_EQ(json::parse(http.posts[0].second), (json{{"step", "aspirate tube_rack A1"}}));
    EXPECT_TRUE(http.gets.empty());
    EXPECT_EQ(r["status"], "done");
    EXPECT_NEAR(r["p_tip_empty"].get<double>(), 0.8, 1e-9);
    EXPECT_TRUE(r["anomaly_detected"].get<bool>());
    EXPECT_NEAR(r["confidence"].get<double>(), 0.8, 1e-9);
    EXPECT_EQ(r["sample_count"], 5);
    EXPECT_EQ(r["samples"].size(), 5u);
    EXPECT_EQ(r["model"], "d3:4b");
    EXPECT_EQ(r["threshold"], 0.5);
    EXPECT_NEAR(r["p_tip_visible"].get<double>(), 0.9, 1e-9);
    EXPECT_NEAR(r["timings_s"]["total"].get<double>(), 10.0, 1e-9);
    EXPECT_TRUE(r["error"].is_null());
    EXPECT_TRUE(r["warning"].is_null());
}

TEST(TipCheck, MeanBelowThresholdIsLiquidInTip) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok(done_job(0.1)));
    json r = make_client(http, clock).run("x");
    EXPECT_FALSE(r["anomaly_detected"].get<bool>());
    EXPECT_NEAR(r["confidence"].get<double>(), 0.9, 1e-9);
}

TEST(TipCheck, MeanEqualToThresholdIsAClog) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok(done_job(0.5)));
    EXPECT_TRUE(make_client(http, clock).run("x")["anomaly_detected"].get<bool>());
}

TEST(TipCheck, MissingThresholdDefaultsToHalf) {
    FakeHttp http;
    Clock clock;
    json job = done_job(0.55);
    job.erase("threshold");
    http.answer("/check", ok(job));
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["threshold"], 0.5);
    EXPECT_TRUE(r["anomaly_detected"].get<bool>());
}

TEST(TipCheck, ImagesComeFromTheFirstCheck) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok(done_job(0.1, "first")));
    http.answer("/check", ok(done_job(0.1, "second")));
    json r = make_client(http, clock, 2).run("x");
    EXPECT_EQ(r["files"]["crop"], "/runs/first/crop.jpg");
    EXPECT_EQ(r["clip_id"], "first");
}

TEST(TipCheck, VisibilityIsOnlyAveragedWhenEveryCheckHasIt) {
    FakeHttp http;
    Clock clock;
    json job = done_job(0.1);
    job.erase("p_tip_visible");
    http.answer("/check", ok(done_job(0.1)));
    http.answer("/check", ok(job));
    json r = make_client(http, clock, 2).run("x");
    EXPECT_TRUE(!r.contains("p_tip_visible") || r["p_tip_visible"].is_null());
}

TEST(TipCheck, WarningsAreCountedAndTheFirstIsKept) {
    FakeHttp http;
    Clock clock;
    json warned = done_job(0.1);
    warned["warning"] = "tip partly out of frame";
    http.answer("/check", ok(done_job(0.1)));
    http.answer("/check", ok(warned));
    json r = make_client(http, clock, 2).run("x");
    EXPECT_EQ(r["warning"], "1 of 2 checks reported a warning. tip partly out of frame");
}

// ── Queued jobs: follow that exact job on /jobs ────────────────────────────────

TEST(TipCheck, QueuedJobIsFollowedOnJobsByClipId) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"clip_id", "mine"}, {"status", "queued"}}));
    http.answer("/jobs", ok(json::array({done_job(0.9, "someone-else"),
                                         {{"clip_id", "mine"}, {"status", "running"}}})));
    http.answer("/jobs", ok({{"jobs", json::array({done_job(0.2, "mine")})}}));
    json r = make_client(http, clock, 1).run("x");
    EXPECT_EQ(r["status"], "done");
    EXPECT_NEAR(r["p_tip_empty"].get<double>(), 0.2, 1e-9);
    EXPECT_EQ(http.gets.size(), 2u);
    EXPECT_EQ(http.gets[0], BASE + "/jobs");
    EXPECT_EQ(clock.sleeps, (std::vector<double>{1.0, 1.0}));
}

TEST(TipCheck, QueuedJobWithoutClipIdIsAnError) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"status", "queued"}}));
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("no job ID"), std::string::npos);
}

TEST(TipCheck, QueuedJobThatNeverFinishesTimesOut) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"clip_id", "mine"}, {"status", "queued"}}));
    http.answer("/jobs", ok(json::array({{{"clip_id", "mine"}, {"status", "running"}}})));
    json r = make_client(http, clock, 5, 10).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("did not finish"), std::string::npos);
    EXPECT_LE(clock.t, 11.0);
}

TEST(TipCheck, FailedJobReportsItsError) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"clip_id", "c"}, {"status", "failed"}, {"error", "camera busy"}}));
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("camera busy"), std::string::npos);
}

// ── Service problems are reported, never thrown ────────────────────────────────

TEST(TipCheck, UnreachableServiceIsAnError) {
    FakeHttp http;
    Clock clock;
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("unreachable"), std::string::npos);
    EXPECT_NE(r["error"].get<std::string>().find(BASE), std::string::npos);
    EXPECT_EQ(http.posts.size(), 1u);  // stops at the first failed pass
}

TEST(TipCheck, HttpErrorUsesTheServiceMessage) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", {true, 503, json{{"error", "model loading"}}.dump(), ""});
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("model loading"), std::string::npos);
}

TEST(TipCheck, HttpErrorWithoutMessageNamesTheCode) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", {true, 500, "oops", ""});
    json r = make_client(http, clock).run("x");
    EXPECT_NE(r["error"].get<std::string>().find("HTTP 500"), std::string::npos);
}

TEST(TipCheck, UnparseableResponseIsAnError) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", {true, 200, "<html>", ""});
    EXPECT_EQ(make_client(http, clock).run("x")["status"], "error");
}

TEST(TipCheck, InvalidProbabilityNamesTheCheck) {
    FakeHttp http;
    Clock clock;
    json bad = done_job(0.1);
    bad["p_tip_empty"] = 1.5;
    http.answer("/check", ok(done_job(0.1)));
    http.answer("/check", ok(bad));
    json r = make_client(http, clock).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("check 2 of 5"), std::string::npos);
}

TEST(TipCheck, MissingProbabilityIsAnError) {
    FakeHttp http;
    Clock clock;
    json bad = done_job(0.1);
    bad.erase("p_tip_empty");
    http.answer("/check", ok(bad));
    EXPECT_EQ(make_client(http, clock).run("x")["status"], "error");
}

TEST(TipCheck, ModelChangeDuringTheChecksIsAnError) {
    FakeHttp http;
    Clock clock;
    json other = done_job(0.1);
    other["model"] = "d3:4c";
    http.answer("/check", ok(done_job(0.1)));
    http.answer("/check", ok(other));
    json r = make_client(http, clock, 2).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("model"), std::string::npos);
}

TEST(TipCheck, ThresholdChangeDuringTheChecksIsAnError) {
    FakeHttp http;
    Clock clock;
    json other = done_job(0.1);
    other["threshold"] = 0.6;
    http.answer("/check", ok(done_job(0.1)));
    http.answer("/check", ok(other));
    json r = make_client(http, clock, 2).run("x");
    EXPECT_EQ(r["status"], "error");
    EXPECT_NE(r["error"].get<std::string>().find("threshold"), std::string::npos);
}

// ── Running it after an OT-2 protocol step ─────────────────────────────────────

namespace {

struct StepRun {
    std::vector<json> dispatched;
    std::vector<std::string> checked;
    std::vector<json> published;
    bool dispatch_ok = true;
    json check_result = {{"status", "done"}, {"anomaly_detected", true}};

    bool run(const json& step, int index, bool checks_enabled = true) {
        return lab_machine::run_protocol_step(
            step, index, "run7",
            [this](const json& s) { dispatched.push_back(s); return dispatch_ok; },
            checks_enabled ? lab_machine::TipCheckFn([this](const std::string& label) {
                checked.push_back(label);
                return check_result;
            }) : lab_machine::TipCheckFn(),
            [this](const json& r) { published.push_back(r); });
    }
};

const json ASPIRATE = {{"action", "aspirate"}, {"resource_name", "tube_rack"},
                       {"well_indices", {0}}, {"volumes", {200}}};

}  // namespace

TEST(ProtocolStep, SuccessfulAspirateIsCheckedAndPublished) {
    StepRun s;
    EXPECT_TRUE(s.run(ASPIRATE, 1));
    ASSERT_EQ(s.dispatched.size(), 1u);
    ASSERT_EQ(s.checked.size(), 1u);
    ASSERT_EQ(s.published.size(), 1u);
    const json& r = s.published[0];
    EXPECT_EQ(r["type"], "tip_check");
    EXPECT_EQ(r["id"], "run7-1");
    EXPECT_EQ(r["protocol_step"], 1);
    EXPECT_EQ(r["aspirate"], ASPIRATE);
    EXPECT_TRUE(r["anomaly_detected"].get<bool>());
}

TEST(ProtocolStep, ClogVerdictDoesNotFailTheStep) {
    StepRun s;
    s.check_result = {{"status", "done"}, {"anomaly_detected", true}};
    EXPECT_TRUE(s.run(ASPIRATE, 0));
}

TEST(ProtocolStep, CheckErrorIsPublishedAndTheStepStillSucceeds) {
    StepRun s;
    s.check_result = {{"status", "error"}, {"error", "tip-check service unreachable"}};
    EXPECT_TRUE(s.run(ASPIRATE, 0));
    ASSERT_EQ(s.published.size(), 1u);
    EXPECT_EQ(s.published[0]["status"], "error");
}

TEST(ProtocolStep, FailedAspirateIsNotCheckedAndFailsTheStep) {
    StepRun s;
    s.dispatch_ok = false;
    EXPECT_FALSE(s.run(ASPIRATE, 0));
    EXPECT_TRUE(s.checked.empty());
    EXPECT_TRUE(s.published.empty());
}

TEST(ProtocolStep, OtherActionsAreNotChecked) {
    StepRun s;
    EXPECT_TRUE(s.run({{"action", "dispense"}}, 2));
    EXPECT_TRUE(s.run({{"action", "pick_up_tips"}}, 0));
    EXPECT_TRUE(s.checked.empty());
    EXPECT_TRUE(s.published.empty());
    EXPECT_EQ(s.dispatched.size(), 2u);
}

TEST(ProtocolStep, DisabledCheckLeavesAspirateAlone) {
    StepRun s;
    EXPECT_TRUE(s.run(ASPIRATE, 0, /*checks_enabled=*/false));
    EXPECT_TRUE(s.checked.empty());
    EXPECT_TRUE(s.published.empty());
}

TEST(ProtocolStep, AspirateMarkedTipCheckFalseIsNotChecked) {
    StepRun s;
    json step = ASPIRATE;
    step["tip_check"] = false;  // e.g. clear liquid the tip camera cannot see
    EXPECT_TRUE(s.run(step, 0));
    EXPECT_TRUE(s.checked.empty());
    EXPECT_TRUE(s.published.empty());
    ASSERT_EQ(s.dispatched.size(), 1u);
    EXPECT_EQ(s.dispatched[0], ASPIRATE);  // the flag never reaches the wrapper
}

TEST(ProtocolStep, AspirateMarkedTipCheckTrueIsCheckedWithoutTheFlag) {
    StepRun s;
    json step = ASPIRATE;
    step["tip_check"] = true;
    EXPECT_TRUE(s.run(step, 0));
    ASSERT_EQ(s.checked.size(), 1u);
    EXPECT_EQ(s.dispatched[0], ASPIRATE);
    EXPECT_EQ(s.published[0]["aspirate"], ASPIRATE);
}

TEST(ProtocolStep, CheckLabelDescribesTheAspirate) {
    StepRun s;
    s.run(ASPIRATE, 3);
    EXPECT_NE(s.checked[0].find("tube_rack"), std::string::npos);
    EXPECT_NE(s.checked[0].find("200"), std::string::npos);
}

// ── Progress while the checks run (shown live on the GCS) ──────────────────────

TEST(TipCheckProgress, ReportsEachPassAndTheSamplesSoFar) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"clip_id", "mine"}, {"status", "queued"}, {"queue_position", 2}}));
    http.answer("/check", ok(done_job(0.3, "mine")));
    http.answer("/jobs", ok(json::array({{{"clip_id", "mine"}, {"status", "running"}, {"stage", "vlm"}}})));
    http.answer("/jobs", ok(json::array({done_job(0.1, "mine")})));
    TipCheckClient client = make_client(http, clock, 2);
    std::vector<json> seen;
    client.on_progress([&seen](const json& p) { seen.push_back(p); });
    json r = client.run("x");
    ASSERT_EQ(r["status"], "done");

    ASSERT_GE(seen.size(), 5u);
    EXPECT_EQ(seen[0]["pass"], 1);
    EXPECT_EQ(seen[0]["passes"], 2);
    EXPECT_EQ(seen[0]["status"], "submitting");
    EXPECT_EQ(seen[1]["status"], "queued");
    EXPECT_EQ(seen[1]["queue_position"], 2);
    EXPECT_EQ(seen[2]["status"], "running");
    EXPECT_EQ(seen[2]["stage"], "vlm");
    // pass 1 finished: its probability is in the samples when pass 2 starts
    auto pass2 = std::find_if(seen.begin(), seen.end(), [](const json& p) { return p["pass"] == 2; });
    ASSERT_NE(pass2, seen.end());
    EXPECT_EQ((*pass2)["status"], "submitting");
    EXPECT_EQ((*pass2)["samples"].size(), 1u);
    EXPECT_EQ(seen.back()["samples"].size(), 2u);
    EXPECT_NEAR(seen.back()["samples"][0].get<double>(), 0.1, 1e-9);
}

TEST(TipCheckProgress, RepeatedIdenticalPollsAreReportedOnce) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok({{"clip_id", "mine"}, {"status", "running"}}));
    http.answer("/jobs", ok(json::array({{{"clip_id", "mine"}, {"status", "running"}}})));
    http.answer("/jobs", ok(json::array({{{"clip_id", "mine"}, {"status", "running"}}})));
    http.answer("/jobs", ok(json::array({done_job(0.2, "mine")})));
    TipCheckClient client = make_client(http, clock, 1);
    int running = 0;
    client.on_progress([&running](const json& p) { if (p["status"] == "running") ++running; });
    client.run("x");
    EXPECT_EQ(running, 1);
}

TEST(TipCheckProgress, NoCallbackIsFine) {
    FakeHttp http;
    Clock clock;
    http.answer("/check", ok(done_job(0.2)));
    EXPECT_EQ(make_client(http, clock, 1).run("x")["status"], "done");
}

TEST(Ot2Progress, NamesTheCommandAndItsAction) {
    json steps = json::array({{{"action", "pick_up_tips"}}, ASPIRATE});
    json p = lab_machine::ot2_progress("run7", "command", steps, 1);
    EXPECT_EQ(p["run_id"], "run7");
    EXPECT_EQ(p["phase"], "command");
    EXPECT_EQ(p["command"], 1);
    EXPECT_EQ(p["commands"], 2);
    EXPECT_EQ(p["action"], "aspirate");
    EXPECT_TRUE(p["tip_check"].is_null());
}

TEST(Ot2Progress, PhasesWithoutACommandHaveNulls) {
    json steps = json::array({ASPIRATE});
    json p = lab_machine::ot2_progress("r", "homing", steps);
    EXPECT_TRUE(p["command"].is_null());
    EXPECT_TRUE(p["action"].is_null());
    EXPECT_EQ(p["commands"], 1);
    EXPECT_TRUE(lab_machine::ot2_progress("r", "command", steps, 5)["command"].is_null());
    EXPECT_EQ(lab_machine::ot2_progress("r", "connecting", json())["commands"], 0);
}

TEST(Ot2Progress, CarriesTheTipCheckProgress) {
    json steps = json::array({ASPIRATE});
    json p = lab_machine::ot2_progress("r", "tip_check", steps, 0, {{"pass", 3}, {"passes", 5}});
    EXPECT_EQ(p["tip_check"]["pass"], 3);
}
