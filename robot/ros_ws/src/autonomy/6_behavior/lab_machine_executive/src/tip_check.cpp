// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.

#include <lab_machine_executive/tip_check.hpp>

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

using nlohmann::json;

namespace lab_machine {

namespace {

const std::set<std::string> PENDING = {"queued", "running", "pending", "processing"};

std::string fmt2(double v) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(2);
    s << v;
    return s.str();
}

bool is_probability(const json& v) {
    return v.is_number() && std::isfinite(v.get<double>()) && v.get<double>() >= 0 &&
           v.get<double>() <= 1;
}

}  // namespace

TipCheckClient::TipCheckClient(HttpClient& http, TipCheckConfig cfg, Sleep sleep, Now now)
    : http_(http), cfg_(std::move(cfg)), sleep_(std::move(sleep)), now_(std::move(now)) {}

json TipCheckClient::parse(const HttpResponse& r, const std::string& what) {
    if (!r.sent)
        throw std::runtime_error("tip-check service unreachable at " + cfg_.base_url + " (" +
                                 r.error + ")");
    json body = json::parse(r.body, nullptr, /*allow_exceptions=*/false);
    if (r.code < 200 || r.code >= 300) {
        std::string message = "HTTP " + std::to_string(r.code);
        if (body.is_object() && body.contains("error") && body["error"].is_string())
            message = body["error"].get<std::string>();
        throw std::runtime_error("clog check failed: " + message);
    }
    if (body.is_discarded()) throw std::runtime_error("unreadable " + what + " response");
    return body;
}

void TipCheckClient::report(int pass, const json& job) {
    if (!progress_) return;
    auto field = [&job](const char* key) {
        return job.is_object() && job.contains(key) && !job[key].is_null() ? job[key] : json();
    };
    json p = {{"pass", pass + 1},
              {"passes", cfg_.passes},
              {"status", job.is_object() ? job.value("status", "submitting") : "submitting"},
              {"stage", field("stage")},
              {"queue_position", field("queue_position")},
              {"samples", samples_so_far_}};
    if (p == last_progress_) return;
    last_progress_ = p;
    progress_(p);
}

json TipCheckClient::one_check(const std::string& label, int pass) {
    report(pass, json());
    json job = parse(http_.post(cfg_.base_url + "/check", json{{"step", label}}.dump()), "/check");
    if (!job.is_object()) throw std::runtime_error("unreadable /check response");
    report(pass, job);
    const json clip = job.value("clip_id", json());
    const double deadline = now_() + cfg_.timeout_s;
    while (job.value("status", "") != "done") {
        const std::string status = job.value("status", "");
        if (!PENDING.count(status)) {
            std::string error = job.contains("error") && job["error"].is_string()
                                    ? job["error"].get<std::string>()
                                    : "ended with status " + status;
            throw std::runtime_error("clog check failed: " + error);
        }
        if (!clip.is_string() || clip.get<std::string>().empty())
            throw std::runtime_error("clog check returned no job ID");
        if (now_() >= deadline) throw std::runtime_error("clog check did not finish in time");
        sleep_(cfg_.poll_s);
        json jobs = parse(http_.get(cfg_.base_url + "/jobs"), "/jobs");
        if (jobs.is_object()) jobs = jobs.value("jobs", json::array());
        if (!jobs.is_array()) throw std::runtime_error("unreadable clog-check job list");
        for (const auto& item : jobs)
            if (item.is_object() && item.value("clip_id", json()) == clip) job = item;
        report(pass, job);
    }
    return job;
}

json TipCheckClient::run(const std::string& label) {
    std::vector<json> samples;
    samples_so_far_ = json::array();
    last_progress_ = json();
    try {
        for (int i = 0; i < cfg_.passes; ++i) {
            json job = one_check(label, i);
            if (!is_probability(job.value("p_tip_empty", json())))
                throw std::runtime_error("check " + std::to_string(i + 1) + " of " +
                                         std::to_string(cfg_.passes) +
                                         " returned no valid tip-empty probability");
            samples.push_back(job);
            samples_so_far_.push_back(job["p_tip_empty"]);
            report(i, job);
        }
        std::set<std::string> models;
        std::vector<double> thresholds;
        for (const auto& j : samples) {
            if (j.contains("model") && j["model"].is_string() && !j["model"].get<std::string>().empty())
                models.insert(j["model"].get<std::string>());
            const json t = j.value("threshold", json(0.5));
            thresholds.push_back(t.is_number() ? t.get<double>() : NAN);
        }
        if (models.size() > 1) throw std::runtime_error("the model changed during the checks");
        for (double t : thresholds)
            if (!std::isfinite(t) || t < 0 || t > 1 || t != thresholds[0])
                throw std::runtime_error("the detection threshold changed during the checks");
    } catch (const std::exception& e) {
        return {{"status", "error"}, {"error", e.what()}, {"sample_count", cfg_.passes},
                {"samples_done", samples.size()}};
    }

    const double n = static_cast<double>(samples.size());
    double sum = 0, visible = 0, total = 0;
    bool all_visible = true;
    json probs = json::array();
    std::vector<std::string> warnings;
    for (const auto& j : samples) {
        const double p = j["p_tip_empty"].get<double>();
        sum += p;
        probs.push_back(p);
        if (is_probability(j.value("p_tip_visible", json())))
            visible += j["p_tip_visible"].get<double>();
        else
            all_visible = false;
        if (j.contains("warning") && j["warning"].is_string() && !j["warning"].get<std::string>().empty())
            warnings.push_back(j["warning"].get<std::string>());
        const json t = j.value("timings_s", json::object());
        if (t.is_object() && t.value("total", json()).is_number()) total += t["total"].get<double>();
    }
    const double mean = sum / n;
    const double threshold = samples[0].value("threshold", json(0.5)).get<double>();
    const bool anomaly = mean >= threshold;
    const json& first = samples[0];

    json result = {
        {"status", "done"},
        {"clip_id", first.value("clip_id", json())},
        {"model", first.value("model", json())},
        {"threshold", threshold},
        {"aggregation", "mean"},
        {"sample_count", samples.size()},
        {"samples", probs},
        {"p_tip_empty", mean},
        {"p_tip_visible", all_visible ? json(visible / n) : json()},
        {"anomaly_detected", anomaly},
        {"confidence", anomaly ? mean : 1 - mean},
        {"summary", "Average of " + std::to_string(samples.size()) + " checks: " +
                        (anomaly ? "tip looks empty" : "liquid in tip") + " (P(empty)=" +
                        fmt2(mean) + ")"},
        {"warning", warnings.empty()
                        ? json()
                        : json(std::to_string(warnings.size()) + " of " +
                               std::to_string(samples.size()) + " checks reported a warning. " +
                               warnings[0])},
        {"error", json()},
        {"files", first.value("files", json::object())},
        {"timings_s", {{"total", total}}},
    };
    return result;
}

bool run_protocol_step(const json& step, int index, const std::string& run_id,
                       const DispatchFn& dispatch, const TipCheckFn& tip_check,
                       const PublishFn& publish) {
    if (!dispatch(step)) return false;
    if (step.value("action", "") != "aspirate" || !tip_check) return true;

    json result = tip_check("OT-2 aspirate " + step.dump());
    if (!result.is_object()) result = {{"status", "error"}, {"error", "no tip-check result"}};
    result["type"] = "tip_check";
    result["id"] = run_id + "-" + std::to_string(index);
    result["protocol_step"] = index;
    result["aspirate"] = step;
    publish(result);
    return true;
}

json ot2_progress(const std::string& run_id, const std::string& phase, const json& steps,
                  int command, const json& tip_check) {
    const bool has_command =
        steps.is_array() && command >= 0 && command < static_cast<int>(steps.size());
    json action;
    if (has_command && steps[command].is_object() && steps[command].contains("action"))
        action = steps[command]["action"];
    return {{"run_id", run_id},
            {"phase", phase},
            {"command", has_command ? json(command) : json()},
            {"commands", steps.is_array() ? steps.size() : 0},
            {"action", action},
            {"tip_check", tip_check}};
}

}  // namespace lab_machine
