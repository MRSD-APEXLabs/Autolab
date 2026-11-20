// Copyright (c) 2024 Carnegie Mellon University
// MIT License — see LICENSE for details.
//
// Tip check (d3:4b clog detection) after each OT-2 aspirate. No ROS here, so it can be tested
// with a fake HTTP client. Same rules as the debug UI's tipCheckApi.js and the FVD deck:
//   POST /check {"step": label}; a queued answer is followed on GET /jobs by its clip_id;
//   N checks in a row, mean p_tip_empty, clog when mean >= threshold; the model and threshold
//   must not change between checks. Problems come back as status "error", never as exceptions.

#pragma once

#include <functional>
#include <string>

#include <nlohmann/json.hpp>

namespace lab_machine {

struct HttpResponse {
    bool sent;          // false: no answer at all (connection refused, timeout, …)
    long code;          // HTTP status
    std::string body;
    std::string error;  // transport error text when !sent
};

class HttpClient {
   public:
    virtual ~HttpClient() = default;
    virtual HttpResponse post(const std::string& url, const std::string& json_body) = 0;
    virtual HttpResponse get(const std::string& url) = 0;
};

struct TipCheckConfig {
    std::string base_url;     // e.g. http://192.168.10.19:8110 (empty: checks disabled)
    int passes = 5;
    double timeout_s = 900;   // per check
    double poll_s = 1.0;      // between GET /jobs while a check is queued
};

class TipCheckClient {
   public:
    using Sleep = std::function<void(double seconds)>;
    using Now = std::function<double()>;  // seconds, monotonic
    // Called as the checks go: {"pass": 1-based, "passes", "status" (submitting | queued |
    // running | … | done), "stage" (service's stage, or null), "queue_position" (or null),
    // "samples" (P(empty) of the finished checks)}. Only when something changed.
    using Progress = std::function<void(const nlohmann::json& progress)>;

    TipCheckClient(HttpClient& http, TipCheckConfig cfg, Sleep sleep, Now now);

    void on_progress(Progress progress) { progress_ = std::move(progress); }

    // Runs cfg.passes checks and returns the averaged verdict, or {"status": "error", ...}.
    nlohmann::json run(const std::string& label);

   private:
    nlohmann::json one_check(const std::string& label, int pass);  // throws std::runtime_error
    nlohmann::json parse(const HttpResponse& r, const std::string& what);
    void report(int pass, const nlohmann::json& job);

    nlohmann::json samples_so_far_ = nlohmann::json::array();
    nlohmann::json last_progress_;
    Progress progress_;

    HttpClient& http_;
    TipCheckConfig cfg_;
    Sleep sleep_;
    Now now_;
};

using DispatchFn = std::function<bool(const nlohmann::json& step)>;
using TipCheckFn = std::function<nlohmann::json(const std::string& label)>;  // empty: disabled
using PublishFn = std::function<void(const nlohmann::json& result)>;

// One OT-2 protocol step: dispatch it, and after a successful aspirate run the tip check and
// publish its result ({"type": "tip_check", "id": "<run_id>-<index>", ...}). The check never
// fails the step; only the dispatch result does.
bool run_protocol_step(const nlohmann::json& step, int index, const std::string& run_id,
                       const DispatchFn& dispatch, const TipCheckFn& tip_check,
                       const PublishFn& publish);

// Live OT-2 progress (topic ot2_progress), what the GCS shows while the protocol runs:
// {"run_id", "phase" (connecting | command | tip_check | homing | disconnecting | done | failed),
//  "command": 0-based index or null, "commands": count, "action": that command's action or null,
//  "tip_check": the client's progress during a tip check, else null}.
nlohmann::json ot2_progress(const std::string& run_id, const std::string& phase,
                            const nlohmann::json& steps, int command = -1,
                            const nlohmann::json& tip_check = nlohmann::json());

}  // namespace lab_machine
