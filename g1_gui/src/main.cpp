#include <cstdio>
#include <cstring>
#include <atomic>
#include <mutex>
#include <string>

#include "mongoose.h"

#include "g1_gui/joint_names.hpp"

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

static const std::string kHgStateTopic = "rt/lowstate";
static const std::string kCustomCtrlCmdTopic = "rt/custom_controller_cmd";
static constexpr double kLowStateTickDtSec = 0.001;  // 500 Hz lowstate

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1U << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000U) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }
  return CRC32;
}

struct JointSnapshot {
  bool valid = false;
  double sim_time_s = 0.0;
  std::array<float, kG1NumMotor> q{};
  std::array<float, kG1NumMotor> dq{};
  std::array<float, 3> imu_rpy{};
  uint8_t mode_machine = 0;
};

struct App {
  mg_mgr mgr{};
  std::string web_root;
  ChannelPublisherPtr<std_msgs::msg::dds_::String_> custom_ctrl_cmd_pub;
  std::mutex snap_mu;
  JointSnapshot snap;
  std::atomic<bool> tick_origin_initialized{false};
  std::atomic<uint32_t> tick_origin{0};
};

static void append_json_float(std::string *out, float v) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
  *out += buf;
}

static std::string snapshot_to_json(const JointSnapshot &s) {
  std::string out;
  out.reserve(4096);
  out += "{\"type\":\"joint_state\",\"valid\":";
  out += s.valid ? "true" : "false";
  out += ",\"elapsed_s\":";
  char tbuf[32];
  std::snprintf(tbuf, sizeof(tbuf), "%.3f", s.sim_time_s);
  out += tbuf;
  out += ",\"mode_machine\":";
  std::snprintf(tbuf, sizeof(tbuf), "%u", static_cast<unsigned>(s.mode_machine));
  out += tbuf;
  out += ",\"imu_rpy\":[";
  append_json_float(&out, s.imu_rpy[0]);
  out += ',';
  append_json_float(&out, s.imu_rpy[1]);
  out += ',';
  append_json_float(&out, s.imu_rpy[2]);
  out += "],\"q\":[";
  for (int i = 0; i < kG1NumMotor; ++i) {
    if (i) {
      out += ',';
    }
    append_json_float(&out, s.q[static_cast<size_t>(i)]);
  }
  out += "],\"dq\":[";
  for (int i = 0; i < kG1NumMotor; ++i) {
    if (i) {
      out += ',';
    }
    append_json_float(&out, s.dq[static_cast<size_t>(i)]);
  }
  out += "]}";
  return out;
}

static void broadcast_websocket(struct mg_mgr *mgr, const std::string &json) {
  for (struct mg_connection *c = mgr->conns; c != nullptr; c = c->next) {
    if (c->is_websocket != 0U) {
      mg_ws_send(c, json.data(), json.size(), WEBSOCKET_OP_TEXT);
    }
  }
}

static void on_timer(void *arg) {
  auto *app = static_cast<App *>(arg);
  std::string json;
  {
    std::lock_guard<std::mutex> lock(app->snap_mu);
    json = snapshot_to_json(app->snap);
  }
  broadcast_websocket(&app->mgr, json);
}

static void http_event(struct mg_connection *c, int ev, void *ev_data) {
  auto *app = static_cast<App *>(c->fn_data);
  if (ev == MG_EV_HTTP_MSG) {
    auto *hm = static_cast<struct mg_http_message *>(ev_data);
    if (mg_match(hm->uri, mg_str("/ws"), nullptr) != 0) {
      mg_ws_upgrade(c, hm, nullptr);
      return;
    }
    struct mg_http_serve_opts opts = {};
    opts.root_dir = app->web_root.c_str();
    mg_http_serve_dir(c, hm, &opts);
  } else if (ev == MG_EV_WS_MSG) {
    auto *wm = static_cast<struct mg_ws_message *>(ev_data);
    const std::string ws_text(wm->data.buf, wm->data.len);
    if (ws_text == "custom_controller_start" && app->custom_ctrl_cmd_pub) {
      std_msgs::msg::dds_::String_ cmd_msg;
      cmd_msg.data() = "custom_controller_start";
      app->custom_ctrl_cmd_pub->Write(cmd_msg);
      static constexpr const char kAck[] = "{\"type\":\"ack\",\"cmd\":\"custom_controller_start\"}";
      mg_ws_send(c, kAck, sizeof(kAck) - 1, WEBSOCKET_OP_TEXT);
    }
  }
}

static void low_state_handler(App *app, const void *message) {
  LowState_ low_state = *static_cast<const LowState_ *>(message);
  if (low_state.crc() != Crc32Core(reinterpret_cast<uint32_t *>(&low_state), (sizeof(LowState_) >> 2) - 1)) {
    return;
  }

  JointSnapshot s;
  s.valid = true;
  uint32_t origin_tick = 0;
  if (!app->tick_origin_initialized.load(std::memory_order_acquire)) {
    const uint32_t initial_tick = low_state.tick();
    app->tick_origin.store(initial_tick, std::memory_order_release);
    app->tick_origin_initialized.store(true, std::memory_order_release);
    origin_tick = initial_tick;
  } else {
    origin_tick = app->tick_origin.load(std::memory_order_acquire);
  }
  const uint32_t tick_now = low_state.tick();
  const uint32_t rel_tick = (tick_now >= origin_tick) ? (tick_now - origin_tick) : 0;
  s.sim_time_s = static_cast<double>(rel_tick) * kLowStateTickDtSec;
  s.mode_machine = low_state.mode_machine();
  const auto &imu = low_state.imu_state();
  s.imu_rpy = {imu.rpy()[0], imu.rpy()[1], imu.rpy()[2]};
  for (int i = 0; i < kG1NumMotor; ++i) {
    s.q[static_cast<size_t>(i)] = low_state.motor_state()[i].q();
    s.dq[static_cast<size_t>(i)] = low_state.motor_state()[i].dq();
  }
  {
    std::lock_guard<std::mutex> lock(app->snap_mu);
    app->snap = std::move(s);
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s <network_interface> [http_port]\n", argv[0]);
    std::fprintf(stderr, "  Example: %s lo\n", argv[0]);
    std::fprintf(stderr, "  Open http://127.0.0.1:<port>/ in a browser (default port 4710).\n");
    return 1;
  }

  const std::string iface = argv[1];
  int port = 4710;
  if (argc >= 3) {
    port = std::atoi(argv[2]);
    if (port <= 0 || port > 65535) {
      std::fprintf(stderr, "Invalid port\n");
      return 1;
    }
  }

  ChannelFactory::Instance()->Init(0, iface);

  App app;
  app.web_root = G1_GUI_WEB_ROOT;
  app.custom_ctrl_cmd_pub.reset(
      new ChannelPublisher<std_msgs::msg::dds_::String_>(kCustomCtrlCmdTopic));
  app.custom_ctrl_cmd_pub->InitChannel();

  mg_mgr_init(&app.mgr);
  app.mgr.userdata = &app;

  char listen_url[64];
  std::snprintf(listen_url, sizeof(listen_url), "http://0.0.0.0:%d", port);
  if (mg_http_listen(&app.mgr, listen_url, http_event, &app) == nullptr) {
    std::fprintf(stderr, "Failed to listen on %s (web root: %s)\n", listen_url, app.web_root.c_str());
    mg_mgr_free(&app.mgr);
    return 1;
  }

  mg_timer_add(&app.mgr, 50, MG_TIMER_REPEAT | MG_TIMER_RUN_NOW, on_timer, &app);

  ChannelSubscriberPtr<LowState_> state_sub;
  state_sub.reset(new ChannelSubscriber<LowState_>(kHgStateTopic));
  state_sub->InitChannel(
      [&app](const void *msg) { low_state_handler(&app, msg); }, 1);

  std::printf("[g1_gui_server] DDS on interface \"%s\", HTTP/WS %s, web root %s\n", iface.c_str(), listen_url,
              app.web_root.c_str());

  for (;;) {
    mg_mgr_poll(&app.mgr, 20);
  }

  return 0;
}
