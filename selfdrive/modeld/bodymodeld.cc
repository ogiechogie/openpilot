#include <sys/resource.h>
#include <limits.h>

#include <cstdio>
#include <cstdlib>

#include "cereal/visionipc/visionipc_client.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "selfdrive/modeld/models/body.h"

ExitHandler do_exit;

void run_model(BodyModelState &model, VisionIpcClient &vipc_client) {
  PubMaster pm({"bodyModel"});
  SubMaster sm({"carState", "carControl"});
  float inputs[INPUT_SIZE];

  while (!do_exit) {
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    sm.update(0);
    if (sm.updated("carState")) {
      inputs[0] = sm["carState"].getCarState().getWheelSpeeds().getFl();
      inputs[1] = sm["carState"].getCarState().getWheelSpeeds().getFr();
      inputs[2] = sm["carState"].getCarState().getVEgo();
      inputs[3] = sm["carState"].getCarState().getAEgo();
    }
    if (sm.updated("carControl")) {
      auto orientation = sm["carControl"].getCarControl().getOrientationNED();
      auto angular_velocity = sm["carControl"].getCarControl().getAngularVelocity();
      if (orientation.size() >= 2) { inputs[4] = orientation[1]; }
      if (angular_velocity.size() >= 2) { inputs[5] = angular_velocity[1]; }
    }

    double t1 = millis_since_boot();
    BodyModelResult res = bodymodel_eval_frame(&model, inputs);
    double t2 = millis_since_boot();

    // send dm packet
    MessageBuilder msg;
    auto framed = msg.initEvent().initBodyModel();
    framed.setFrameId(extra.frame_id);
    framed.setModelExecutionTime((t2 - t1) / 1000.0);
    framed.setTorqueLeft(res.torque_left);
    framed.setTorqueRight(res.torque_right);
    pm.send("bodyModel", msg);
  }
}

int main(int argc, char **argv) {
  setpriority(PRIO_PROCESS, 0, -15);

  // init the models
  BodyModelState model;
  bodymodel_init(&model);

  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_WIDE_ROAD, true);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  if (vipc_client.connected) {
    LOGW("connected with buffer size: %d", vipc_client.buffers[0].len);
    run_model(model, vipc_client);
  }

  bodymodel_free(&model);
  return 0;
}