#include <cstring>

#include "common/mat.h"
#include "common/params.h"
#include "common/timing.h"
#include "system/hardware/hw.h"
#include "selfdrive/modeld/models/body.h"

void bodymodel_init(BodyModelState* s) {
  s->m = new SNPEModel("models/bodycontrol.dlc", &s->output[0], OUTPUT_SIZE, USE_DSP_RUNTIME);
}

BodyModelResult bodymodel_eval_frame(BodyModelState* s, float* input_buf) {
  for (int i=0; i<INPUT_SIZE; i++) {
    s->net_input_buf[i] = input_buf[i];
  }

  double t1 = millis_since_boot();
  s->m->addImage(s->net_input_buf, INPUT_SIZE);
  s->m->execute();
  double t2 = millis_since_boot();

  BodyModelResult ret = {0};
  ret.torque_left = s->output[0];
  ret.torque_right = s->output[1];
  ret.execution_time = (t2 - t1) / 1000.;
  return ret;
}

void bodymodel_free(BodyModelState* s) {
  delete s->m;
}