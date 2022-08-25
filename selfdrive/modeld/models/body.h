#pragma once

#include <vector>

#include "cereal/messaging/messaging.h"
#include "common/util.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"

#define OUTPUT_SIZE 2
#define INPUT_SIZE 6

typedef struct BodyModelResult {
  float torque_left;
  float torque_right;
  float execution_time;
} BodyModelResult;

typedef struct BodyModelState {
  RunModel *m;
  float net_input_buf[INPUT_SIZE];
  float output[OUTPUT_SIZE];
} BodyModelState;

void bodymodel_init(BodyModelState* s);
BodyModelResult bodymodel_eval_frame(BodyModelState* s, float* input_buf);
void bodymodel_free(BodyModelState* s);
