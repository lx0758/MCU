#include "encoder.h"
#include <string.h>

#define ENCODER_MAX_CB               20

typedef struct {
    int cb_num;
    Encoder_InitTypeDef *encoder[ENCODER_MAX_CB];
} Encoder_CallbackTypeDef;
static Encoder_CallbackTypeDef encoder_callback;

uint8_t ENCODER_Init(Encoder_InitTypeDef *encoder) {
    if (encoder_callback.cb_num >= ENCODER_MAX_CB) return ERROR;
    if (encoder->frequency == 0) {
        encoder->frequency = 2;
    }
    encoder_callback.encoder[encoder_callback.cb_num] = encoder;
    encoder_callback.cb_num++;
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
    return SUCCESS;
}

uint8_t ENCODER_DeInit(Encoder_InitTypeDef *encoder) {
    for (int i = 0; i < encoder_callback.cb_num; i++) {
        if (encoder_callback.encoder[i]->htim == encoder->htim) {
            if (i != (encoder_callback.cb_num - 1)) {
                memmove(&encoder_callback.encoder[i], &encoder_callback.encoder[i + 1], (encoder_callback.cb_num - i - 1) * sizeof(*encoder));
            }
            encoder_callback.cb_num--;
            return SUCCESS;
        }
    }
    return ERROR;
}

void ENCODER_Scan() {
    Encoder_InitTypeDef *encoder;
    for (int i = 0; i < encoder_callback.cb_num; i++) {
        encoder = encoder_callback.encoder[i];
        encoder->count += (int16_t) __HAL_TIM_GET_COUNTER(encoder->htim);
        __HAL_TIM_SET_COUNTER(encoder->htim, 0);
        if (encoder->count >= encoder->frequency || encoder->count <= -encoder->frequency) {
            encoder->cb(encoder->count / encoder->frequency);
            encoder->count = encoder->count % encoder->frequency;
        }
    }
}