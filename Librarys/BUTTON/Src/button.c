#include <string.h>
#include "button.h"

#define BUTTON_MAX_CB               20
#define BUTTON_LONG_PRESS_TIME_OUT  1000

#define BUTTON_STATE_INIT           0
#define BUTTON_STATE_MAYBE          1
#define BUTTON_STATE_PRESS          2
#define BUTTON_STATE_LONG_PRESS     3

typedef struct {
    int cb_num;
    Button_InitTypeDef *button[BUTTON_MAX_CB];
} btn_cb_t;
static btn_cb_t btn_cb;

uint8_t BUTTON_Init(Button_InitTypeDef *btn) {
    if (btn_cb.cb_num >= BUTTON_MAX_CB) return ERROR;
    btn_cb.button[btn_cb.cb_num] = btn;
    btn_cb.cb_num++;
    return SUCCESS;
}

uint8_t BUTTON_DeInit(Button_InitTypeDef *btn) {
    for (int i = 0; i < btn_cb.cb_num; i++) {
        if (btn_cb.button[i]->type == btn->type && btn_cb.button[i]->pin == btn->pin) {
            if (i != (btn_cb.cb_num - 1)) {
                memmove(&btn_cb.button[i], &btn_cb.button[i + 1], (btn_cb.cb_num - i - 1) * sizeof(*btn));
            }
            btn_cb.cb_num--;
            return SUCCESS;
        }
    }
    return ERROR;
}

void BUTTON_Scan() {
    GPIO_PinState state;
    Button_InitTypeDef *button;
    for (int i = 0; i < btn_cb.cb_num; i++) {
        button = btn_cb.button[i];
        state = HAL_GPIO_ReadPin(button->type, button->pin);
        switch (button->state) {
            case BUTTON_STATE_INIT:
                if (state == button->target) {
                    button->state = BUTTON_STATE_MAYBE;
                }
                break;
            case BUTTON_STATE_MAYBE:
                if (state == button->target) {
                    button->state = BUTTON_STATE_PRESS;
                    button->press_time = HAL_GetTick();
                    if (button->event != NULL) {
                        button->event(BUTTON_EVENT_DOWN);
                    }
                } else {
                    button->state = BUTTON_STATE_INIT;
                }
                break;
            case BUTTON_STATE_PRESS:
                if (state == button->target) {
                    if (HAL_GetTick() - button->press_time > BUTTON_LONG_PRESS_TIME_OUT) {
                        button->state = BUTTON_STATE_LONG_PRESS;
                        if (button->long_press != NULL ) {
                            button->long_press();
                        }
                    }
                } else {
                    if (button->press != NULL) {
                        button->press();
                    }
                    if (button->event != NULL) {
                        button->event(BUTTON_EVENT_UP);
                    }
                }
                break;
            case BUTTON_STATE_LONG_PRESS:
                if (state != button->target) {
                    if (button->event != NULL) {
                        button->event(BUTTON_EVENT_UP);
                    }
                }
                break;
        }
        if (state != button->target) {
            button->state = BUTTON_STATE_INIT;
        }
    }
}