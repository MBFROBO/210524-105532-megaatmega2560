#pragma once 

#include <servo_angle_restrictions.h>
#include <Arduino.h>
#include <Servo.h>

#define SERVO_LIMIT_MAX         1
#define SERVO_LIMIT_MIN         2
#define SERVO_LIMIT_NORMAL      3

Servo servo_RHAND_FINGER_1;
Servo servo_RHAND_FINGER_2;
Servo servo_RHAND_FINGER_3;
Servo servo_RHAND_FINGER_4;
Servo servo_RHAND_FINGER_5;
Servo servo_RHAND_WRIST;

Servo servo_LHAND_FINGER_1;
Servo servo_LHAND_FINGER_2;
Servo servo_LHAND_FINGER_3;
Servo servo_LHAND_FINGER_4;
Servo servo_LHAND_FINGER_5;
Servo servo_LHAND_WRIST;

Servo servo_RHAND_BICEPS;
Servo servo_RSHOUL_ROTAT;
Servo servo_RSHOUL_UD;
Servo servo_RSHOUL_TILT;

Servo servo_LHAND_BICEPS;
Servo servo_LSHOUL_ROTAT;
Servo servo_LSHOUL_UD;
Servo servo_LSHOUL_TILT;

Servo servo_HEAD_EYE_ROTAT;
Servo servo_HEAD_EYE_TILT;
Servo servo_HEAD_MOUTH;
Servo servo_HEAD_ROTAT;

Servo servo_NECK_RIGHT;
Servo servo_NECK_LEFT;
Servo servo_NECK_MID;

void right_arm_control(uint8_t data[], uint8_t state[]) {

    for (uint8_t i = 0; i < 6; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] > RHAND_FINGER_1_MAX) {
        data[0] = RHAND_FINGER_1_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] < RHAND_FINGER_1_MIN) {
        data[0] = RHAND_FINGER_1_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] > RHAND_FINGER_2_MAX) {
        data[1] = RHAND_FINGER_2_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] < RHAND_FINGER_2_MIN) {
        data[1] = RHAND_FINGER_2_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] > RHAND_FINGER_3_MAX) {
        data[2] = RHAND_FINGER_3_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] < RHAND_FINGER_3_MIN) {
        data[2] = RHAND_FINGER_3_MIN;
        state[2] = SERVO_LIMIT_MIN;
    } 

    if (data[3] > RHAND_FINGER_4_MAX) {
        data[3] = RHAND_FINGER_4_MAX;
        state[3] = SERVO_LIMIT_MAX;
    } else if (data[3] < RHAND_FINGER_4_MIN) {
        data[3] = RHAND_FINGER_4_MIN;
        state[3] = SERVO_LIMIT_MIN;
    }

    if (data[4] > RHAND_FINGER_5_MAX) {
        data[4] = RHAND_FINGER_5_MAX;
        state[4] = SERVO_LIMIT_MAX;
    } else if (data[4] < RHAND_FINGER_5_MIN) {
        data[4] = RHAND_FINGER_5_MIN;
        state[4] = SERVO_LIMIT_MIN;
    }

    if (data[5] > RHAND_WRIST_MAX) {
        data[5] = RHAND_WRIST_MAX;
        state[5] = SERVO_LIMIT_MAX;
    } else if (data[5] < RHAND_WRIST_MIN) {
        data[5] = RHAND_WRIST_MIN;
        state[5] = SERVO_LIMIT_MIN;
    }

    servo_RHAND_FINGER_1.attach(RHAND_FINGER_1_PIN);
    servo_RHAND_FINGER_2.attach(RHAND_FINGER_2_PIN);
    servo_RHAND_FINGER_3.attach(RHAND_FINGER_3_PIN);
    servo_RHAND_FINGER_4.attach(RHAND_FINGER_4_PIN);
    servo_RHAND_FINGER_5.attach(RHAND_FINGER_5_PIN);
    servo_RHAND_WRIST.attach(RHAND_WRIST_PIN);

    servo_RHAND_FINGER_1.write(data[0]);
    servo_RHAND_FINGER_2.write(data[1]);
    servo_RHAND_FINGER_3.write(data[2]);
    servo_RHAND_FINGER_4.write(data[3]);
    servo_RHAND_FINGER_5.write(data[4]);
    servo_RHAND_WRIST.write(data[5]);

}

void left_arm_control(uint8_t data[], uint8_t state[]) {

    for (uint8_t i = 0; i < 6; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] > LHAND_FINGER_1_MAX) {
        data[0] = LHAND_FINGER_1_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] < LHAND_FINGER_1_MIN) {
        data[0] = LHAND_FINGER_1_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] > LHAND_FINGER_2_MAX) {
        data[1] = LHAND_FINGER_2_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] < LHAND_FINGER_2_MIN) {
        data[1] = LHAND_FINGER_2_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] > LHAND_FINGER_3_MAX) {
        data[2] = LHAND_FINGER_3_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] < LHAND_FINGER_3_MIN) {
        data[2] = LHAND_FINGER_3_MIN;
        state[2] = SERVO_LIMIT_MIN;
    } 

    if (data[3] > LHAND_FINGER_4_MAX) {
        data[3] = LHAND_FINGER_4_MAX;
        state[3] = SERVO_LIMIT_MAX;
    } else if (data[3] < LHAND_FINGER_4_MIN) {
        data[3] = LHAND_FINGER_4_MIN;
        state[3] = SERVO_LIMIT_MIN;
    }

    if (data[4] > LHAND_FINGER_5_MAX) {
        data[4] = LHAND_FINGER_5_MAX;
        state[4] = SERVO_LIMIT_MAX;
    } else if (data[4] < LHAND_FINGER_5_MIN) {
        data[4] = LHAND_FINGER_5_MIN;
        state[4] = SERVO_LIMIT_MIN;
    }

    if (data[5] > LHAND_WRIST_MAX) {
        data[5] = LHAND_WRIST_MAX;
        state[5] = SERVO_LIMIT_MAX;
    } else if (data[5] < LHAND_WRIST_MIN) {
        data[5] = LHAND_WRIST_MIN;
        state[5] = SERVO_LIMIT_MIN;
    }

    servo_LHAND_FINGER_1.attach(LHAND_FINGER_1_PIN);
    servo_LHAND_FINGER_2.attach(LHAND_FINGER_2_PIN);
    servo_LHAND_FINGER_3.attach(LHAND_FINGER_3_PIN);
    servo_LHAND_FINGER_4.attach(LHAND_FINGER_4_PIN);
    servo_LHAND_FINGER_5.attach(LHAND_FINGER_5_PIN);
    servo_LHAND_WRIST.attach(LHAND_WRIST_PIN);

    servo_LHAND_FINGER_1.write(data[0]);
    servo_LHAND_FINGER_2.write(data[1]);
    servo_LHAND_FINGER_3.write(data[2]);
    servo_LHAND_FINGER_4.write(data[3]);
    servo_LHAND_FINGER_5.write(data[4]);
    servo_LHAND_WRIST.write(data[5]);
}

void right_shoulder_control(uint8_t data[], uint8_t state[]) {
    for (uint8_t i = 0; i < 4; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] < RHAND_BICEPS_MAX) {
        data[0] = RHAND_BICEPS_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] > RHAND_BICEPS_MIN) {
        data[0] = RHAND_BICEPS_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] < RSHOUL_ROTAT_MAX) {
        data[1] = RSHOUL_ROTAT_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] > RSHOUL_ROTAT_MIN) {
        data[1] = RSHOUL_ROTAT_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] < RSHOUL_UD_MAX) {
        data[2] = RSHOUL_UD_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] > RSHOUL_UD_MIN) {
        data[2] = RSHOUL_UD_MIN;
        state[2] = SERVO_LIMIT_MIN;
    } 

    if (data[3] < RSHOUL_TILT_MAX) {
        data[3] = RSHOUL_TILT_MAX;
        state[3] = SERVO_LIMIT_MAX;
    } else if (data[3] > RSHOUL_TILT_MIN) {
        data[3] = RSHOUL_TILT_MIN;
        state[3] = SERVO_LIMIT_MIN;
    }

    servo_RHAND_BICEPS.attach(RHAND_BICEPS_PIN);
    servo_RSHOUL_ROTAT.attach(RSHOUL_ROTAT_PIN);
    servo_RSHOUL_UD.attach(RSHOUL_UD_PIN);
    servo_RSHOUL_TILT.attach(RSHOUL_TILT_PIN);

    servo_RHAND_BICEPS.write(data[0]);
    servo_RSHOUL_ROTAT.write(data[1]);
    servo_RSHOUL_UD.write(data[2]);
    servo_RSHOUL_TILT.write(data[3]);
}

void left_shoulder_control(uint8_t data[], uint8_t state[]) {
    for (uint8_t i = 0; i < 4; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] < LHAND_BICEPS_MAX) {
        data[0] = LHAND_BICEPS_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] > LHAND_BICEPS_MIN) {
        data[0] = LHAND_BICEPS_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] < LSHOUL_ROTAT_MAX) {
        data[1] = LSHOUL_ROTAT_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] > LSHOUL_ROTAT_MIN) {
        data[1] = LSHOUL_ROTAT_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] < LSHOUL_UD_MAX) {
        data[2] = LSHOUL_UD_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] > LSHOUL_UD_MIN) {
        data[2] = LSHOUL_UD_MIN;
        state[2] = SERVO_LIMIT_MIN;
    } 

    if (data[3] < LSHOUL_TILT_MAX) {
        data[3] = LSHOUL_TILT_MAX;
        state[3] = SERVO_LIMIT_MAX;
    } else if (data[3] > LSHOUL_TILT_MIN) {
        data[3] = LSHOUL_TILT_MIN;
        state[3] = SERVO_LIMIT_MIN;
    }

    servo_LHAND_BICEPS.attach(LHAND_BICEPS_PIN);
    servo_LSHOUL_ROTAT.attach(LSHOUL_ROTAT_PIN);
    servo_LSHOUL_UD.attach(LSHOUL_UD_PIN);
    servo_LSHOUL_TILT.attach(LSHOUL_TILT_PIN);

    servo_LHAND_BICEPS.write(data[0]);
    servo_LSHOUL_ROTAT.write(data[1]);
    servo_LSHOUL_UD.write(data[2]);
    servo_LSHOUL_TILT.write(data[3]);
}

void head_control(uint8_t data[], uint8_t state[]) {
    for (uint8_t i = 0; i < 4; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] > HEAD_MOUTH_MAX) {
        data[0] = HEAD_MOUTH_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] < HEAD_MOUTH_MIN) {
        data[0] = HEAD_MOUTH_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] > HEAD_EYE_ROTAT_MAX) {
        data[1] = HEAD_EYE_ROTAT_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] < HEAD_EYE_ROTAT_MIN) {
        data[1] = HEAD_EYE_ROTAT_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] > HEAD_EYE_TILT_MAX) {
        data[2] = HEAD_EYE_TILT_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] < HEAD_EYE_TILT_MIN) {
        data[2] = HEAD_EYE_TILT_MIN;
        state[2] = SERVO_LIMIT_MIN;
    } 

    if (data[3] > HEAD_ROTAT_MAX) {
        data[3] = HEAD_ROTAT_MAX;
        state[3] = SERVO_LIMIT_MAX;
    } else if (data[3] < HEAD_ROTAT_MIN) {
        data[3] = HEAD_ROTAT_MIN;
        state[3] = SERVO_LIMIT_MIN;
    }

    servo_HEAD_MOUTH.attach(HEAD_MOUTH_PIN);
	servo_HEAD_EYE_ROTAT.attach(HEAD_EYE_ROTAT_PIN);
	servo_HEAD_EYE_TILT.attach(HEAD_EYE_TILT_PIN);
    servo_HEAD_ROTAT.attach(HEAD_ROTAT_PIN);

    servo_HEAD_MOUTH.write(data[0]);
    servo_HEAD_EYE_ROTAT.write(data[1]);
    servo_HEAD_EYE_TILT.write(data[2]);
    servo_HEAD_ROTAT.write(data[3]);
}

void neck_control(uint8_t data[], uint8_t state[]) {



    for (uint8_t i = 0; i < 3; i++) {
        state[i] = SERVO_LIMIT_NORMAL;
    }

    if (data[0] > NECK_RIGHT_MAX) {
        data[0] = NECK_RIGHT_MAX;
        state[0] = SERVO_LIMIT_MAX;
    } else if (data[0] < NECK_RIGHT_MIN) {
        data[0] = NECK_RIGHT_MIN;
        state[0] = SERVO_LIMIT_MIN;
    } 

    if (data[1] > NECK_LEFT_MAX) {
        data[1] = NECK_LEFT_MAX;
        state[1] = SERVO_LIMIT_MAX;
    } else if (data[1] < NECK_LEFT_MIN) {
        data[1] = NECK_LEFT_MIN;
        state[1] = SERVO_LIMIT_MIN;
    } 

    if (data[2] > NECK_MID_MAX) {
        data[2] = NECK_MID_MAX;
        state[2] = SERVO_LIMIT_MAX;
    } else if (data[2] < NECK_MID_MIN) {
        data[2] = NECK_MID_MIN;
        state[2] = SERVO_LIMIT_MIN;
    }

    servo_NECK_RIGHT.attach(NECK_RIGHT_PIN);
	servo_NECK_LEFT.attach(NECK_LEFT_PIN);
	servo_NECK_MID.attach(NECK_MID_PIN);

    servo_NECK_RIGHT.write(data[0]);
	servo_NECK_LEFT.write(data[1]);
	servo_NECK_MID.write(data[2]);
}

void servo_speed_control(uint8_t current_pose, uint8_t previous_pose) {
    

}