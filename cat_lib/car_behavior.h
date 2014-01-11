
void forward_motor(void);
void backward_motor(void);
void stop_motor(void);

typedef enum{
	CAR_STATE_IDLE,
	CAR_STATE_MOVE_FORWARD,
	CAR_STATE_MOVE_BACK,
	CAR_STATE_MOVE_BOTH,
	CAR_STATE_MOVING,
	CAR_STATE_REST,
}car_state_t;

void init_car(void);

