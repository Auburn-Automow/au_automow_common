uint8 BATTERY_DISCHARGING = 0
uint8 BATTERY_CHARGING_RECOVERY = 1
uint8 BATTERY_CHARGING = 2
uint8 BATTERY_TRICKLE_CHARGING = 3
uint8 BATTERY_DISCHARGING_CRITICAL = 4
uint8 BATTERY_ERROR = 5

uint8 battery_state

uint16 voltage  # mV
int16 current   # mA
uint8 charge    # %

float32 temperature_1
float32 temperature_2

bool cutter_1
bool cutter_2
