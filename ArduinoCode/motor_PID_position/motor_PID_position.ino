#define IN1 14
#define IN2 15
#define ENA 2
#define ENB 3
#define PWM 9
#define STB 11

// Encoder variables
long long int ticks = 0;
// PID variables
float error, error_prev, error_sum = 0.0;
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
// Motor characteristics variables
float gearRatio = 100.0;
float encoderResolution = 12.0;

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, INPUT);
    pinMode(ENB, INPUT);
    pinMode(PWM, OUTPUT);
    pinMode(STB, OUTPUT);
    digitalWrite(STB, HIGH);

    Serial.begin(9600);
    //interrupt setup
    attachInterrupt(digitalPinToInterrupt(ENA), readEncoder, RISING);

    // PID setup
    error_prev = 0.0;
}

void loop()
{
    PID_position_control(90.0); // rotate 90 degrees
}

// The target value is in the unit of angle
void PID_position_control(float target)
{
    // transform the target value to the unit of ticks
    error = target / 360.0 * gearRatio * encoderResolution - (float)ticks;
    error_sum += error;
    // the driving signal
    float u = Kp * error + Ki * error_sum + Kd * (error - error_prev);
    // determine the direction of the motor
    int direction = 1;
    if (u < 0)
    {
        direction = -1;
        u = -1 * u;
    }
    else
    {
        direction = 1;
    }
    // saturate the driving signal to the range of 0 to 255
    if (u > 255) u = 255;
    // drive the motor
    driveMotor((int)u, direction);
    // update the error for D controller
    error_prev = error;
    // print the current position in angle
    Serial.println((float)ticks / gearRatio / encoderResolution * 360.0);
}

void driveMotor(int pwmSignal, int direction)
{
    if (direction == 1)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(PWM, pwmSignal);
    }
    else if (direction == -1)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(PWM, pwmSignal);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(PWM, 0);
    }
}

void readEncoder()
{
    // if the ENB pin is high, the motor is rotating in the positive direction
    if (digitalRead(ENB)) ticks++;
    else ticks--;
}