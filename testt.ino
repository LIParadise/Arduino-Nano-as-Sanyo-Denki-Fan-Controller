#include "string.h"

static const uint8_t ButtonTurnDownFanPin = A5;
static const uint8_t ButtonTurnUpFanPin = 10;
static const uint8_t ButtonFanMiddleSpinPin = 4;
static const uint8_t ReadPwmViaInterrupt = 2;
static const uint8_t FanAmpleIndicator = 12;

/*
 * For Timer2 OC2B for 25 kHz PWM
 */
static const uint8_t ControlFanPWMPin = 3;              // Timer2 OC2A on Arduino Nano
static const auto TCCR2A__ = _BV(COM2B1) | _BV(WGM20);  // Phase Correct PWM allows for easy 0% PWM
static const auto TCCR2B__ = _BV(CS21) | _BV(WGM22);    // No scaling (16 MHz), use OC2A to fine tune resulting freq.
static const unsigned OCR2A__ = 79;                     // Both for PWM granularity and down-scaling to 25 kHz.
static const unsigned DEFAULT_PWM_DUTY = OCR2A__ / 4;
static const unsigned AMPLE_PWM_DUTY = OCR2A__ / 5;
/*
 * Above is Timer2 OC2B PWM config.
 */

// Volatile data for ISR
static volatile unsigned long falling_edges = 0;
void pwm_counter_isr() {
    falling_edges += 1;
}

static const int BufLen = 255;
static const unsigned long DelayTime = 20;  // In ms, determines the loop delay

class FanControlState {
   private:
    typedef enum FanControl {
        Up,
        Down,
        ConstantMiddle,
        None,
    } FanControl;
    FanControl fc;
    unsigned state;
    unsigned short clock;
    unsigned short clock_tick;
    static const unsigned short period;

   public:
    FanControlState() : fc(None), state(0), clock(0), clock_tick(1) {}
    void change_state(bool up_button, bool down_button, bool const_button) {
        if ((fc == Down && down_button) || (fc == Up && up_button)) {
            if (state == period) {
                clock_tick = 6;
            } else {
                ++state;
            }
        } else if ((fc == None || fc == ConstantMiddle) && const_button) {
            fc = ConstantMiddle;
        } else if (fc == None && (up_button || down_button)) {
            state = 1;
            fc = up_button ? Up : Down;
        } else {
            fc = None;
            state = 0;
            clock_tick = 1;
            clock = 0;
        }
    }
    void apply_pwm() {
        if (clock == 0) {
            if (fc == Down) {
                OCR2B = (OCR2B > 0) ? OCR2B - 1 : 0;
            } else if (fc == Up) {
                OCR2B = (OCR2B < OCR2A__) ? OCR2B + 1 : OCR2A__;
            } else if (fc == ConstantMiddle) {
                OCR2B = DEFAULT_PWM_DUTY;
            }
        }
    }
    inline void tick() {
        if (fc != None) {
            clock = (clock < period - clock_tick) ? clock + clock_tick : 0;
        }
    }
    void dump(char* const buf) {
        char tmp_c = '\0';
        if (Up == fc) {
            tmp_c = 'U';
        } else if (Down == fc) {
            tmp_c = 'D';
        } else if (ConstantMiddle == fc) {
            tmp_c = 'M';
        } else {
            tmp_c = 'N';
        }
        sprintf(buf, "state%d, clock%d, clock_tick%d, fc%c", state, clock, clock_tick, tmp_c);
    }
};

const unsigned short FanControlState::period = 36;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ButtonTurnDownFanPin, INPUT);
    pinMode(ButtonTurnUpFanPin, INPUT);
    pinMode(ButtonFanMiddleSpinPin, INPUT_PULLUP);
    pinMode(ControlFanPWMPin, OUTPUT);
    pinMode(ReadPwmViaInterrupt, INPUT);
    pinMode(FanAmpleIndicator, OUTPUT);

    // Sanyo Denki Tachometer uses open collector.
    attachInterrupt(digitalPinToInterrupt(ReadPwmViaInterrupt), pwm_counter_isr, FALLING);

    /* Setup PWM Output
     *
     * Sanyo Denki fans use 25 kHz signal to do PWM
     * whereas Arduino Nano uses 490/980 Hz depending on which pin.
     *
     * Use Phase Correct PWM
     * since on Arduino Nano Fast PWM doesn't allow 0% PWM
     * without some hacking: need to turn off PWM to do that.
     * It's possible but just more work.
     *
     * Here we use Timer2 since Timer0 is used for clock for several internal
     * library functions while Timer1 is used for the Servo library.
     */
    // Override OC2A to Phase Correct PWM Mode
    TCCR2A = TCCR2A__;
    // No scaling, Arduino is 16M
    // Phase Correct, OCRA as TOP s.t. output is 25K.
    TCCR2B = TCCR2B__;
    OCR2A = OCR2A__;
    OCR2B = DEFAULT_PWM_DUTY;  // initial PWM set at around 25% duty

    Serial.begin(9600);
}

void loop() {
    static int counter = 0;
    static char buf[BufLen];
    static FanControlState fcs;
    static const unsigned serial_period = 80;
    static unsigned serial_clock = 0;

    const bool UpButton = digitalRead(ButtonTurnUpFanPin) == HIGH;
    const bool DownButton = digitalRead(ButtonTurnDownFanPin) == HIGH;
    const bool ConstButton = digitalRead(ButtonFanMiddleSpinPin) == LOW;

    fcs.change_state(UpButton, DownButton, ConstButton);
    fcs.apply_pwm();

    if (0 == serial_clock) {
        const unsigned reg = OCR2B;
        memset(buf, 0, BufLen);
        noInterrupts();
        const unsigned long fes = falling_edges;
        falling_edges = 0;
        interrupts();
        const unsigned long rpm = (fes * 30000ul) / ((unsigned long)serial_period * (unsigned long)DelayTime);
        sprintf(buf, "OCR2B=%u, UpButton%d, DownButton%d, ConstButton%d, fes%d, rpm%04X_%ld, ", reg, UpButton, DownButton, ConstButton, fes, rpm, rpm);
        // fcs.dump(buf + strlen(buf));
        digitalWrite(FanAmpleIndicator, (reg >= AMPLE_PWM_DUTY) ? HIGH : LOW);
        Serial.println(buf);
    }

    serial_clock = (serial_clock < serial_period - 1) ? serial_clock + 1 : 0;
    fcs.tick();
    delay(DelayTime);
}