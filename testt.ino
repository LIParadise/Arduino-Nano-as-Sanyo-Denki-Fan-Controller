#include <TM1637Display.h>

#include "string.h"

static const uint8_t ButtonTurnDownFanPin = A5;
static const uint8_t TM1637Toggle = A4;
static const uint8_t ButtonTurnUpFanPin = 10;
static const uint8_t ButtonFanMiddleSpinPin = 4;
static const uint8_t ReadPwmViaInterrupt = 2;
static const uint8_t FanAmpleIndicator = 12;
static const uint8_t TM1637CLK = 5;
static const uint8_t TM1637DIO = 6;
TM1637Display tm1637 = TM1637Display(TM1637CLK, TM1637DIO, 80);
// const uint8_t tm1637_void_pattern[] = {0x00, 0x00, 0x00, 0x00};
const uint8_t tm1637_all_pattern[] = {0xff, 0xff, 0xff, 0xff};

/*
 * For Timer2 OC2B for 25 kHz PWM
 */
static const uint8_t ControlFanPWMPin = 3;              // Timer2 OC2A on Arduino Nano
static const auto TCCR2A__ = _BV(COM2B1) | _BV(WGM20);  // Phase Correct PWM allows for easy 0% PWM
static const auto TCCR2B__ = _BV(CS21) | _BV(WGM22);    // No scaling (16 MHz), use OC2A to fine tune resulting freq.
static const unsigned MAX_DUTY_OCR2A = 40;              // Both for PWM granularity and down-scaling to 25 kHz.
static const unsigned DEFAULT_PWM_DUTY = MAX_DUTY_OCR2A / 4;
static const unsigned AMPLE_PWM_DUTY = MAX_DUTY_OCR2A / 5;
/*
 * Above is Timer2 OC2B PWM config.
 */

// Volatile data for ISR
static volatile unsigned long falling_edges = 0;
void pwm_counter_isr() {
    falling_edges += 1;
}

// static const int BufLen = 255;
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
    const unsigned short period;

   public:
    FanControlState(const unsigned short period) : fc(None), state(0), clock(0), clock_tick(1), period(period) {}
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
                OCR2B = (OCR2B < MAX_DUTY_OCR2A) ? OCR2B + 1 : MAX_DUTY_OCR2A;
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

class Control1637State {
   public:
    Control1637State(const unsigned short db, const unsigned short sw) : debounce_thres(db), sticky_thres(sw), debounce(0), sticky(0), disp_now(On), disp_prev(On) {}
    void change_state(bool button) {
        if (button) {
            if (debounce == 0) {
                if (sticky > 0) {
                    // sticky key
                    // user is switching type of display, rather than switching on/off
                    // restore previous display on/off information
                    disp_now = disp_prev;
                    dk = (RPM == dk) ? Duty : RPM;
                    sticky = 0;
                } else {
                    disp_prev = disp_now;
                    disp_now = MaySwitch;
                }
            }
            debounce = debounce_thres;
        }
    }
    void tick() {
        if (debounce > 0) {
            if (1 == debounce) {
                debounce = 0;
                sticky = sticky_thres;
            } else {
                debounce -= 1;
            }
        } else if (sticky > 0) {
            sticky = (1 == sticky) ? 0 : sticky - 1;
            if (0 == sticky && MaySwitch == disp_now) {
                disp_now = (On == disp_prev) ? Off : On;
            }
        }
    }
    void apply_display(TM1637Display& tm1637, const unsigned long& rpm, const unsigned& duty_reg) const {
        if (On == disp_now || (On == disp_prev && MaySwitch == disp_now)) {
            switch (dk) {
                case RPM:
                    tm1637.showNumberDec((int)rpm);
                    break;
                case Duty:
                    static const uint8_t percent_segments = SEG_A | SEG_C | SEG_D | SEG_F;
                    // upper-left, up, lower-right, bottom
                    int duty = (int)((unsigned long)duty_reg * 1000ul / (unsigned long)MAX_DUTY_OCR2A / 10ul);
                    if (duty > 100) {
                        duty = 100;
                    } else if (duty < 0) {
                        duty = 0;
                    }
                    tm1637.showNumberDec(duty, false, 3, 0);
                    tm1637.setSegments(&percent_segments, 1, 3);
                    break;
            }
        } else {
            tm1637.clear();
        }
    }

   private:
    typedef enum DisplayKind {
        RPM,
        Duty,
    } DisplayKind;
    DisplayKind dk;
    typedef enum DisplayOnOff {
        Off,
        MaySwitch,
        On,
    } DisplayOnOff;
    DisplayOnOff disp_now;
    DisplayOnOff disp_prev;
    unsigned short debounce;
    unsigned short sticky;
    const unsigned short debounce_thres;
    const unsigned short sticky_thres;
};

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ButtonTurnDownFanPin, INPUT);
    pinMode(ButtonTurnUpFanPin, INPUT);
    pinMode(ButtonFanMiddleSpinPin, INPUT_PULLUP);
    pinMode(TM1637Toggle, INPUT_PULLUP);
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
    OCR2A = MAX_DUTY_OCR2A;
    OCR2B = DEFAULT_PWM_DUTY;  // initial PWM set at around 25% duty

    // Serial.begin(9600);
    tm1637.setBrightness(0, true);
    tm1637.clear();
}

void loop() {
    static int counter = 0;
    // static char buf[BufLen];
    static FanControlState fcs(36);
    static Control1637State c1s(3, 400 / DelayTime);
    static const unsigned clock_period = 72;
    static unsigned clock = 0;

    const bool UpButton = digitalRead(ButtonTurnUpFanPin) == HIGH;
    const bool DownButton = digitalRead(ButtonTurnDownFanPin) == HIGH;
    const bool ConstButton = digitalRead(ButtonFanMiddleSpinPin) == LOW;
    const bool TM1637Button = digitalRead(TM1637Toggle) == LOW;

    fcs.change_state(UpButton, DownButton, ConstButton);
    c1s.change_state(TM1637Button);
    fcs.apply_pwm();

    if (0 == clock) {
        const unsigned reg = OCR2B;
        // memset(buf, 0, BufLen);
        noInterrupts();
        const unsigned long fes = falling_edges;
        falling_edges = 0;
        interrupts();
        const unsigned long rpm = (fes * 30000ul) / ((unsigned long)clock_period * (unsigned long)DelayTime);
        // sprintf(buf, "OCR2B=%u, UpButton%d, DownButton%d, ConstButton%d, fes%d, rpm%04X_%ld, ", reg, UpButton, DownButton, ConstButton, fes, rpm, rpm);
        // fcs.dump(buf + strlen(buf));
        digitalWrite(FanAmpleIndicator, (reg >= AMPLE_PWM_DUTY) ? HIGH : LOW);
        // Serial.println(buf);
        if (0 == rpm) {
            tm1637.setSegments(tm1637_all_pattern);
        } else {
            c1s.apply_display(tm1637, rpm, reg);
        }
    }

    clock = (clock < clock_period - 1) ? clock + 1 : 0;
    fcs.tick();
    c1s.tick();
    delay(DelayTime);
}