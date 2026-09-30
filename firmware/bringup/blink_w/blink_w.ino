// PLAN 2.3 "done when a blink sketch runs": first power-up of the Pico 2 W over USB, nothing else connected.
// The W's LED hangs off the wireless chip, so THIS sketch alone is built for rpipico2w (the core starts the
// chip for it); the real firmware is built for the plain Pico 2 and leaves the radio off (firmware/build.sh).
//   arduino-cli compile --fqbn rp2040:rp2040:rpipico2w firmware/bringup/blink_w -u -p /dev/cu.usbmodemXXXX
// First upload: hold BOOTSEL while plugging in USB; the Pico appears as a drive and arduino-cli finds it.
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    static uint32_t n = 0;
    digitalWrite(LED_BUILTIN, HIGH); delay(250);
    digitalWrite(LED_BUILTIN, LOW);  delay(250);
    Serial.printf("blink %lu: RP2350 at %lu MHz\n", (unsigned long)++n, (unsigned long)(rp2040.f_cpu() / 1000000));
}
