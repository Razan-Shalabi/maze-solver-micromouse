# DRV8833 Motor Driver

![DRV8833 2-channel motor driver](drv8833.jpeg)

## What is this?

A 2-channel H-bridge driver that lets the ESP32 (3.3V logic) control both N20 motors independently — forward, reverse, and speed via PWM — since the ESP32's GPIO pins can't drive motors directly.

## Hardware Connections

| DRV8833 Pin | Connects to | Role |
|---|---|---|
| IN1 | ESP32 GPIO 14 | Left motor, direction A |
| IN2 | ESP32 GPIO 12 | Left motor, direction B |
| IN3 | ESP32 GPIO 13 | Right motor, direction A |
| IN4 | ESP32 GPIO 15 | Right motor, direction B |

## How It Works

```cpp
void setMotor(int leftSpeed, int rightSpeed) {
  rightSpeed = -rightSpeed;   // right motor is mounted flipped
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0)      { analogWrite(IN1, leftSpeed); analogWrite(IN2, 0); }
  else if (leftSpeed < 0) { analogWrite(IN1, 0); analogWrite(IN2, -leftSpeed); }
  else                    { analogWrite(IN1, 0); analogWrite(IN2, 0); }

  // same pattern for IN3/IN4 with rightSpeed
}
```

- Each wheel is driven by a pair of pins — PWM goes on whichever pin matches the desired direction, the other pin goes LOW
- Speed range is `-255` to `255` (negative = reverse)
- `rightSpeed` is inverted before use because the right motor is physically mounted in the opposite orientation to the left one — without this flip, "forward" on the right wheel would spin it backward

## Used In

- `setMotor(leftSpeed, rightSpeed)` is the single point everything else calls — PID output, wall correction, and turning all funnel through it
- `stopMotorsOnly()` → `setMotor(0, 0)`

## Troubleshooting

**Motor spins the wrong way:**
- Swap that motor's two IN pins in wiring, or flip the sign in code for that side

**One wheel is weaker than the other at the same speed:**
- Check `BASE_CORRECTION` (currently `3`) — this exists specifically to compensate for a slight speed mismatch between the two motors

**No movement at all:**
- Confirm the driver has its separate motor power supply connected (DRV8833 logic and motor power are usually separate) — not just the ESP32's 3.3V line
